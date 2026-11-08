"""Absolute heading from the BNO085 magnetometer, without ROS.

Pipeline (all in the sensor frame bno_imu_link, field in microtesla):

  1. Vertical hard iron hz in the sensor frame, before levelling: B' = B - (0, 0, hz)
     (the horizontal part is left to the 2D centre c, see hard_xy_pre_level).
     The vertical part hz matters on a slope: after levelling it would leak
     into the horizontal as hz sin(tilt).  h comes from a linear regression on
     the RTK samples (see estimate_hard); hz needs tilt in the samples and is
     pulled to its previous value (0 at first) when the ground was flat.
  2. Levelling: roll/pitch from the game rotation vector (/bno085/imu, gyro +
     accelerometer, no magnetometer).  R = Rz(yaw) Ry(pitch) Rx(roll) (Euler
     'ZYX'); R_level = Ry(pitch) Rx(roll) = Rz(-yaw) R; Bl = R_level B'.
  3. Calibration (2D, in the levelled frame): Hc = W (Bl_xy - c), where c is
     what is left of the hard-iron centre and W maps the ellipse onto the unit
     circle (soft iron).  heading_raw = -atan2(Hc_y, Hc_x); yaw = heading_raw + offset.
  4. Smoothing without lag: the difference d = heading_raw - game_yaw changes
     slowly, so yaw = game_yaw_now + circmean_{last smooth_s}(d) + offset.
  5. Quality gate ("trust / do not trust"), all without RTK:
       dF   = |F - Fref| / Fref     F = hypot(|Hc| H0, Bz), H0 = mean semi-axis
       dI   = |incl - Iref|         incl = atan2(-Bz, |Hc| H0)
       gyro_std   = std(d) over gyro_std_window_s
       gyro_drift = change of circmean(d) over drift_window_s
  6. Self-calibration from the RTK heading: while the RTK heading is valid,
     levelled samples (decimated) are kept in bins by RTK yaw; once enough bins
     are covered, an ellipse is fitted to the per-bin medians, the offset to the
     RTK yaw is taken, samples with an odd field strength are dropped, and the
     result is accepted only when its residual against RTK is small.

Yaw convention: the same as /gnss_heading/navheading (ENU, Euler 'ZYX' yaw of
the orientation quaternion); the offset absorbs the mounting and the magnetic
declination, so the output can be compared with that topic directly.
"""
import datetime
import math
import os
import tempfile
import time
from collections import deque

import numpy as np

TWO_PI = 2.0 * math.pi
DEG = 180.0 / math.pi
CAL_VERSION = 1


def wrap(a):
    """Angle(s) to [-pi, pi)."""
    return (a + math.pi) % TWO_PI - math.pi


def circmean(a, weights=None, axis=None):
    """Circular mean (radians) of angles a."""
    z = np.exp(1j * np.asarray(a))
    if weights is None:
        return np.angle(np.mean(z, axis=axis))
    return np.angle(np.sum(z * weights, axis=axis))


def quat_yaw(qx, qy, qz, qw):
    """Yaw of Euler 'ZYX' (same as scipy Rotation.as_euler('ZYX')[0])."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_level(qx, qy, qz, qw):
    """(yaw, R_level as 3 row tuples) of an orientation quaternion, None when zero."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    yaw = math.atan2(r10, r00)
    c, s = math.cos(yaw), math.sin(yaw)
    return yaw, ((c * r00 + s * r10, c * r01 + s * r11, c * r02 + s * r12),
                 (-s * r00 + c * r10, -s * r01 + c * r11, -s * r02 + c * r12),
                 (r20, r21, r22))


def fit_ellipse(xy):
    """Ellipse through points xy (N,2): (centre, W, semi-axes) or None.

    W is symmetric positive definite and maps the ellipse onto the unit circle.
    """
    xy = np.asarray(xy, float)
    if len(xy) < 6 or not np.all(np.isfinite(xy)):
        return None
    mean = xy.mean(axis=0)               # centred: better conditioned
    x, y = xy[:, 0] - mean[0], xy[:, 1] - mean[1]
    D = np.c_[x * x, x * y, y * y, x, y]
    try:
        p, _, rank, _ = np.linalg.lstsq(D, np.ones_like(x), rcond=None)
        if rank < 5:
            return None          # points on a line / too few distinct directions
        A = np.array([[p[0], p[1] / 2], [p[1] / 2, p[2]]])
        c = -0.5 * np.linalg.solve(A, p[3:5])
    except np.linalg.LinAlgError:
        return None
    k = 1 + c @ A @ c
    if not np.isfinite(k) or abs(k) < 1e-12:
        return None
    ev, V = np.linalg.eigh(A / k)
    if np.any(ev <= 0):
        return None              # hyperbola / degenerate: not an ellipse
    W = V @ np.diag(np.sqrt(ev)) @ V.T
    c = c + mean
    hc = (xy - c) @ W.T
    if np.std(np.hypot(hc[:, 0], hc[:, 1])) > 0.25:
        return None              # the points are no ellipse
    return c, W, 1.0 / np.sqrt(ev)


def level(L, b):
    """R_level (...,9 row-major) applied to b (...,3)."""
    L = np.asarray(L, float).reshape(np.shape(b)[:-1] + (3, 3))
    return np.einsum('...ij,...j->...i', L, b)


class Calibration(object):
    """3D hard iron (sensor frame), 2D soft iron in the levelled frame, the
    references for the gate."""

    def __init__(self, center, W, radii, offset=0.0, source='uncal', Fref=None, Iref=None,
                 resid_p50_deg=None, resid_p90_deg=None, bins=0, samples=0, created=None,
                 hard=(0.0, 0.0, 0.0), best_p90_deg=None):
        self.hard = np.asarray(hard, float).reshape(3)
        self.center = np.asarray(center, float).reshape(2)
        self.W = np.asarray(W, float).reshape(2, 2)
        self.radii = np.asarray(radii, float).reshape(2)
        self.H0 = float(np.mean(self.radii))
        self.offset = float(offset)
        self.source = str(source)
        self.Fref = None if Fref is None else float(Fref)
        self.Iref = None if Iref is None else float(Iref)
        self.resid_p50_deg = resid_p50_deg
        self.resid_p90_deg = resid_p90_deg
        self.bins = int(bins)
        self.samples = int(samples)
        self.created = float(created) if created is not None else 0.0
        self.best_p90_deg = best_p90_deg if best_p90_deg is not None else resid_p90_deg
        self.check()

    def check(self):
        vals = [self.hard, self.center, self.W, self.radii, [self.offset]]
        vals += [[x] for x in (self.Fref, self.Iref) if x is not None]
        if not all(np.all(np.isfinite(v)) for v in vals):
            raise ValueError('calibration with a non-finite value')
        if np.any(self.radii <= 0) or np.any(np.linalg.eigvalsh((self.W + self.W.T) / 2) <= 0):
            raise ValueError('calibration with a degenerate ellipse')
        if self.Fref is not None and self.Fref <= 0:
            raise ValueError('calibration with Fref <= 0')

    def apply_raw(self, b, L):
        """Raw field b (...,3) and R_level (...,9) -> as apply()."""
        return self.apply(level(L, np.asarray(b, float) - self.hard))

    def apply(self, bl):
        """bl (...,3) levelled field -> heading_raw, F, inclination (rad), |Hc|."""
        bl = np.asarray(bl, float)
        v = bl[..., :2] - self.center
        hx = self.W[0, 0] * v[..., 0] + self.W[0, 1] * v[..., 1]
        hy = self.W[1, 0] * v[..., 0] + self.W[1, 1] * v[..., 1]
        hn = np.hypot(hx, hy)
        hut = hn * self.H0
        bz = bl[..., 2]
        return -np.arctan2(hy, hx), np.hypot(hut, bz), np.arctan2(-bz, hut), hn

    def to_dict(self):
        def r(x, n=4):
            return None if x is None else round(float(x), n)
        return {
            'version': CAL_VERSION,
            'source': self.source,
            'hard_uT': [r(v) for v in self.hard],
            'center_uT': [r(v) for v in self.center],
            'W': [[r(v, 6) for v in row] for row in self.W],
            'radii_uT': [r(v) for v in self.radii],
            'H0_uT': r(self.H0),
            'offset_deg': r(self.offset * DEG),
            'Fref_uT': r(self.Fref),
            'Iref_deg': r(None if self.Iref is None else self.Iref * DEG),
            'resid_p50_deg': r(self.resid_p50_deg, 2),
            'resid_p90_deg': r(self.resid_p90_deg, 2),
            'best_p90_deg': r(self.best_p90_deg, 2),
            'bins': self.bins,
            'samples': self.samples,
            'created': r(self.created, 1),
            'created_iso': datetime.datetime.fromtimestamp(self.created).isoformat(timespec='seconds')
            if self.created else None,
        }

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise ValueError('not a mapping')
        if int(d.get('version', 0)) != CAL_VERSION:
            raise ValueError('calibration version %r' % d.get('version'))
        return cls(d['center_uT'], d['W'], d['radii_uT'], math.radians(d['offset_deg']),
                   d['source'], d.get('Fref_uT'),
                   None if d.get('Iref_deg') is None else math.radians(d['Iref_deg']),
                   d.get('resid_p50_deg'), d.get('resid_p90_deg'),
                   d.get('bins', 0), d.get('samples', 0), d.get('created'),
                   d.get('hard_uT') or (0.0, 0.0, 0.0), d.get('best_p90_deg'))


def save_calibration(path, cal):
    """Atomic write (temporary file + rename in the same directory)."""
    import yaml
    path = os.path.expanduser(path)
    folder = os.path.dirname(path) or '.'
    os.makedirs(folder, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.mag_cal_', dir=folder)
    try:
        with os.fdopen(fd, 'w') as fh:
            fh.write('# vitulus_imu mag_heading calibration, written by the node\n')
            yaml.safe_dump(cal.to_dict(), fh, default_flow_style=None, sort_keys=False)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def load_calibration(path, source=None):
    """(Calibration or None, note).  A calibration of another source is refused."""
    import yaml
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None, 'no calibration file %s' % path
    try:
        with open(path) as fh:
            cal = Calibration.from_dict(yaml.safe_load(fh))
    except Exception as exc:  # noqa: BLE001 - a broken file means no calibration
        return None, 'calibration file %s unreadable: %s' % (path, exc)
    if source is not None and cal.source != source:
        return None, 'calibration in %s is for source %r, not %r' % (path, cal.source, source)
    return cal, 'loaded %s' % path


class Ring(object):
    """Fixed-size numpy ring of rows, oldest first on read; column 0 is time."""

    def __init__(self, cols, size):
        self.a = np.zeros((size, cols))
        self.i = 0
        self.n = 0

    def append(self, row):
        self.a[self.i] = row
        self.i = (self.i + 1) % len(self.a)
        self.n = min(self.n + 1, len(self.a))

    def clear(self):
        self.i = self.n = 0

    def rows(self):
        if self.n < len(self.a):
            return self.a[:self.n]
        return np.concatenate((self.a[self.i:], self.a[:self.i]))

    def since(self, t0):
        r = self.rows()
        return r[np.searchsorted(r[:, 0], t0, side='right'):]


class SampleBins(object):
    """Self-calibration samples by RTK yaw: a ring per bin.

    A sample: the raw field and R_level, the window of the last smooth_s (raw
    field, R_level, game-yaw change up to now: for the smoothed residual), the
    RTK yaw (carried by the gyro to the sample time) and its time.
    """

    def __init__(self, n_bins=36, per_bin=40, window_k=20):
        self.n_bins, self.per_bin, self.k = n_bins, per_bin, window_k
        self.b = np.zeros((n_bins, per_bin, 3))
        self.L = np.zeros((n_bins, per_bin, 9))
        self.win_b = np.zeros((n_bins, per_bin, window_k, 3))
        self.win_L = np.zeros((n_bins, per_bin, window_k, 9))
        self.win_dyaw = np.zeros((n_bins, per_bin, window_k))
        self.win_w = np.zeros((n_bins, per_bin, window_k))
        self.ref = np.zeros((n_bins, per_bin))
        self.t = np.zeros((n_bins, per_bin))
        self.count = np.zeros(n_bins, int)
        self.next = np.zeros(n_bins, int)

    def bin_of(self, yaw):
        return int(((wrap(yaw) + math.pi) / TWO_PI) * self.n_bins) % self.n_bins

    def add(self, t, b, L, win_b, win_L, win_dyaw, ref):
        k = self.bin_of(ref)
        j = self.next[k]
        self.b[k, j] = b
        self.L[k, j] = L
        m = min(len(win_b), self.k)
        self.win_b[k, j] = 0.0
        self.win_L[k, j] = 0.0
        self.win_b[k, j, :m] = win_b[:m]
        self.win_L[k, j, :m] = win_L[:m]
        self.win_dyaw[k, j, :m] = win_dyaw[:m]
        self.win_w[k, j] = 0.0
        self.win_w[k, j, :m] = 1.0 / m
        self.ref[k, j] = ref
        self.t[k, j] = t
        self.next[k] = (j + 1) % self.per_bin
        self.count[k] = min(self.count[k] + 1, self.per_bin)

    def clear(self):
        self.count[:] = 0
        self.next[:] = 0

    def coverage(self, min_per_bin=1):
        return int(np.sum(self.count >= min_per_bin))

    def total(self):
        return int(self.count.sum())

    def snapshot(self, min_t=None):
        """Flat copies of the filled samples newer than min_t."""
        mask = np.arange(self.per_bin)[None, :] < self.count[:, None]
        if min_t is not None:
            mask &= self.t >= min_t
        return {'b': self.b[mask].copy(), 'L': self.L[mask].copy(),
                'win_b': self.win_b[mask].copy(), 'win_L': self.win_L[mask].copy(),
                'win_dyaw': self.win_dyaw[mask].copy(), 'win_w': self.win_w[mask].copy(),
                'ref': self.ref[mask].copy(), 't': self.t[mask].copy(),
                'bin': np.nonzero(mask)[0]}


def smoothed_raw(cal, snap):
    """Smoothed heading_raw at each sample's time, as the live output computes it."""
    h = cal.apply_raw(snap['win_b'], snap['win_L'])[0]
    return circmean(h + snap['win_dyaw'], weights=snap['win_w'], axis=1)


def estimate_hard(b, L, ref, prior_hz=0.0, prior_tilt_deg=0.3):
    """3D hard iron (sensor frame) by linear least squares on RTK samples.

    Levelled field: L (b - h) = c' + A [cos psi, sin psi] with psi the RTK yaw,
    A a 2x2 matrix (soft iron, field strength, declination).  The x and y rows
    are linear in h (3) and A (4); c' is absorbed by h.  hz is only seen
    through tilt (columns L_x2, L_y2), so a weak prior pulls it to prior_hz:
    as strong as samples tilted by prior_tilt_deg rms.  None when singular.
    """
    n = len(ref)
    if n < 10:
        return None
    L = L.reshape(n, 3, 3)
    c, s = np.cos(ref), np.sin(ref)
    z = np.zeros(n)
    X = np.concatenate([np.c_[L[:, 0, :], c, s, z, z], np.c_[L[:, 1, :], z, z, c, s]])
    y = np.concatenate([np.einsum('ij,ij->i', L[:, 0, :], b), np.einsum('ij,ij->i', L[:, 1, :], b)])
    lam = math.sqrt(2.0 * n) * math.sin(math.radians(prior_tilt_deg))
    X = np.vstack([X, [0, 0, lam, 0, 0, 0, 0]])
    y = np.append(y, lam * prior_hz)
    u, _, rank, _ = np.linalg.lstsq(X, y, rcond=None)
    if rank < 7 or not np.all(np.isfinite(u)):
        return None
    return u[:3]


def _bin_medians(bl, bins, min_per_bin):
    pts = []
    for b in np.unique(bins):
        sel = bins == b
        if sel.sum() >= min_per_bin:
            pts.append(np.median(bl[sel, :2], axis=0))
    return np.array(pts)


def fit_calibration(snap, source, min_bins=30, min_per_bin=3, reject_dF_pct=12.0, reject_err_deg=20.0,
                    max_rejected=0.3, max_p90_deg=8.0, max_worse_deg=1.0, max_axis_ratio=3.0,
                    current=None, now=0.0, hard_xy=False):
    """Fit from SampleBins.snapshot(); dict with accepted, reason, cal, numbers.

    Three rounds: fit all -> drop odd field strength (dock, metal) -> fit, take
    the offset -> drop samples far off the RTK yaw (RTK glitch, local
    disturbance) -> final fit.  The residual is measured on the kept samples
    with the smoothing of the live output; too many dropped samples fail.
    """
    res = {'accepted': False, 'reason': '', 'cal': None, 'n': int(len(snap['ref'])),
           'coverage': 0, 'rejected': 0, 'p50_deg': None, 'p90_deg': None, 'current_p90_deg': None}
    if res['n'] == 0:
        res['reason'] = 'no samples'
        return res
    braw, Lraw, bins, ref = snap['b'], snap['L'], snap['bin'], snap['ref']
    keep = np.ones(len(ref), bool)
    cal = None
    same = current is not None and current.source == source
    prior_hz = float(current.hard[2]) if same else 0.0
    for it in range(3):
        hard = estimate_hard(braw[keep], Lraw[keep], ref[keep], prior_hz)
        if hard is None:
            hard = np.array([0.0, 0.0, prior_hz])
        if not hard_xy:
            hard = np.array([0.0, 0.0, hard[2]])   # horizontal part left to the 2D centre
        bl = level(Lraw, braw - hard)
        pts = _bin_medians(bl[keep], bins[keep], min_per_bin)
        res['coverage'] = len(pts)
        if len(pts) < min_bins:
            res['reason'] = 'coverage %d/%d bins' % (len(pts), min_bins)
            return res
        fit = fit_ellipse(pts)
        if fit is None:
            res['reason'] = 'ellipse fit failed'
            return res
        c, W, radii = fit
        if radii.max() / radii.min() > max_axis_ratio:
            res['reason'] = 'ellipse axis ratio %.1f' % (radii.max() / radii.min())
            return res
        try:
            cal = Calibration(c, W, radii, 0.0, source, hard=hard)
        except ValueError as exc:
            res['reason'] = str(exc)
            return res
        if it == 0:
            F = cal.apply(bl)[1]
            Fmed = np.median(F[keep])
            keep &= np.abs(F - Fmed) / Fmed * 100.0 < reject_dF_pct
        elif it == 1:
            est = smoothed_raw(cal, snap)
            off = circmean(wrap(ref[keep] - est[keep]))
            keep &= np.abs(wrap(est + off - ref)) * DEG < reject_err_deg
    res['rejected'] = int((~keep).sum())
    res['hard_uT'] = [round(float(v), 2) for v in cal.hard]
    _, F, incl, _ = cal.apply(bl)
    est = smoothed_raw(cal, snap)
    cal.offset = float(circmean(wrap(ref[keep] - est[keep])))
    cal.Fref = float(np.median(F[keep]))
    cal.Iref = float(np.median(incl[keep]))
    err = np.abs(wrap(est + cal.offset - ref))[keep] * DEG
    cal.resid_p50_deg = float(np.percentile(err, 50))
    cal.resid_p90_deg = float(np.percentile(err, 90))
    cal.bins, cal.samples, cal.created = res['coverage'], int(keep.sum()), now
    res.update(cal=cal, p50_deg=round(cal.resid_p50_deg, 2), p90_deg=round(cal.resid_p90_deg, 2))
    if not all(np.isfinite(x) for x in (cal.offset, cal.Fref, cal.Iref, cal.resid_p90_deg)):
        res['reason'] = 'non-finite fit'
        return res
    # Ratchet: not worse than the current calibration on these samples, nor than
    # the best residual so far, unless the current one no longer fits at all
    # (robot changed): then the lineage starts again.
    best = None
    if same:
        e_cur = np.abs(wrap(smoothed_raw(current, snap) + current.offset - ref))[keep] * DEG
        res['current_p90_deg'] = round(float(np.percentile(e_cur, 90)), 2)
        if res['current_p90_deg'] < max_p90_deg:
            best = current.best_p90_deg if current.best_p90_deg is not None else current.resid_p90_deg
        else:
            res['current_p90_deg'] = None
    res['best_p90_deg'] = None if best is None else round(float(best), 2)
    cal.best_p90_deg = cal.resid_p90_deg if best is None else min(float(best), cal.resid_p90_deg)
    share = res['rejected'] / float(res['n'])
    if share > max_rejected:
        res['reason'] = 'rejected %.0f %% of samples > %.0f %%' % (100 * share, 100 * max_rejected)
    elif cal.resid_p90_deg >= max_p90_deg:
        res['reason'] = 'residual p90 %.1f deg >= %.1f' % (cal.resid_p90_deg, max_p90_deg)
    elif res['current_p90_deg'] is not None and cal.resid_p90_deg > res['current_p90_deg'] + max_worse_deg:
        res['reason'] = 'worse than current (p90 %.1f vs %.1f deg)' % (
            cal.resid_p90_deg, res['current_p90_deg'])
    elif best is not None and cal.resid_p90_deg > best + max_worse_deg:
        res['reason'] = 'worse than the best so far (p90 %.1f vs %.1f deg)' % (cal.resid_p90_deg, best)
    else:
        res['accepted'] = True
        res['reason'] = 'accepted'
    return res


def summary(x):
    """median/p90/p99/max of |x| in degrees, None when empty."""
    if len(x) == 0:
        return None
    x = np.asarray(x)
    return {'n': int(len(x)), 'median': round(float(np.median(x)), 2),
            'p90': round(float(np.percentile(x, 90)), 2),
            'p99': round(float(np.percentile(x, 99)), 2), 'max': round(float(x.max()), 2)}


class MagHeadingEstimator(object):
    DEFAULTS = {
        'source': 'uncal',            # magnetometer report the calibration belongs to
        'smooth_s': 1.0,
        'out_period_s': 0.1,          # output and gate rate
        'gyro_std_window_s': 5.0,
        'drift_window_s': 10.0,
        'max_dF_pct': 8.0,
        'max_dIncl_deg': 6.0,
        'max_gyro_std_deg': 4.0,
        'max_gyro_drift_deg': 6.0,
        'sigma_ok_deg': 8.0,
        'imu_max_age_s': 0.2,
        'stale_s': 1.0,
        'rtk_cov_max': 0.01,          # rad^2, orientation_covariance[8]
        'rtk_hold_s': 1.0,            # RTK yaw carried by the gyro this long
        'rtk_gyro_tol_deg': 15.0,     # RTK glitch: off from the RTK-gyro offset by more
        'self_calibrate': True,
        'sample_period_s': 0.2,
        'n_bins': 36,
        'per_bin': 40,
        'window_k': 20,
        'min_bins': 30,
        'min_per_bin': 3,
        'fit_reject_dF_pct': 12.0,
        'fit_reject_err_deg': 20.0,
        'fit_max_rejected': 0.3,
        'rtk_settle_s': 5.0,          # RTK valid this long before samples are taken
        'fit_max_p90_deg': 8.0,
        'fit_max_worse_deg': 1.0,
        'sample_max_age_s': 7 * 86400.0,   # older self-calibration samples are not used
        # Also subtract the horizontal hard iron before levelling (not only hz).
        # Off: on the 2026-11-08 bag it made F noisier and refused every fit.
        'hard_xy_pre_level': False,
        'stats_window_s': 600.0,
    }

    def __init__(self, cal=None, **params):
        unknown = set(params) - set(self.DEFAULTS)
        if unknown:
            raise ValueError('unknown parameters: %s' % sorted(unknown))
        self.p = dict(self.DEFAULTS, **params)
        self.cal = None
        self.cal_note = ''
        if cal is not None:
            self.set_calibration(cal)
        p = self.p
        self.mag = Ring(14, 4096)         # t, raw b x y z, R_level (9), game yaw (unwrapped)
        self.imu = Ring(2, 1024)          # t, game yaw (unwrapped)
        self.bins = SampleBins(p['n_bins'], p['per_bin'], p['window_k'])
        self.level = None
        self.yaw_u = None
        self.t_imu = None
        self.t_mag = None
        self.last_b = None
        self.last_out_t = -1e18
        self.last_sample_t = -1e18
        self.anchors = deque(maxlen=30)
        self.anchor = None
        self.t_rtk = None
        self.t_rtk_since = None
        self.rtk_rejects = 0
        self.rtk_used = 0
        self.rtk_glitches = 0
        self.out = None
        self.last_err = None
        self.errors = deque()            # (t, |err| deg, ok)
        self.last_fit = None

    # -- inputs ---------------------------------------------------------------

    def set_calibration(self, cal):
        if cal is not None and cal.source != self.p['source']:
            self.cal_note = 'calibration for source %r refused (running %r)' % (cal.source, self.p['source'])
            return False
        self.cal = cal
        return True

    def on_imu(self, t, qx, qy, qz, qw):
        lv = quat_level(qx, qy, qz, qw)
        if lv is None:
            return
        yaw, L = lv
        self.level = L[0] + L[1] + L[2]
        self.yaw_u = yaw if self.yaw_u is None else self.yaw_u + wrap(yaw - self.yaw_u)
        self.t_imu = t
        self.imu.append((t, self.yaw_u))

    def game_yaw_at(self, t):
        r = self.imu.rows()
        if len(r) == 0 or t < r[0, 0] - 0.05 or t > r[-1, 0] + 0.5:
            return None
        return float(np.interp(t, r[:, 0], r[:, 1]))

    def on_rtk(self, t, qx, qy, qz, qw, cov_yaw):
        """True when the RTK heading was taken as reference."""
        if not (cov_yaw < self.p['rtk_cov_max']) or (qx == qy == qz == qw == 0.0) \
                or not all(math.isfinite(v) for v in (t, qx, qy, qz, qw)):
            return False
        yg = self.game_yaw_at(t)
        if yg is None:
            return False
        a = wrap(quat_yaw(qx, qy, qz, qw) - yg)
        if len(self.anchors) >= 5:
            m = circmean(list(self.anchors))
            if abs(wrap(a - m)) * DEG > self.p['rtk_gyro_tol_deg']:
                self.rtk_rejects += 1
                self.rtk_glitches += 1
                if self.rtk_rejects < 10:
                    return False
                self.anchors.clear()          # gyro origin moved (IMU restart): start over
        self.rtk_rejects = 0
        if self.t_rtk is None or t - self.t_rtk > self.p['rtk_hold_s']:
            self.t_rtk_since = t
        self.anchors.append(a)
        self.anchor = float(circmean(list(self.anchors)[-5:]))
        self.t_rtk = t
        self.rtk_used += 1
        return True

    def rtk_yaw(self, t):
        """RTK yaw carried to time t by the gyro, None when not valid or not
        settled (a fresh RTK heading converges for a few seconds)."""
        if self.t_rtk is None or self.yaw_u is None or abs(t - self.t_rtk) > self.p['rtk_hold_s'] \
                or t - self.t_rtk_since < self.p['rtk_settle_s']:
            return None
        return wrap(self.yaw_u + self.anchor)

    def on_mag(self, t, bx, by, bz):
        """Field in microtesla.  Returns the output dict at out_period_s, else None."""
        if self.level is None or self.t_imu is None or abs(t - self.t_imu) > self.p['imu_max_age_s']:
            return None
        b = (bx, by, bz)
        if b == self.last_b:
            return None                        # repeated report (100 Hz topic over a 20 Hz report)
        if not (math.isfinite(bx) and math.isfinite(by) and math.isfinite(bz)):
            return None
        self.last_b = b
        self.mag.append((t, bx, by, bz) + self.level + (self.yaw_u,))
        self.t_mag = t
        if self.p['self_calibrate'] and t - self.last_sample_t >= self.p['sample_period_s']:
            ref = self.rtk_yaw(t)
            if ref is not None:
                self.last_sample_t = t
                self._add_sample(t, b, ref)
        if t - self.last_out_t >= self.p['out_period_s']:
            self.last_out_t = t
            return self._output(t)
        return None

    def _add_sample(self, t, b, ref):
        w = self.mag.since(t - self.p['smooth_s'])
        k = self.p['window_k']
        if len(w) > k:
            w = w[np.linspace(0, len(w) - 1, k).round().astype(int)]
        self.bins.add(t, b, self.level, w[:, 1:4], w[:, 4:13], self.yaw_u - w[:, 13], ref)

    # -- output ---------------------------------------------------------------

    def _output(self, t):
        p = self.p
        if self.cal is None:
            self.out = {'t': t, 'state': 'uncalibrated', 'ok': False, 'reasons': ['no calibration'],
                        'yaw': None}
            return self.out
        r = self.mag.since(t - p['drift_window_s'] - p['smooth_s'] - 0.05)
        h, F, incl, _ = self.cal.apply_raw(r[:, 1:4], r[:, 4:13])
        d = np.unwrap(h - r[:, 13])
        tt = r[:, 0]
        now_w = tt > t - p['smooth_s']
        d_now = float(circmean(d[now_w]))
        yaw = float(wrap(self.yaw_u + d_now + self.cal.offset))
        reasons = []
        dF = dI = None
        if self.cal.Fref:
            dF = abs(float(np.mean(F[now_w])) - self.cal.Fref) / self.cal.Fref * 100.0
            if dF >= p['max_dF_pct']:
                reasons.append('field')
        if self.cal.Iref is not None:
            dI = abs(float(np.mean(incl[now_w])) - self.cal.Iref) * DEG
            if dI >= p['max_dIncl_deg']:
                reasons.append('inclination')
        gs = tt > t - p['gyro_std_window_s']
        gyro_std = None
        if tt[0] <= t - 0.8 * p['gyro_std_window_s'] and gs.sum() >= 3:
            gyro_std = float(np.std(d[gs])) * DEG
            if gyro_std >= p['max_gyro_std_deg']:
                reasons.append('gyro_std')
        old_w = (tt > t - p['drift_window_s'] - p['smooth_s']) & (tt <= t - p['drift_window_s'])
        drift = None
        if old_w.any():
            drift = abs(float(wrap(d_now - circmean(d[old_w])))) * DEG
            if drift >= p['max_gyro_drift_deg']:
                reasons.append('gyro_drift')
        if gyro_std is None or drift is None:
            reasons.append('warmup')
        vals = [yaw, d_now] + [x for x in (dF, dI, gyro_std, drift) if x is not None]
        if not all(math.isfinite(x) for x in vals):
            out = {'t': t, 'state': 'disturbed', 'ok': False, 'reasons': ['invalid'], 'yaw': None}
            self.out = out
            return out
        ok = not reasons
        out = {'t': t, 'state': 'ok' if ok else 'disturbed', 'ok': ok, 'reasons': reasons,
               'yaw': yaw, 'sigma_deg': p['sigma_ok_deg'] if ok else None,
               'dF_pct': dF, 'dIncl_deg': dI, 'gyro_std_deg': gyro_std, 'gyro_drift_deg': drift}
        ref = self.rtk_yaw(t)
        if ref is not None:
            err = float(wrap(yaw - ref)) * DEG
            out['rtk_err_deg'] = err
            self.last_err = (t, err, ok)
            self.errors.append((t, abs(err), ok))
        while self.errors and self.errors[0][0] < t - p['stats_window_s']:
            self.errors.popleft()
        self.out = out
        return out

    # -- self-calibration -----------------------------------------------------

    def fit_snapshot(self):
        min_t = None if self.t_mag is None else self.t_mag - self.p['sample_max_age_s']
        return self.bins.snapshot(min_t)

    def fit_from(self, snap, now):
        p = self.p
        return fit_calibration(snap, p['source'], p['min_bins'], p['min_per_bin'],
                               p['fit_reject_dF_pct'], p['fit_reject_err_deg'], p['fit_max_rejected'],
                               p['fit_max_p90_deg'], p['fit_max_worse_deg'], current=self.cal, now=now,
                               hard_xy=p['hard_xy_pre_level'])

    def adopt(self, res, now):
        self.last_fit = dict((k, v) for k, v in res.items() if k != 'cal')
        self.last_fit['t'] = now
        if res['accepted']:
            self.set_calibration(res['cal'])
            self.cal_note = 'self-calibrated from RTK'
        return res['accepted']

    def fit(self, now):
        """Snapshot, fit, adopt: the whole step (tests, replay)."""
        res = self.fit_from(self.fit_snapshot(), now)
        self.adopt(res, now)
        return res

    # -- status ---------------------------------------------------------------

    def status(self, now):
        p = self.p
        r2 = lambda x, n=2: None if x is None else round(float(x), n)  # noqa: E731
        out = self.out
        if self.t_mag is None or now - self.t_mag > p['stale_s'] or self.t_imu is None \
                or now - self.t_imu > p['stale_s']:
            state, reasons = 'stale', ['no magnetometer or IMU data']
        elif self.cal is None:
            state, reasons = 'uncalibrated', ['no calibration']
        elif out is None or out.get('yaw') is None:
            state, reasons = 'disturbed', ['warmup']
        else:
            state, reasons = out['state'], out['reasons']
        s = {'state': state, 'reasons': reasons, 'source': p['source'],
             'yaw_deg': None, 'sigma_deg': None}
        if out is not None and out.get('yaw') is not None and state != 'stale':
            s.update(yaw_deg=r2(out['yaw'] * DEG), sigma_deg=out['sigma_deg'],
                     dF_pct=r2(out['dF_pct']), dIncl_deg=r2(out['dIncl_deg']),
                     gyro_std_deg=r2(out['gyro_std_deg']), gyro_drift_deg=r2(out['gyro_drift_deg']))
        cal = self.cal
        if cal is not None:
            s['calibration'] = {
                'age_s': r2(time.time() - cal.created, 0) if cal.created else None,
                'created': cal.to_dict()['created_iso'],
                'center_uT': [r2(v) for v in cal.center], 'radii_uT': [r2(v) for v in cal.radii],
                'hard_uT': [r2(v) for v in cal.hard],
                'offset_deg': r2(cal.offset * DEG), 'Fref_uT': r2(cal.Fref),
                'Iref_deg': r2(None if cal.Iref is None else cal.Iref * DEG),
                'resid_p50_deg': r2(cal.resid_p50_deg), 'resid_p90_deg': r2(cal.resid_p90_deg),
                'best_p90_deg': r2(cal.best_p90_deg), 'bins': cal.bins, 'samples': cal.samples}
        else:
            s['calibration'] = None
        if self.cal_note:
            s['calibration_note'] = self.cal_note
        ref_ok = self.rtk_yaw(now) is not None
        s['self_cal'] = {'enabled': bool(p['self_calibrate']), 'samples': self.bins.total(),
                         'bins_covered': self.bins.coverage(p['min_per_bin']), 'bins_needed': p['min_bins'],
                         'last_fit': self.last_fit}
        cmp = {'rtk_valid': ref_ok, 'rtk_used': self.rtk_used, 'rtk_glitches': self.rtk_glitches,
               'window_s': p['stats_window_s']}
        if self.last_err is not None and now - self.last_err[0] <= p['rtk_hold_s'] + 0.5:
            cmp['err_deg'] = r2(self.last_err[1])
            cmp['err_ok'] = self.last_err[2]
        if self.errors:
            e = np.array([x[1] for x in self.errors])
            okm = np.array([x[2] for x in self.errors])
            cmp['all'] = summary(e)
            cmp['ok'] = summary(e[okm])
            cmp['ok_share'] = round(float(okm.mean()), 3)
        s['rtk_compare'] = cmp
        return s
