# vitulus_imu
 Vitulus ROS package for BNO085 Imu.

## Nodes

### bno085_node (`launch/vitulus_imu.launch`)
BNO085 over UART (`~port`, default `/dev/imu`).  Watches itself (timeouts,
respawn, `/bno085/health`); see the docstring of `nodes/bno085_node`.

| topic | type | |
|---|---|---|
| `/bno085/imu` | sensor_msgs/Imu | game rotation vector (gyro + accel, no mag), gyro, accel, 100 Hz |
| `/bno085/navheading` | sensor_msgs/Imu | BNO rotation vector (with mag) — useless near the robot (median error 42°) |
| `/bno085/mag` | sensor_msgs/MagneticField | report 0x03, "calibrated" by the BNO dynamic calibration (it drifts) |
| `/bno085/mag_uncal` | sensor_msgs/MagneticField | report 0x0F, uncalibrated, one message per new report (~20 Hz), tesla, frame `bno_imu_link` |
| `/bno085/imu_status` | vitulus_msgs/Imu_status | 1 Hz |
| `/bno085/health` | std_msgs/String JSON | latched, 1 Hz; also `mag_accuracy` (0–3, BNO accuracy of report 0x03), `mag_uncal` (`on` / `no data` / `unavailable` / `off`; `no data` also when 0x0F stops for `~mag_uncal_stale_s`, 2 s, while the rest streams), `mag_uncal_accuracy`, `mag_bias_uT` (the BNO's own hard-iron estimate from 0x0F) |

Report 0x0F (SH-2 "Magnetic Field Uncalibrated", 16 bytes: id, seq, status
with accuracy in bits 1:0, delay, X Y Z uncalibrated, X Y Z bias, int16 LE,
Q4 µT) is optional: if the sensor does not enable it the node runs on without
it and logs one warning.  `~mag_uncal:=false` switches it off.  The BNO dynamic
calibration is neither enabled nor disabled by the node.

### mag_heading (`nodes/mag_heading`, logic in `src/vitulus_imu/mag_heading.py`)
Absolute heading from the magnetometer for times without RTK: the gyro (game
rotation vector) carries the heading, the magnetometer corrects it slowly.
With a "trust / do not trust" gate and self-calibration from the RTK heading.
**Navigation does not use it yet**; it is published and compared with RTK.

| topic | type | |
|---|---|---|
| in `~mag_topic` | sensor_msgs/MagneticField | default `/bno085/mag_uncal` |
| in `~imu_topic` | sensor_msgs/Imu | default `/bno085/imu` (game rotation vector: levelling + gyro) |
| in `~rtk_topic` | sensor_msgs/Imu | default `/gnss_heading/navheading`; a reference only with a full fix (below) |
| in `~relpos_topic` | ublox_msgs/NavRELPOSNED9 | default `/gnss_heading/navrelposned`: carrier solution of the RTK heading |
| in `~motion_topic` | nav_msgs/Odometry | default `/mobile_base_controller/odom`: standing or moving (`twist.linear.x`) |
| out `/bno085/mag_heading` | sensor_msgs/Imu | ~7 Hz (every second or third report of the 20 Hz magnetometer, `~out_period_s` 0.1 s), only with a calibration; stamp = time of the magnetometer sample; orientation = yaw only; `orientation_covariance` = `[1e3,0,0, 0,1e3,0, 0,0,σ²]`, σ ≥ `~sigma_ok_deg` (4°) when the state is `ok` (see **Sigma**), else σ² = 1e3; angular velocity / acceleration covariance −1 (not provided) |
| out `/bno085/mag_heading/status` | std_msgs/String JSON | latched, 1 Hz (below) |

**Yaw convention:** the same as `/gnss_heading/navheading` (ENU, yaw of Euler
`'ZYX'` of the quaternion, radians).  The calibration offset is fitted against
that topic, so it absorbs the mounting of the IMU and the magnetic declination;
the two can be compared directly.

**States** (`status.state`):
- `ok` — calibrated and every gate passes; σ = 4° or more (see **Sigma**).
- `disturbed` — calibrated, but a gate fails (`reasons`):
  `field` (|F − Fref|/Fref ≥ 8 %, mean of 1 s), `inclination` (|I − Iref| ≥ 6°),
  `gyro_std` (std of mag − gyro over 5 s ≥ 10°: a field that swings),
  `gyro_drift` (mean innovation of the fusion over 10 s ≥ 6°: the
  magnetometer and the gyro-carried heading disagree for seconds),
  `anchor` (the correction is ≥ `~max_anchor_dev_deg`, 10°, further from its
  anchor than the gyro can have drifted: a field that is off for minutes,
  see **Sigma**),
  `warmup` (fusion window below 4.5 s, or less than 4 s of data).  Published
  with σ² = 1e3.  Near the dock the field is ~24 % off: `disturbed` there is
  right; the heading itself is then carried by the gyro from the last good
  correction.
- `uncalibrated` — no calibration for this magnetometer source; nothing published.
- `stale` — no magnetometer or IMU message for `~stale_s` (1 s).

**Status JSON**: `state`, `reasons`, `yaw_deg`, `sigma_deg`, `dF_pct`,
`dIncl_deg`, `gyro_std_deg`, `gyro_drift_deg`;
`fusion` — `tau_s`, `tau_eff_s` (effective window now), `frozen` (standing:
the correction is held), `freeze_standing`, `motion`
(`moving` / `standing` / `unknown`), `note` (only when the wheel odometry is
missing: runs without the freeze), `axis_weighting`, `resets`, `last_reset`,
`anchor` (`rtk` / `start` / null), `anchor_age_s`, `anchor_dev_deg` and
`hold_age_s` (what sigma is made of, below);
`calibration` (`version`, age, centre, semi-axes, offset, Fref, Iref, residual
p50/p90, bins, samples, `axis_weights`);
`self_cal` (samples, bins covered/needed, last fit with its reason);
`rtk_compare` — `rtk_valid`, `rtk_reason` (`ok`, or why the RTK heading is not
a reference: `carrSoln 1 (not FIXED)`, `no NAV-RELPOSNED received`, `yaw
variance … outside …`, `settling`, …), `rtk_require_fixed`; while the RTK
heading is valid: `err_deg` (current error), and over the last
`~stats_window_s` (10 min) `all` and `ok` (`median`/`p90`/`p99`/`max` of
|error|) and `ok_share`.  This is what a field test reads.

**How it works**
1. Vertical hard iron hz in the sensor frame, subtracted before levelling
   (h = (0, 0, hz); the horizontal part stays in the 2D centre c — subtracting
   it before levelling too, `~hard_xy_pre_level`, made the field strength
   noisier on the 2026-11-08 bag and every fit was refused).  hz would otherwise leak into the horizontal on a slope as
   hz·sin(tilt) (30 µT at 4° ≈ 2 µT ≈ 6° with H ≈ 20 µT).  h comes from a
   linear regression (3D h, of which hz is kept) on the RTK samples: L(B − h) = c' + A·[cos ψ, sin ψ]
   (ψ RTK yaw).  hz is visible only through tilt; with flat samples a weak
   prior keeps the previous hz (0 at first).
2. Levelling: roll/pitch of the game rotation vector; B_level = Ry(pitch) Rx(roll) (B − (0, 0, hz)).
3. 2D calibration in the levelled frame: Hc = W (B_level,xy − c)
   (c what is left of the hard iron, W soft iron: ellipse → unit circle);
   heading_raw = −atan2(Hc_y, Hc_x).
4. Slow fusion with the gyro: yaw = game_yaw + d + offset.  The correction d
   (heading_raw − game_yaw) is a recursive least-squares estimate with
   exponential forgetting, time constant `~fuse_tau_s` (60 s; the game
   rotation vector drifts only 0.12–0.19°/min).  The information starts at
   zero, so after a start or a restart the window grows with the data
   (effective constant ≈ min(elapsed, tau), `fusion.tau_eff_s`) and the output
   is usable after `~fuse_init_s` (5 s).
   - **Axis weighting.**  Each sample pulls d by its residual in the unit
     plane, Hc − (cos a, −sin a) with a = game_yaw + d, weighted by the
     inverse of the 2×2 residual covariance of the calibration (body axes).
     The field of the drive current lies along one body axis (about −145°
     from sensor x: 1.5 µT of residual there, 0.6 µT across), so that axis
     counts less.  The matrix is estimated by the self-calibration from the
     same RTK samples (1 s means of the residual) and stored in the file
     (`resid_cov`); a missing, non-finite or degenerate matrix means both
     axes count the same.  `~axis_weighting:=false` switches it off.
   - **Standing.**  With the holding torque the magnetometer is off by −5…+7°
     while the robot stands.  Standing = mean |v| of the wheel odometry
     < 0.05 m/s and mean |gyro z| < 0.1 rad/s over 1 s: the correction is
     frozen and the gyro carries the heading (`~freeze_standing`, default
     true).  The frozen correction is held unchanged for `~freeze_max_s`
     (30 min: the gyro drifts 3.6–5.7° in that time, about what the standing
     magnetometer is off); then its window is cut to half of `~fuse_init_s`
     and topped up from the standing magnetometer, so a robot that never
     moves still has a bounded heading (σ = 8° from then on).  A robot that
     starts standing takes its first `~fuse_init_s` from the magnetometer
     the same way.
     Without the odometry (older than 0.5 s) there is no freeze and the status
     says so.
   - **Bad field.**  Samples that fail the `field` / `inclination` gate (dock,
     metal) are not fused.
   - **Game yaw origin.**  The origin of the game yaw moves when the IMU node
     restarts.  A gap in the IMU data > `~imu_gap_reset_s` (1 s), a yaw step
     faster than `~max_yaw_rate` (8 rad/s), or used samples more than
     `~fuse_reset_deg` (30°) from the gyro for `~fuse_reset_s` (3 s) start the
     filter again (`fusion.resets`, `last_reset`).
   - **Time of the sample.**  The game yaw is taken at the stamp of the
     magnetometer sample.  A sample that arrives before the IMU message of
     its time waits for it (the two topics are not delivered in step: on the
     bags up to 0.13 s, which is 8° in a turn).
   - **Sigma and the anchor.**  The gate cannot see a field that is off but
     inside the limits (3–5 µT in the horizontal is 8–12° and passes): the
     correction follows it within a minute or two, for as long as it lasts
     (a long fence, a steel hall).  But the correction (magnetometer − gyro)
     can truly change only as fast as the gyro drifts, about 0.2°/min.  So it
     is compared with an **anchor**:
     - while the RTK heading is a valid reference (full fix, variance band,
       agrees with the gyro, settled) the anchor is the correction RTK
       implies (RTK − gyro over the last 30 messages, minus the calibration
       offset) and is renewed all the time; when RTK goes, it stays;
     - before any RTK the anchor is the correction itself at the moment the
       window first reached `~sigma_settle_s` (30 s).  That one is not
       confirmed — the filter may have started in a disturbed place — and σ
       stays at `~sigma_start_deg` (8°) until RTK has been seen;
     - a restart of the filter (game yaw origin) or a new calibration voids it.

     deviation = |correction − anchor| (`fusion.anchor_dev_deg`), and

     σ = max(base, `~sigma_k` (0.6) × (deviation + |mean innovation over 10 s|)
     + `~gyro_drift_deg_min` (0.2°/min) × time since the last fused sample)

     with base = `~sigma_ok_deg` (4°) when the anchor is confirmed by RTK and
     the window is ≥ 30 s, else `~sigma_start_deg` (8°).  The part of the
     deviation that the gyro drift cannot explain (deviation − 0.2°/min ×
     anchor age) at or above `~max_anchor_dev_deg` (10°) makes the state
     `disturbed`, reason `anchor`.  A calibration that is 1–3° off between
     drives is a deviation of 1–3° and does not lift σ (0.6 × 3° < 4°).
     With time since the last RTK the gyro drift explains more (after an
     hour 12°): the state then stays `ok`, but σ still takes the whole
     deviation, because a drift of the gyro and a field that crept off
     cannot be told apart any more.  While the robot stands (correction
     frozen) σ grows by the full 0.2°/min of the hold.
5. Gate as above, F = hypot(|Hc|·H0, Bz), I = atan2(−Bz, |Hc|·H0), H0 = mean semi-axis.
6. Self-calibration: while RTK is valid (and has been for `~rtk_settle_s`,
   5 s; RTK samples that disagree with the gyro by > 15° are ignored, see
   below),
   levelled samples are taken at 5 Hz into 36 bins by RTK yaw (40 per bin,
   oldest replaced).  Every `~fit_interval_s` (45 s), when ≥ `~min_bins` (30)
   bins are covered: ellipse fit on the per-bin medians → drop samples with an
   odd field strength (> 12 %) → refit, offset → drop samples > 20° off RTK →
   final fit (the hard iron is re-estimated in each round).  Samples older
   than `~sample_max_age_s` (7 days) are not used.  Accepted when the
   1 s-smoothed residual against RTK has p90 < `~fit_max_p90_deg` (8°), at most
   30 % of the samples were dropped, it is not worse than the current
   calibration by more than `~fit_max_worse_deg` (1°) on the same samples,
   and not worse than the best residual so far (`best_p90_deg`, kept in the
   file) by more than that either — so the calibration cannot creep worse
   1° at a time.  (The best-so-far may come from a smaller, earlier sample
   set; a later fit on more samples is then refused until it comes within
   1° — the current calibration stays in use meanwhile.)  When the current calibration no longer fits the samples
   (p90 ≥ 8°, e.g. the robot was changed) the best-so-far is forgotten and
   any calibration that passes the other tests is taken.  Accepted calibrations are written atomically to
   `~calibration_file` (default `~/.ros/vitulus_imu/mag_calibration.yaml`)
   and loaded at start.  The file records the magnetometer `source`
   (`uncal` / `cal`): a calibration of another source is not used.

**RTK heading as reference** (self-calibration and `rtk_compare`): only with a
full fix, as `navi_transform` does — the last NAV-RELPOSNED (received at most
`~relpos_max_age_s`, 0.3 s, before the heading) has carrSoln FIXED and
relPosHeadingValid, and the yaw variance is within `~rtk_cov_min`…`~rtk_cov_max`
(0.00043…0.00079 rad²).  On the 2026-11-08 bag the headings with a small
variance but no full fix were a median 34° off, and fixed ones outside the
band 5° (max 19°); inside the band max 5.9°.  When NAV-RELPOSNED does not
arrive the RTK heading is not taken and `rtk_compare.rtk_reason` says so
(`~rtk_require_fixed:=false` falls back to the variance band alone).

**RTK against the gyro:** the RTK yaw minus the game yaw is a slowly moving
offset.  A valid RTK message whose offset is more than `~rtk_gyro_tol_deg`
(15°) from the mean of the last 30 is ignored (counted in
`rtk_compare.rtk_glitches`).  After 10 such messages in a row the history is
dropped and the RTK is believed again.  So an RTK that is wrong for more than
~3 s at 3 Hz is taken as reference; the fit drops samples > 20° off, but the
comparison statistics include them.

**Calibration file format.**  `version: 2` adds `resid_cov` (2×2, unit plane).
Version 1 files (no matrix) load and behave as before the axis weighting
(both axes the same); the first fit whose samples the old calibration still
fits adds the matrix to it and rewrites the file, even when the new fit
itself is refused.  A node from before version 2 refuses a version 2 file
(`calibration version 2`, it starts `uncalibrated` and calibrates again from
RTK).  For that case the version 1 file is copied to
`mag_calibration.v1.yaml` next to it before the first version 2 write (once;
an existing copy is never replaced; on a file system without hard links it
is written directly; if it cannot be made the node logs one warning and
saves anyway): going back to the old code = copy it over
`mag_calibration.yaml`.  Calibrations with a non-finite or
degenerate value are refused at load; a non-finite input or result is not
published as a heading (`reasons: ["invalid"]`).  Files without `hard_uT`
load with h = 0.

A new robot (no file) is `uncalibrated` until it has turned through ≥ 30 of
36 directions with RTK, e.g. a slow turn on the spot or two.

**Parameters** (private): `mag_topic`, `mag_source` (`uncal`), `imu_topic`,
`rtk_topic`, `relpos_topic`, `motion_topic` (empty: no odometry), `frame_id`
(`bno_imu_link`), `calibration_file`, `fit_interval_s`, `status_period_s`,
and every key of `MagHeadingEstimator.DEFAULTS` (`fuse_tau_s`,
`freeze_standing`, `axis_weighting`, `max_dF_pct`, `max_dIncl_deg`,
`max_gyro_std_deg`, `max_gyro_drift_deg`, `sigma_ok_deg`, `sigma_k`,
`max_anchor_dev_deg`, `anchor_from_rtk`, `freeze_max_s`, `rtk_cov_min`,
`rtk_cov_max`, `rtk_require_fixed`, `rtk_settle_s`, `self_calibrate`,
`min_bins`, `fit_max_p90_deg`, ...).

## Tools
`tools/mag_replay.py BAG [--two-pass] [--cal-bag BAG2] [--mag-topic /bno085/mag --source cal]`
runs the same logic over a bag (magnetometer, IMU, RTK heading, NAV-RELPOSNED,
wheel odometry) and prints the error against RTK — all outputs, `ok`, moving,
standing — and the share of `ok`.  `--cal-bag BAG2` is the cross test: the
calibration self-calibrated on BAG2 is frozen and BAG replayed with it.
`--set KEY=VALUE` changes an estimator parameter, `--dump FILE.npz` writes the
outputs.  Bags from before report 0x0F only have `/bno085/mag` (picked
automatically, source `cal`); a bag without `/gnss_heading/navrelposned`
is replayed with the variance band alone (printed).

## Tests (run file by file)
```
nice -n 10 python3 -m pytest test/test_mag_heading.py -q
nice -n 10 python3 -m pytest test/test_bno085_mag_uncal.py -q
nice -n 10 python3 test/test_bno085_recovery.py [--old-ref REF]
```

## Replay 2026-11-14: slow fusion against the first version

Three drives: two of 2026-11-08 (b1, b2) and one of 2026-11-14 with the mower
running (b3: turns, straight strips, 4.3 min of mowing at 3300 rpm).
|error| as median / p90 / max in degrees, in brackets the share of the time
in `ok`.  "Cross": the calibration comes from another drive and is frozen.
The stored file of the robot was rewritten by the live node during b3, so
that row is not an independent test.

**Raw** — against the single RTK heading messages (full fix, variance in the
band, valid for ≥ 5 s).  This includes the noise of the RTK heading itself
(inside the band up to 5.9°), so the maxima are an upper bound.

| test | first version (1 s window) | now |
|---|---|---|
| cross b1 → b2, all | 3.7 / 8.9 / 12.4 | 1.2 / 3.0 / 5.9 |
| cross b1 → b2, `ok` | 2.9 / 5.1 / 9.4 (22 %) | 1.2 / 3.0 / 5.4 (64 %) |
| cross b2 → b1, all | 5.3 / 8.9 / 14.3 | 2.1 / 3.6 / 8.4 |
| cross b2 → b1, `ok` | 5.5 / 8.6 / 9.8 (55 %) | 2.1 / 3.5 / 7.5 (80 %) |
| cross b2 → b3 (6 days later), all | 3.4 / 6.6 / 11.4 | 2.0 / 3.7 / 7.5 |
| cross b2 → b3, `ok` | 3.9 / 7.0 / 9.7 (50 %) | 2.1 / 3.8 / 6.1 (70 %) |
| robot file → b3, all | 2.0 / 4.6 / 9.9 | 1.1 / 2.4 / 5.8 |
| robot file → b3, `ok` | 2.0 / 4.6 / 7.7 (64 %) | 1.1 / 2.4 / 4.8 (83 %) |

In `ok` the error is within the published σ in 94–99.6 % of the outputs and
within 2σ in all of them (worst 1.8σ); p90 in `ok` is ≤ 3.8°, so
`sigma_ok_deg` 4° stands as a 1σ.

**Smoothed** — against the RTK heading carried by the gyro (20 s median of
RTK − gyro, full fix only): the RTK noise is averaged out, which shows the
error of the heading itself; the maxima are a lower bound.

| test | first version (1 s window) | now |
|---|---|---|
| inside a bag (b1, b2; calibration of the same bag), all | 2.0 / 5.4 / 11.9 | 0.35 / 1.6 / 2.5 |
| cross b1 ↔ b2, all | 4.6 / 8.8 / 14.3 | 1.4 / 2.6 / 3.6 |
| cross b1 ↔ b2, `ok` | 4.6 / 8.4 / 9.5 (26–56 %) | 1.5 / 2.7 / 3.6 (66–82 %) |
| cross b1 ↔ b2, standing | 5.2 / 9.3 / 14.3 | 1.7 / 2.3 / 3.4 |
| cross b2 → b3, all | 3.3 / 6.3 / 10.1 | 1.7 / 3.6 / 4.0 |
| cross b2 → b3, `ok` | 3.9 / 6.8 / 9.3 (51 %) | 1.8 / 3.7 / 4.0 (72 %) |
| cross b2 → b3, mower off | 3.9 / 6.8 / 9.3 | 2.0 / 3.7 / 4.0 |
| cross b2 → b3, mower on | 2.7 / 5.6 / 10.1 | 1.5 / 3.5 / 3.9 |
| cross b2 → b3, standing | 4.7 / 7.7 / 9.3 | 2.0 / 3.8 / 4.0 |

What each part gives on the cross test b1 ↔ b2 (all outputs): without the
axis weights 2.7 / 3.3 / 4.3, without the standing freeze 1.2 / 3.0 / 5.5,
with `fuse_tau_s` 120 s 1.3 / 2.2 / 3.4.  With the filter restarted every
60 s (the worst case for the growing window) the state `ok` reaches 7.9°
while the window is below 30 s — hence `sigma_start_deg` 8°; with the window
grown the largest smoothed error in `ok` is 3.6–4.0° and the raw p90 3.8° —
`sigma_ok_deg` 4°.

Synthetic check of what the field gate lets through (steady field in the
horizontal, no RTK while it lasts; the heading follows it and is back about
80 s after the end).  Every output is either `disturbed` or `ok` with the
error inside 2σ:

| field | largest error | state | σ in `ok` | worst error / σ |
|---|---|---|---|---|
| 3 µT for 180 s | 7.8° | `ok` | 4–7.6° | 1.7 |
| 5 µT for 180 s | 12.0° | `disturbed` (`anchor`), `ok` at the edges | up to 9.9° | 1.2 |
| 3 µT for 420 s | 8.1° | `ok` | 4–7.8° | 1.7 |
| 4 µT for 600 s | 10.3° | `ok` | 4–9.0° | 1.7 |
| 5 µT for 600 s | 12.3° | `disturbed` (`anchor`) 60 %, else `ok` | up to 10.2° | 1.7 |
| 0 → 6 µT over 600 s, held 300 s | 14.2° | `ok` to 12°, then `disturbed` (`anchor`) | up to 10.2° | 1.3 |
| 10 µT for 180 s | 18.7° | `disturbed` (`field`, `anchor`) | – | 0.9 |

(anchor confirmed by RTK before; with the unconfirmed start anchor σ is 8° or
more and the worst case is 1.5σ.)  On the three drives the anchor changes
neither the accuracy nor the share of `ok` (the reason `anchor` never
appears), σ in `ok` has a median of 4.0°, and the same holds when RTK is
taken away 60 s after it first became valid (4–11 min without RTK).

The mower motor adds no visible disturbance: the per-sample noise is the same
with the knife on and off (1.2 / 1.8 µT in sensor x / y), the mean horizontal
field moves by less than 0.5 µT (about 1°), the vertical by −0.6 µT.

Not done, on purpose: compensation of the wheel power (B −= K·power) — on
the cross test it gave 1.25 / 3.3 / 5.3 against 1.1 / 3.3 / 5.8 without, and
inside the bags it was worse (p90 4.0 against 1.4); a deviation table by
heading, a map by position and a 3D ellipsoid did not hold between drives.

Limits: the cross-drive error is mostly a constant 1–3° (the ellipse centre
and offset move between drives); the drives were short (≤ 14 min, strips ≤ 2 m)
— a field that is locally off for longer than `fuse_tau_s` is followed; the
anchor then lifts σ and, from 10°, the state.  What it cannot see: a field
that creeps off slower than the gyro drifts (0.2°/min), and a robot that has
never had RTK since the filter started (σ stays 8°).  While RTK is valid the
anchor is RTK itself, so the comparison in `rtk_compare` and the state are
then not independent: an error ≥ 10° against RTK reads `disturbed`.

## Field test 2026-11-09 (first version, 1 s window; uncal source, self-calibrated from RTK)

Drive with three full turns, straights and arcs up to 10 m from the dock:
self-calibration accepted after ~4 min (36 of 36 bins), Fref 46.8 µT,
inclination 64.4° (Earth here ~49 µT / ~66°), vertical hard-iron 14 µT.
Live error against RTK heading: median ~4°, p90 ~7°, max ~11°; in `ok`
max 8.6° (hence `sigma_ok_deg` 8°). About 2° of the median comes from RTK
heading latency (~0.1 s) during fast turns (up to 1.6 rad/s); standing
still the error at one spot was a steady +6° (local field / heading-dependent
residual), so treat the heading as a coarse prior, not as a fine reference.
