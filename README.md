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
Absolute heading from the magnetometer for times without RTK, with a
"trust / do not trust" gate and self-calibration from the RTK heading.
**Navigation does not use it yet**; it is published and compared with RTK.

| topic | type | |
|---|---|---|
| in `~mag_topic` | sensor_msgs/MagneticField | default `/bno085/mag_uncal` |
| in `~imu_topic` | sensor_msgs/Imu | default `/bno085/imu` (game rotation vector: levelling + gyro) |
| in `~rtk_topic` | sensor_msgs/Imu | default `/gnss_heading/navheading`, valid when `orientation_covariance[8] < ~rtk_cov_max` |
| out `/bno085/mag_heading` | sensor_msgs/Imu | ~10 Hz, only with a calibration; orientation = yaw only; `orientation_covariance` = `[1e3,0,0, 0,1e3,0, 0,0,σ²]`, σ = `~sigma_ok_deg` (8°) when the state is `ok`, else σ² = 1e3; angular velocity / acceleration covariance −1 (not provided) |
| out `/bno085/mag_heading/status` | std_msgs/String JSON | latched, 1 Hz (below) |

**Yaw convention:** the same as `/gnss_heading/navheading` (ENU, yaw of Euler
`'ZYX'` of the quaternion, radians).  The calibration offset is fitted against
that topic, so it absorbs the mounting of the IMU and the magnetic declination;
the two can be compared directly.

**States** (`status.state`):
- `ok` — calibrated and every gate passes; σ = 6°.
- `disturbed` — calibrated, but a gate fails (`reasons`):
  `field` (|F − Fref|/Fref ≥ 8 %), `inclination` (|I − Iref| ≥ 6°),
  `gyro_std` (std of mag − gyro over 5 s ≥ 4°), `gyro_drift` (mag − gyro
  moved ≥ 6° in 10 s), `warmup` (less than 10 s of data).  Published with
  σ² = 1e3.  Near the dock the field is ~24 % off: `disturbed` there is right.
- `uncalibrated` — no calibration for this magnetometer source; nothing published.
- `stale` — no magnetometer or IMU message for `~stale_s` (1 s).

**Status JSON**: `state`, `reasons`, `yaw_deg`, `sigma_deg`, `dF_pct`,
`dIncl_deg`, `gyro_std_deg`, `gyro_drift_deg`; `calibration` (age, centre,
semi-axes, offset, Fref, Iref, residual p50/p90, bins, samples);
`self_cal` (samples, bins covered/needed, last fit with its reason);
`rtk_compare` — while the RTK heading is valid: `err_deg` (current error),
and over the last `~stats_window_s` (10 min) `all` and `ok`
(`median`/`p90`/`p99`/`max` of |error|) and `ok_share`.  This is what a
field test reads.

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
   heading_raw = −atan2(Hc_y, Hc_x); yaw = heading_raw + offset.
4. Smoothing without lag: yaw = game_yaw_now + circular mean over 1 s of
   (heading_raw − game_yaw) + offset.
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
   smoothed residual against RTK has p90 < `~fit_max_p90_deg` (8°), at most
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

**RTK against the gyro:** the RTK yaw minus the game yaw is a slowly moving
offset.  A valid RTK message whose offset is more than `~rtk_gyro_tol_deg`
(15°) from the mean of the last 30 is ignored (counted in
`rtk_compare.rtk_glitches`).  After 10 such messages in a row the history is
dropped and the RTK is believed again (the game yaw origin moves when the IMU
node restarts).  So an RTK that is wrong for more than ~3 s at 3 Hz is taken
as reference; the fit drops samples > 20° off, but the comparison statistics
include them.

Calibrations with a non-finite or degenerate value are refused at load; a
non-finite input or result is not published as a heading (`reasons:
["invalid"]`).  Files without `hard_uT` (older format) load with h = 0.

A new robot (no file) is `uncalibrated` until it has turned through ≥ 30 of
36 directions with RTK, e.g. a slow turn on the spot or two.

**Parameters** (private): `mag_topic`, `mag_source` (`uncal`), `imu_topic`,
`rtk_topic`, `frame_id` (`bno_imu_link`), `calibration_file`,
`fit_interval_s`, `status_period_s`, and every key of
`MagHeadingEstimator.DEFAULTS` (`smooth_s`, `max_dF_pct`, `max_dIncl_deg`,
`max_gyro_std_deg`, `max_gyro_drift_deg`, `sigma_ok_deg`, `rtk_cov_max`,
`rtk_settle_s`, `self_calibrate`, `min_bins`, `fit_max_p90_deg`, ...).

## Tools
`tools/mag_replay.py BAG [--two-pass] [--mag-topic /bno085/mag --source cal]`
runs the same logic over a bag and prints the error against RTK and the share
of `ok`.  Bags from before report 0x0F only have `/bno085/mag` (picked
automatically, source `cal`).

## Tests (run file by file)
```
nice -n 10 python3 -m pytest test/test_mag_heading.py -q
nice -n 10 python3 -m pytest test/test_bno085_mag_uncal.py -q
nice -n 10 python3 test/test_bno085_recovery.py [--old-ref REF]
```

## Field test 2026-11-09 (uncal source, self-calibrated from RTK)

Drive with three full turns, straights and arcs up to 10 m from the dock:
self-calibration accepted after ~4 min (36 of 36 bins), Fref 46.8 µT,
inclination 64.4° (Earth here ~49 µT / ~66°), vertical hard-iron 14 µT.
Live error against RTK heading: median ~4°, p90 ~7°, max ~11°; in `ok`
max 8.6° (hence `sigma_ok_deg` 8°). About 2° of the median comes from RTK
heading latency (~0.1 s) during fast turns (up to 1.6 rad/s); standing
still the error at one spot was a steady +6° (local field / heading-dependent
residual), so treat the heading as a coarse prior, not as a fine reference.
