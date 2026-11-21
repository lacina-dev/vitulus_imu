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

Sensor reports are asked for at the library default, 50 ms: the values on the
100 Hz topics change 20 times a second and the stamp is the time of publishing,
not of the measurement (so in both drivers below).

#### Two drivers, one switch (`imu_driver`, T39 2026-11-16)
The driver exists twice, with the same node name, topics, types, parameters,
health JSON, exit codes and bytes on the line:

| `imu_driver` | executable | |
|---|---|---|
| `python` (default; any value but `cpp`) | `nodes/bno085_node` | as before, uses `adafruit_bno08x` |
| `cpp` | `bno085_node_cpp` (`src/bno085_node.cpp`, `src/bno085_supervisor.h`) | the same logic in C++ (roscpp, termios; no new dependency): the port is read in blocks, not byte by byte |

**State: `cpp` is NOT deployed and not tried on the real sensor yet** — only
against the fake sensor of the tests.  The robot runs `python`.

Switch on the robot: `export VITULUS_IMU_DRIVER=cpp` in `~/.bashrc` (sourced by `vitulus.service`) and restart the
stack; remove the line to go back to Python. By hand: `roslaunch vitulus_imu vitulus_imu.launch imu_driver:=cpp`, or
`<arg name="imu_driver" value="cpp"/>` in the include of `vitulus_imu.launch`
(`vitulus/launch/vitulus_start.launch`, `vitulus_bringup/launch/vitulus_indoor.launch`);
the package has to be built first (`bno085_node_cpp` in `devel/lib/vitulus_imu`).
Back: `imu_driver:=python` or leave the argument out.  Only one of the two may
run: both lock the port (`flock`), the second one fails to connect and exits 1.

Known differences of `cpp`: `header.seq` of a topic starts at 0 (rospy: 1;
roscpp numbers it itself); after a failure the process leaves 0.2 s later (so
that the last health message gets out); log lines of the first moments reach
the log file but not always `/rosout`; the library's busy waits during the
start sleep 0.5 ms; an unknown report during the start ends the process with
exit 1 and a FATAL line (Python: traceback, exit 1).

Measured on the fake sensor only (`tools/bno085_bench.py`: pty, 60 s per run,
nodes at nice 0, robot computer in the dock with load average 6.7) — an
indication, not the real sensor:

| sensor reports | driver | CPU of the node (one core) | period of `/bno085/imu`, ms: min / median / p99 / max | periods off 10 ms by more than 1 ms |
|---|---|---|---|---|
| 20 Hz (what the node asks for) | `python` | 6.82 % | 0.19 / 10.00 / 12.75 / 21.99 | 40.6 % |
| | `cpp` | 0.90 % | 1.33 / 10.00 / 11.48 / 18.66 | 2.9 % |
| 100 Hz (heavier stream) | `python` | 21.47 % | 0.21 / 10.03 / 14.99 / 113.36 | 66.6 % |
| | `cpp` | 0.97 % | 0.02 / 10.00 / 10.38 / 22.38 | 1.1 % |

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

     With `~sigma_from_rtk` (default **off**; T35 I2) the base in state
     `ok` with a confirmed anchor follows the error measured against the
     RTK heading while that is a valid reference: base =
     clip(`~sigma_rtk_k` (1.5) × RMS error, `~sigma_min_deg` (1.5°),
     `~sigma_ok_deg`), RMS over about `~sigma_rtk_tau_s` (300 s) of
     comparisons, used after `~sigma_rtk_min_s` (30 s) of them and kept
     when RTK is gone.  σ is never above what it is without it; the other
     terms raise it as before.  18 drives 2026-11-15/16 (recomputed from
     the recorded outputs): σ median 4.0° → 2.2°, error below 2σ in 99.6 %
     of the `ok` outputs (RMS error 1.18°).  Status then has `sigma`:
     `from_rtk`, `ok_deg`, `min_deg`, `measured_rms_deg` (null before 30 s
     of comparisons), `measured_s`.

     **Start inside a disturbance without RTK** (T35, 2026-11-21).  The
     field gate works on the size of the field (`~max_dF_pct` 8 % of Fref
     = 4.1 µT at Fref 51 µT) and on the inclination (`~max_dIncl_deg` 6°).
     A disturbance across the horizontal field turns the heading and
     changes neither much, so the gate lets it through, and without a
     reference the filter cannot know it started on a wrong value: the
     unconfirmed anchor only promises σ = `~sigma_start_deg` (8°).
     Synthetic start inside a steady field across the Earth's horizontal
     field, no RTK (`ok` share and error while inside):

     | field | error | `ok` | error / σ |
     |---|---|---|---|
     | 5 µT | 12.4° | 96 % | 1.55 |
     | 6 µT | 14.3° | 93 % | 1.79 |
     | 7 µT | 16.1° | 91 % | 2.02 |
     | 8 µT | 17.8° | 66 % | 2.23 |
     | 9 µT | 19.5° | 32 % | 2.42 |
     | 10 µT | 21.4° | 14 % | 2.64 |

     **Limit: from about 7 µT across the horizontal field the error of an
     `ok` output is above 2σ until RTK has been seen** (6 µT is the last
     value below; the same field along it gives 6–9° and is gated from
     8 µT by the inclination).  From 8 µT up `gyro_drift` and
     `inclination` cut the `ok` share, but not to zero.  To move the limit:
     `~sigma_start_deg` 11 covers 10 µT (21.4° / 11° = 1.9σ) at the price
     of σ 11° for every drive before the first RTK; tighter gates
     (`~max_dF_pct`, `~max_dIncl_deg`) do not help against a field across
     the horizontal one and cost `ok` share on the lawn (below).  A consumer that
     needs 2σ should take outputs with `fusion.anchor` = `rtk` only.

     Measured on the robot (drives dock → lawn → dock, 2026-11-15 two
     bags and 2026-11-18 one, live calibration, 1 s mean of the field):
     in the dock the field is **+8.2 … +9.0 µT** over Fref (median; 16–18 %,
     gate 8 %), the inclination +1.4 … +2.0°, the raw magnetometer heading
     −5 … −10° off; 94–98 % of the samples fail the gate, the filter does
     not fuse them and **no anchor is seated in the dock** (state
     `disturbed`, `field` + `warmup`).  0.15–1 m out of the dock the field
     is back at −0.3 … +0.8 µT (median), with the raw heading still
     0 … +5° off in that first metre; on the lawn (> 5 m) median
     −0.1 … +1.9 µT, 5–95 % −5 … +5.5 µT (drive current, turns), 5–16 % of
     the samples outside the gate.  So the dock itself is far above the
     limit and is caught by the gate; the case above needs a start in a
     place that is off by a few µT across the horizontal field and passes
     the gate (a car, a fence, a wall with steel).

     **Re-seating the unconfirmed anchor** (`~anchor_reseat_s`, default
     **0 = off**, as before).  After such a start the robot drives out
     into the clean field: the heading is right within two minutes, but the
     correction is now 12–20° from the `start` anchor and the state is
     falsely `disturbed` / `anchor` until 0.2°/min × anchor age explains
     it (5 µT: about 3–7 min, 6 µT and more: over 10 min).  With
     `~anchor_reseat_s` > 0 an anchor that RTK has not confirmed is seated
     again when the correction has stayed within `~anchor_reseat_tol_deg`
     (3°) of one value for that many seconds of fused samples (standing
     frozen and gated samples do not count) and that value is further
     than the tolerance from the anchor.  The step is remembered
     (`fusion.anchor_moved_deg`, count `fusion.anchor_reseats`; both only
     in the status when the switch is on) and σ counts it with the
     deviation until RTK confirms the anchor: which of the two places was
     the disturbed one cannot be told without a reference, so σ is never
     below what it is with the switch off; only the reason `anchor` goes.
     An anchor confirmed by RTK is never re-seated, and when RTK arrives
     the anchor is the RTK one within seconds, with the switch on or off.
     Synthetic (4 min inside, then 11 min clean, `~anchor_reseat_s` 120):
     falsely `disturbed` after leaving 167 s → 21 s (5 µT), > 558 s →
     148 s (6 µT), > 581 s → 250 s (7 µT; σ after the re-seat 9.4°); the
     heading is bit-identical.  Price: a robot that started clean and then
     enters a steady disturbance without ever seeing RTK loses the reason
     `anchor` after the same time (σ still covers the step).  On the three
     drives above nothing is re-seated and every output is the same with
     120 s and 300 s as with 0 (no start inside a disturbance there:
     **not yet seen on a real drive**).  Tests:
     `test/test_mag_heading_t35_kotva.py`.
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
7. **Refinement without full coverage** (`~refine_calibration`, on; off =
   nothing happens until 30 bins are covered, as before).  A robot that
   drives dock → point → dock never heads some ways (the 2026-11-15 nights:
   27 of 36 bins over six drives, −120…−30° never), so the full fit above
   never comes.  When a fit is refused for coverage, the current calibration
   (same source) is checked on the same samples instead: samples with an
   odd field strength (> 12 % off `Fref`) or > 20° off RTK are dropped (at
   most 30 %), and it must still fit them (residual p90 < `~fit_max_p90_deg`,
   8°), with ≥ `~refine_min_samples` (100, 20 s of valid RTK) samples in
   ≥ `~refine_min_bins` (18, half the circle) bins of ≥ 3 samples.  Then:
   - the **residual covariance** (axis weighting) is added to it when it has
     none, or refreshed when the one it has came from a refinement on fewer
     than 2/3 of the samples now at hand (a matrix from a full fit stays);
   - only with `~refine_offset` (default **off**) the **offset** gets a step
     = − median over the bins of the per-bin median residual (each direction
     counts once), only when that is clear: the bins agree (std ≤
     `~refine_max_spread_deg`, 3°), the centre is ≥ `~refine_min_step_deg`
     (0.3°) and ≥ 3 standard errors, and the residual p90 is not worse with
     the step.  One step is at most `~refine_max_step_deg` (2°), the sum of
     the steps since the last full fit at most ±`~refine_max_total_deg`
     (5°); then it waits for a full fit.  σ of every `ok` output is raised
     by |sum of the steps|.
   The ellipse and the hard iron are never changed by a refinement.
   The reference is the same as for the full fit (full fix, variance band,
   agrees with the gyro, settled), so a float or glitching RTK gives no
   samples; repeated fits on the same samples do not add up (each one is
   measured against the offset as it is now).  The file (version 2) records
   `refine: {count, offset_sum_deg, last, cov_samples}`; a full fit starts
   it again.  At the moment of an offset step the output does not jump: the
   correction and its anchor take the opposite step, so the heading moves
   to the new offset over the fusion window (`fuse_tau_s`, ~1 min; while
   standing frozen, until it drives); the deviation from the anchor does
   not change at that moment, σ grows by the step.
   **A reference that is off by a steady amount** (an RTK antenna moved, a
   wrong baseline for drive after drive) passes all the gates.  The matrix
   only weights the two body axes and carries no direction, so it does not
   follow such an error.  The offset step would: with RTK 6° off, steps of
   +2, +2, +1° up to the 5° limit (error in `ok` 6–7°), and after the
   reference is right again it comes back only a step per fit and only as
   far as the evidence is clear (in a check, +2° were left after two wrong
   and three right drives).  On the drives so far the step was never
   needed (the axis weights removed the steady +1.5°), hence off by default.
   Switching `~refine_calibration` off does not undo what was written (the
   matrix and an offset step stay in the yaml): back = copy
   `mag_calibration.v1.yaml` over it, or `~axis_weighting:=false` for the
   matrix.
   On the 2026-11-15 drives the axis weights alone removed the steady +1.5°
   (the offset was within the noise, no step); see "Replay 2026-11-15".
8. **Visibility.**  `status.self_cal` has `missing_deg` (directions of the
   RTK yaw without enough samples, as `[from, to]` ranges in degrees, from >
   to across ±180°), `refine_enabled`, `refinements` and
   `offset_refined_deg` (since the last full fit), `axis_weighting`, and
   `last_fit.refine` (the last refinement's numbers and reason).  A fit
   refused for coverage is logged once per `~coverage_log_s` (30 min) with
   the missing directions and what the refinement did; every change of the
   calibration is logged (matrix added / refreshed, offset step).

**RTK heading as reference** (self-calibration and `rtk_compare`): only with a
full fix, as `navi_transform` does — the last NAV-RELPOSNED (received at most
`~relpos_max_age_s`, 0.3 s, before the heading) has carrSoln FIXED and
relPosHeadingValid, and the yaw variance is within `~rtk_cov_min`…`~rtk_cov_max`
(0.00043…0.00079 rad²).  On the 2026-11-08 bag the headings with a small
variance but no full fix were a median 34° off, and fixed ones outside the
band 5° (max 19°); inside the band max 5.9°.  When NAV-RELPOSNED does not
arrive the RTK heading is not taken and `rtk_compare.rtk_reason` says so
(`~rtk_require_fixed:=false` falls back to the variance band alone).

**Samples only from RTK that stayed valid** (`~sample_confirm_s`, default 0 =
off; T35 I3).  In front of the garage the RTK heading is off for the last
seconds before it is lost (full fix, variance in the band, 6° off in the
median).  With `~sample_confirm_s` N > 0 a self-calibration sample waits and
goes to the bins only when the RTK heading stayed a valid reference for N s
after it without a break; when RTK is lost first (or the gyro origin moves)
the waiting samples are dropped.  On the samples of 18 drives 2026-11-15/16
(981, of which the refinement dropped 131 for the field strength) 10 s keeps
115 of those 131 out, at the cost of 130 of the 850 good ones.
`status.self_cal` then has `sample_confirm_s`, `samples_waiting` and
`samples_unconfirmed` (dropped so far).

**RTK against the gyro:** the RTK yaw minus the game yaw is a slowly moving
offset.  A valid RTK message whose offset is more than `~rtk_gyro_tol_deg`
(15°) from the mean of the last 30 is ignored (counted in
`rtk_compare.rtk_glitches`).  After 10 such messages in a row the history is
dropped and the RTK is believed again.  So an RTK that is wrong for more than
~3 s at 3 Hz is taken as reference; the fit drops samples > 20° off, but the
comparison statistics include them.

**Calibration file format.**  `version: 2` adds `resid_cov` (2×2, unit plane).
Version 1 files (no matrix) load and behave as before the axis weighting
(both axes the same).  The matrix is added to it and the file rewritten
(a) by a full fit that is refused (ratchet, worse than current) on samples
the old calibration still fits, or (b) by a refinement (item 7) when the
directions are not covered — the usual case; before 2026-11-15 case (b)
did not exist and a coverage refusal returned before the matrix was
computed, so a robot that never covers 30 bins stayed without it.  After
an update the matrix comes once samples in 18 directions with valid RTK
are at hand (on the 2026-11-15 drives: 12 min into the first drive).  A node from before version 2 refuses a version 2 file
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
36 directions with RTK, e.g. a slow turn on the spot or two (the directions
add up over drives and restarts, the samples are on disk).

**Samples on disk.**  The self-calibration samples (36 × 40 with their
1 s window; compressed ~0.7 kB per sample, at most ~1 MB) are written to
`~samples_file` (default `mag_samples.npz` next to the calibration; `''` =
memory only) atomically, at most every `~samples_save_s` (60 s) when there
are new ones, and at shutdown; the copy is taken under the lock, the
compression and the write in the timer thread, and a write error is a
warning (once per 10 min), never a stop.  At start they are loaded when the
file has the same format version and source; samples older than
`~sample_max_age_s` (7 days) are left out, as are samples with a
non-finite value or an RTK yaw out of range; samples up to 6 h after the
current time (a clock set back) count as fresh, later ones are left out
with a warning.  A broken or foreign file is logged and the collection
starts empty.  So the bins add up over restarts and drives (before: every
restart of the node started from 0).  `status.self_cal.samples_file` and
`samples_note` say what was loaded and, after every write, what is on disk
now (`samples file …: N samples, B bins, written <time>`, or `cannot be
written (samples in memory only)` with the time of the last good write);
`samples_saved_age_s` is the age of the last write of this run (null before
the first).

**Parameters** (private): `mag_topic`, `mag_source` (`uncal`), `imu_topic`,
`rtk_topic`, `relpos_topic`, `motion_topic` (empty: no odometry), `frame_id`
(`bno_imu_link`), `calibration_file`, `fit_interval_s`, `status_period_s`,
and every key of `MagHeadingEstimator.DEFAULTS` (`fuse_tau_s`,
`freeze_standing`, `axis_weighting`, `max_dF_pct`, `max_dIncl_deg`,
`max_gyro_std_deg`, `max_gyro_drift_deg`, `sigma_ok_deg`, `sigma_k`,
`max_anchor_dev_deg`, `anchor_from_rtk`, `freeze_max_s`, `rtk_cov_min`,
`rtk_cov_max`, `rtk_require_fixed`, `rtk_settle_s`, `self_calibrate`,
`min_bins`, `fit_max_p90_deg`, `refine_calibration`, `refine_min_samples`,
`refine_min_bins`, `refine_max_step_deg`, `refine_max_total_deg`,
`refine_min_step_deg`, `refine_max_spread_deg`, `refine_offset`, ...);
node only: `coverage_log_s` (1800).

## Tools
`tools/mag_replay.py BAG [--two-pass] [--cal-bag BAG2] [--mag-topic /bno085/mag --source cal]`
runs the same logic over a bag (magnetometer, IMU, RTK heading, NAV-RELPOSNED,
wheel odometry) and prints the error against RTK — all outputs, `ok`, moving,
standing — and the share of `ok`.  `--cal-bag BAG2` is the cross test: the
calibration self-calibrated on BAG2 is frozen and BAG replayed with it.
`--set KEY=VALUE` changes an estimator parameter, `--dump FILE.npz` writes the
outputs, `--samples FILE` loads the self-calibration samples at the start and
writes them at the end (with `--cal`/`--save`: drives in a row as on the
robot).  Bags from before report 0x0F only have `/bno085/mag` (picked
automatically, source `cal`); a bag without `/gnss_heading/navrelposned`
is replayed with the variance band alone (printed).

## Tests (run file by file)
```
nice -n 10 python3 -m pytest test/test_mag_heading.py -q
nice -n 10 python3 -m pytest test/test_bno085_mag_uncal.py -q
nice -n 10 python3 test/test_bno085_recovery.py [--old-ref REF] [--variants new,old,cpp]
nice -n 10 python3 -m pytest test/test_bno085_equivalence.py -q
nice -n 10 python3 -m pytest test/test_bno085_fast_report.py -q
```
The bno085 tests run the Python node and, with
`BNO085_CPP_NODE=<workspace>/devel/lib/vitulus_imu/bno085_node_cpp` in the
environment, the C++ node against the same checks (without it the C++ runs are
skipped, `test_bno085_equivalence.py` as a whole).  `test_bno085_equivalence.py`
feeds both nodes the same bytes and compares messages, bytes sent to the sensor
and health.  The old faults of `test_bno085_recovery.py` need
`--old-ref 835d38f~1` (the node before the recovery fix).  Build for the tests
outside the live workspace, e.g. a workspace in `/tmp` with a link to this
package, extending `~/catkin_ws/devel`.  `tools/bno085_bench.py` measures CPU
and period of both drivers on the fake sensor.

## Replay 2026-11-15: refinement without full coverage

The stored calibration of the robot (version 1, from 2026-11-14, offset
−5.16°), the two night drives that have the raw magnetometer
(`nav_baseline_2026-11-15_a`, then `nav_d1_2026-11-15_a`, the samples carried
from one to the other as on the robot).  |error| against the single RTK
messages (full fix, band, valid ≥ 5 s) in `ok`, median / p90 / max:

| drive | before (no refinement) | with refinement |
|---|---|---|
| baseline_a (weights from 721 s on) | 1.41 / 2.18 / 4.51 | the same (no `ok` RTK after 721 s) |
| d1 (weights from the start) | 1.77 / 2.61 / 5.12, mean +1.54 | 0.59 / 1.62 / 5.98, mean +0.04 |
| baseline_a, the refined file | 1.41 / 2.18 / 4.51 | 0.44 / 0.99 / 3.41 (in-sample) |
| magtest3 (2026-11-14, where the old file was made) | 1.09 / 2.36 / 4.75 | 0.88 / 2.05 / 4.57 |

The share of `ok` does not change (72 / 77 / 83 %).  The refinement added
the axis weights (380 samples in 18 directions) 721 s into the first drive
and did not move the offset (centre +0.3…+0.85°, within 3 standard errors;
the directions disagree by a std of 2.2–3°): the steady +1.5° of the nights
came from the equal axes, not from the offset.  The larger maximum on d1
is four RTK messages in the first 2 s of valid RTK of the drive where the error was
already −4.3° and the removed +1.5° bias makes it −6°; the share above 4°
went from 8 to 3 messages, p99 4.3 → 3.6°.

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
