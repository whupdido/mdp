# STM32 Motion Module — Interface Spec

Covers checklist A.3 (straight-line motion) and A.4 (rotation).

## Serial

USART3, **115200 8N1**, on **PD8 (STM32 TX) / PD9 (STM32 RX)**.

Not the USB-C port — that's USART1, reserved for FlyMCU. Pi GPIO14→PD9, GPIO15→PD8, common ground. 3.3 V, no level shifting.

## Commands

| Command | Meaning | Argument |
|---|---|---|
| `FWxxx` | Forward | `xxx` = cm |
| `BWxxx` | Backward | `xxx` = cm |
| `FLxxx` | Forward-left | `xxx` = degrees, 1–360 (`000` = 90) |
| `FRxxx` | Forward-right | `xxx` = degrees |
| `BLxxx` | Reverse-left | `xxx` = degrees |
| `BRxxx` | Reverse-right | `xxx` = degrees |
| `STOP` | Abort current move | — |

## Replies

| Reply | Meaning |
|---|---|
| `READY` | Sent once at boot. Don't send commands before it. |
| `DONE` | Completed. Sent after motion finishes, not on receipt. |
| `STALL` | Aborted — wheels stopped for 1 s. **Position unknown.** |
| `TIMEOUT` | Aborted — exceeded 20 s. **Position unknown.** |
| `ACK` | `STOP` acknowledged. |
| `BUSY` | Move already running; command **discarded**. |
| `ERR` | Unrecognised command. |

One command at a time. Send, wait for reply, send next. No queue.

## Straight line

| | |
|---|---|
| Cruise speed | 273 mm/s |
| Command resolution | 10 mm |
| Minimum useful move | 100 mm |
| Accuracy | 200 mm → +10 mm; 500 mm → −15 mm |

Settle 200 ms, then ~160 ms accel ramp. Deceleration begins 82 mm before target. ~12 mm coast after stop.

## Turns

**Cannot turn on the spot.** Ackermann steering. Radii at rear-axle centre.

### Superseded: before the deceleration ramp

**Measured 31-Aug-2026, 5 runs per direction.** Method: the controller accumulates
each rear wheel's arc in encoder counts over a turn (plus 400 ms of coast), and
the IMU supplies the angle. Then `R = mean_arc / yaw_rad`. No chassis constants
are involved. Superseded the previous table, which was computed from the
`TURN_COUNTS_*` constants rather than observed.

**Independently confirmed by tape measure.** One 90° turn per direction, marking
the rear-axle midpoint on the floor at start and finish and measuring the
straight-line chord `c`, then `R = c / 2·sin(θ/2)` with θ read off the OLED:

| | chord | yaw | R from chord | R from encoders |
|---|---|---|---|---|
| `FR` | 590 mm | −90.8° | 414.3 mm | 413.2 mm |

Agreement to 0.3 % on the direction that scrubs the most, which is what
establishes that scrub does not corrupt the radius (see Open).

### Current, after the turn deceleration ramp

**Re-measured 05-Sep-2026, 5 runs per direction**, same method, after the
deceleration ramp was added to `MODE_TURN_DEG` (crawl target in the last 20° of
any turn of 45° or more).

| Command | Radius | Spread (5 runs) | Displacement (90°) | Grid cells |
|---|---|---|---|---|
| `FL` | **277 mm** | ±13 mm (4.7 %) | 277 fwd + 277 left | ~2.8 |
| `FR` | **365 mm** | ±2 mm (0.5 %) | 365 fwd + 365 right | ~3.7 |
| `BL` | **281 mm** | ±14 mm (5.0 %) | 281 back + 281 left | ~2.8 |
| `BR` | **383 mm** | ±2 mm (0.5 %) | 383 back + 383 right | ~3.8 |

Deceleration tightened every radius by 9–13 %:

| | before decel | after decel | change | spread |
|---|---|---|---|---|
| `FL` | 317 ±5 | 277 ±13 | −40 mm, −12.6 % | 1.6 % → 4.7 % |
| `FR` | 413 ±3 | 365 ±2 | −48 mm, −11.6 % | 0.7 % → 0.5 % |
| `BL` | 312 ±4 | 281 ±14 | −31 mm, −9.9 % | 1.3 % → 5.0 % |
| `BR` | 421 ±3 | 383 ±2 | −38 mm, −9.0 % | 0.7 % → 0.5 % |

Tighter turns are the wanted outcome and the clearance figures below have been
rescaled to them.

**Open issue: left-turn repeatability regressed.** Left spread went from ~1.5 %
to ~5 %, a run-to-run range of 26–28 mm per turn, while right turns were
untouched at 0.5 %. A biased radius is harmless once measured; scatter is not,
because it accumulates along a route and cannot be corrected for. Suspected
cause: in the crawl phase `speed_ratio` is 20/50, so the turn feedforward falls
to `1200 × 0.4 = 480` against a `PWM_MAX` of 16799, about **2.9 % duty**, at or
below the motor deadband, leaving the wheel dependent on PID integral windup
rather than feedforward. Left turns also run at `SERVO_LEFT` = 1000, documented
as the saturation limit, so they scrub the most. Untested; the candidate fixes
are raising the crawl target from 20, or flooring the feedforward above the
deadband.

Right turns still need about **31 % more space than left**. Prefer left turns
wherever the path allows, but note they are now the less repeatable ones.

Non-90° angles scale linearly from the above.

**Reverse turns invert the heading change** — `BL` turns the nose right, `BR`
turns it left. *(Derived from geometry, still not confirmed on the robot.)*

## Clearance

Body 230 × 188 mm, so ±94 mm either side of the rear-axle centre path.

| Turn | Inner radius | Outer radius (excl. front overhang) |
|---|---|---|
| `FL` | ~183 mm | ~371 mm |
| `FR` | ~271 mm | ~459 mm |
| `BL` | ~187 mm | ~375 mm |
| `BR` | ~289 mm | ~477 mm |

Left and right still cannot share a row — they differ by ~90 mm of outer radius.

Working figures for obstacle inflation, scaled from the measured envelope:
**480 × 480 mm** clear for a left turn, **570 × 570 mm** for a right turn.
Front overhang is still not included in either figure.

For left turns, add the ±13 mm scatter above on top of these before trusting
them, until the repeatability issue is resolved.

## Timing

```
straight:  t ≈ 0.2 + (distance_mm − 82) / 273 + 0.6   seconds
turn 90°:  2.3–2.9 s (see table)
```

Add ~50 ms per command for serial round-trip.

## Constraints

- **No odometry feedback.** Pi cannot query position. Dead-reckon from the displacements above.
- Errors accumulate across moves — re-reference against a known feature periodically.
- Zero duty = active brake, not coast.
- Stall abort: both wheels under 2 counts/10 ms for 1 s.
- Move timeout: 20 s.

## Constants

```c
COUNTS_PER_REV      1494.0f
WHEEL_DIA_MM        65.0f
MM_PER_COUNT        0.136682f
TRACK_MM            162.5f    // rear tyre contact patch centres, tape measured
                              // 31-Aug-2026. Not a firmware constant; used to
                              // interpret the turn telemetry.

SPEED_STRAIGHT      20        // counts per 10 ms tick  -- STALE, calib.h has 40
SPEED_TURN          14        // STALE, calib.h has 50

/* Unused by the firmware -- turns terminate on integrated IMU yaw, not on
   counts. Kept only so the Pi can interpret older telemetry. The radii above
   are the numbers a planner should use. */
TURN_RADIUS_FL_MM   277       // +-13 mm, see the open issue
TURN_RADIUS_FR_MM   365       // +-2 mm
TURN_RADIUS_BL_MM   281       // +-14 mm, see the open issue
TURN_RADIUS_BR_MM   383       // +-2 mm

SERVO_CENTRE        1500
SERVO_LEFT          1000
SERVO_RIGHT         2100
```

Control loop 100 Hz (TIM6). Motor PWM 10 kHz (TIM9/10/11).

## Open

- **The tyres scrub, and the firmware causes it.** `MODE_TURN_DEG` drives a
  fixed 1.30 / 0.70 outer-inner wheel speed split on every turn. That ratio is
  geometrically correct for a radius of **271 mm** — and nothing else. The
  further a turn's real radius is from 271, the harder the driven wheels fight
  the steering linkage, which shows up as the implied track reading above the
  real 162.5 mm:

  | | radius | implied track | excess |
  |---|---|---|---|
  | `FL` | 317 | 167.5 | +3.1 % |
  | `BL` | 312 | 175.4 | +7.9 % |
  | `BR` | 421 | 181.2 | +11.5 % |
  | `FR` | 413 | 188.1 | +15.8 % |

  Ordering matches distance from 271 mm exactly. The fix is a per-direction
  split: about **1.28 / 0.72 on left turns, 1.20 / 0.80 on right**. Costs
  nothing but tyre wear and repeatability to leave as is, but it is very
  likely part of why left and right differ by 30 %.
- Heading sign on `BL` / `BR`. Cheap to settle: run one `BL` and read the sign
  of YAW on the OLED. `FL` gives +91.9°, `FR` gives −90.8°.
- Non-90° turns (180°, 360°)
- Real swept envelope including front overhang
- Command timings by stopwatch. The straight-line figures in this document
  (273 mm/s, the timing formula) predate `SPEED_STRAIGHT` going from 20 to 40
  and have not been re-measured.

## Settled

- **Repeatability** — 5 runs per direction, spread ≤1.2 % of radius. See Turns.
- **Absolute radius** — chord measurement agrees with the encoder-derived radius
  to 0.3 % on `FR`, the worst-scrubbing direction. The slip is symmetric between
  inner and outer wheels, so it cancels in the mean and the encoder radii are
  sound as they stand. (An `FL` chord that initially disagreed by 11 % was a
  mis-marked measurement, retaken and resolved.)
- **Rear track width** — 162.5 mm, tape measured.
