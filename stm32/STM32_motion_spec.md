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

| Command | Radius | Spread (5 runs) | Displacement (90°) | Grid cells |
|---|---|---|---|---|
| `FL` | **317 mm** | ±5 mm (1.2 %) | 317 fwd + 317 left | ~3.2 |
| `FR` | **413 mm** | ±3 mm (0.6 %) | 413 fwd + 413 right | ~4.1 |
| `BL` | **312 mm** | ±4 mm (0.9 %) | 312 back + 312 left | ~3.1 |
| `BR` | **421 mm** | ±3 mm (0.6 %) | 421 back + 421 right | ~4.2 |

Every radius is 21–39 % larger than the previous table claimed. **Any planner
still using 261/318/246/303 is under-reserving space on every turn.**

Right turns need **30 % more space than left going forward, 35 % in reverse** —
not the 22 % previously stated. Prefer left turns wherever the path allows.

Repeatability is good: worst-case spread is 1.2 % of radius, so turns are
consistent even though they are large.

Non-90° angles scale linearly from the above.

**Reverse turns invert the heading change** — `BL` turns the nose right, `BR`
turns it left. *(Derived from geometry, still not confirmed on the robot.)*

## Clearance

Body 230 × 188 mm, so ±94 mm either side of the rear-axle centre path.

| Turn | Inner radius | Outer radius (excl. front overhang) |
|---|---|---|
| `FL` | ~223 mm | ~411 mm |
| `FR` | ~319 mm | ~507 mm |
| `BL` | ~218 mm | ~406 mm |
| `BR` | ~327 mm | ~515 mm |

Left and right can no longer share a row — they differ by ~100 mm of outer radius.

Working figures for obstacle inflation, scaled from the measured envelope:
**520 × 520 mm** clear for a left turn, **620 × 620 mm** for a right turn.
Both were 450/500 before, i.e. too small by a comfortable margin. Front
overhang is still not included in either figure.

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
   these. Kept as documentation. Values below are back-computed from the
   radii measured 31-Aug-2026; the ones in calib.h are older and lower. */
TURN_COUNTS_FL      3640      // 90 deg
TURN_COUNTS_FR      4750
TURN_COUNTS_BL      3580
TURN_COUNTS_BR      4830

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
