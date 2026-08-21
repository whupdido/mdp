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

**Cannot turn on the spot.** Ackermann steering. Radii measured at rear-axle centre.

| Command | Radius | Displacement (90°) | Grid cells | Duration |
|---|---|---|---|---|
| `FL` | 261 mm | 261 fwd + 261 left | ~2.6 | ~2.4 s |
| `FR` | 318 mm | 318 fwd + 318 right | ~3.2 | ~2.9 s |
| `BL` | 246 mm | 246 back + 246 left | ~2.5 | ~2.3 s |
| `BR` | 303 mm | 303 back + 303 right | ~3.0 | ~2.8 s |

Right turns need 22% more space than left. Prefer left turns where the path allows a choice.

Non-90° angles scale linearly from the above.

**Reverse turns invert the heading change** — `BL` turns the nose right, `BR` turns it left. *(Derived from geometry, not yet confirmed on the robot.)*

## Clearance

Body 230 × 188 mm.

| Turn | Inner radius | Outer radius (excl. front overhang) |
|---|---|---|
| FL / BL | ~167 mm | ~355 mm |
| FR / BR | ~209 mm | ~397 mm |

Working figures for obstacle inflation: **450 × 450 mm** clear for a left turn, **500 × 500 mm** for a right turn.

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

SPEED_STRAIGHT      20        // counts per 10 ms tick
SPEED_TURN          14

TURN_COUNTS_FL      3000      // 90 deg
TURN_COUNTS_FR      3650
TURN_COUNTS_BL      2830
TURN_COUNTS_BR      3480

SERVO_CENTRE        1500
SERVO_LEFT          1000
SERVO_RIGHT         2100
```

Control loop 100 Hz (TIM6). Motor PWM 10 kHz (TIM9/10/11).

## Unverified

- Heading sign on `BL` / `BR`
- Non-90° turns (180°, 360°)
- Repeatability — each command ×5, record spread
- Real swept envelope including front overhang
- Command timings by stopwatch
