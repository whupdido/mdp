# Turn radius test (archived, not built)

Measures the radius and achieved angle of the four 90 degree turn geometries
(FL, FR, BL, BR), 5 runs each, driven by SW1 and the OLED. Produced the numbers
in `STM32_motion_spec.md` under "Turns".

Lives here rather than in `Core/` so CubeIDE does not compile it. To use it:

1. Copy `turn_test.c` and `ui.c` into `Core/Src/`, `turn_test.h` and `ui.h`
   into `Core/Inc/`.
2. Re-add the accessor it needs in `control.c`, next to `move_turn_deg`:

       float motion_last_turn_deg(void) { return accum_deg; }

   and declare it in `control.h`.
3. Call `turn_test_run()` from the SW1 handler in `main.c`.

## What it measures and why that way

Radius, not encoder counts, because the radius is what displaces the car on the
grid: after a 90 degree turn the car has moved one radius forward and one
sideways. `R = arc / theta`, arc from the mean of both encoders, theta from the
gyro.

It calibrates gyro bias first, with the car held still. Every angle is a gyro
integral, so an uncalibrated bias becomes an angle error that grows with turn
duration.

It keeps integrating the gyro for 700 ms after the move returns. The controller
stops steering `BRAKING_LEAD_DEG` short of target and lets the chassis coast
into the rest, so the angle the controller reports is not the heading the car
finishes at. Deceleration changes exactly that coast, so measuring only what the
controller believed would hide the effect being looked for.

It runs each case 5 times and reports the spread. That is what caught the
left-turn repeatability regression, which a single run per case would have
missed entirely.

`ui.c` is only SW1 debounce and tap-versus-hold timing. A release is not
believed until the pin has read high continuously for 60 ms; exiting on the
first high sample makes every hold register as a tap, because the switch
chatters mid-press.
