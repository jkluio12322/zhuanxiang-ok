# BlueCar Debug Log

## 2026-09-23: Speed PID Reverse Saturation at Zero Target

### Symptom

- VOFA waveform showed the left and right actual speeds falling from a
  positive target to about `-140 RPM`.
- The reverse motion happened on both wheels and was sustained, so it was not
  a single encoder spike.
- The fault became obvious when the speed target returned to zero.

### Root Cause

The speed loop uses an incremental PID:

```text
deltaOut = Kp * (e0 - e1) + Ki * e0 + Kd * (e0 - 2*e1 + e2)
Out      = Out + deltaOut
```

When the target changed from `+70 RPM` to `0 RPM`, the error became about
`-70 RPM`. The loop generated a negative output increment and accumulated
negative PWM while trying to reduce the measured speed.

When the measured speed entered the `1.8 RPM` PID deadband, the output
increment stopped, but the existing negative `Out` remained. The negative PWM
continued to drive the motor in reverse until the measured speed reached the
reverse maximum.

The speed PID has no feedforward term in the current implementation:

- `Kf` is not used by `PID_IncrementalUpdate()`.
- Both speed PID objects initialize `Kf` to `0`.
- The historical `Kf=0.69/0.65` implementation is not the cause of this
  reverse saturation.

### Fix

File: `Core/Application/yingYongKongZhi.c`

Added `SPEED_PID_STOP_HOLD_RPM = 0.5f` and
`appControlConstrainSpeedPidOutput()`.

The new constraint is applied after each incremental PID update:

- A target whose absolute value is at most `0.5 RPM` forces PWM to zero.
- A positive target cannot output negative PWM.
- A negative target cannot output positive PWM.
- PID state reset on target direction changes remains enabled.

This prevents the speed loop from driving opposite to its target. The tradeoff
is that a zero target now coasts instead of using reverse PWM for active
braking.

### Verification

- Keil build: `0 Error(s), 0 Warning(s)`.
- Firmware: `MDK-ARM/bluecar/bluecar.hex`.
- Real-car test: passed; the previous reverse saturation no longer occurs.

### Regression Check

If a wheel still reaches a sustained reverse maximum while the corresponding
target is zero:

1. Confirm the flashed firmware is the build containing
   `appControlConstrainSpeedPidOutput()`.
2. Add left and right PWM channels to VOFA telemetry.
3. If PWM is zero but measured speed is still a sustained negative maximum,
   check motor driver polarity and encoder sign.

### Follow-Up: Pivot Turn With One Positive And One Negative Target

The first zero-target guard prevented all PWM opposite to a nonzero target.
That was too restrictive for a pivot turn using `+40 RPM` on the outer wheel
and `-40 RPM` on the inner wheel. If the inner wheel overshot its negative
target, the PID could not apply positive braking PWM to bring it back.

The guard was temporarily revised to allow opposite-direction PWM up to 25%.
Hardware testing showed that this hard 25% brake clamp caused visible wheel
judder, so the clamp was removed. The current rules are:

- A target at or near zero still forces PWM to zero.
- For a nonzero target, the full normal `-100%...+100%` PID output range is
  available, so braking through zero stays continuous.

### Current Issue: Actual Wheels Rotate With A Start-Stop Judder

The incremental speed loop had `Kf = 0`. It therefore had to accumulate PWM
from zero through the motor and gearbox stiction region whenever a target
started. During that region the wheel could catch, move briefly, stall again,
and catch again, which appeared as a one-step-at-a-time judder.

The old position PID used `Kf = 0.69` for the left wheel and `Kf = 0.65` for
the right wheel. Incremental feedforward was first added with those values, but
the defaults are now temporarily set to zero for an A/B hardware test:

```text
SPEED_PID_LEFT_KF  = 0.0f
SPEED_PID_RIGHT_KF = 0.0f
```

The feedforward calculation and runtime commands remain in place. Setting the
defaults to zero isolates whether the actual-wheel judder comes from
feedforward or from the incremental PI state.

Runtime tuning is available through USART1:

```text
skf 0.65
skfl 0.69
skfr 0.65
```

Build result after temporarily disabling feedforward:
`0 Error(s), 0 Warning(s)`.

Hardware re-test with straight motion, zero-target braking, and pivot turning
is pending. Restore feedforward online with:

```text
skfl 0.69
skfr 0.65
```
