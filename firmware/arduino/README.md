 ┌──────────────┐
 │   Time (t)   │   ← from millis()
 └──────┬───────┘
        │
        ▼
 ┌─────────────────────────┐
 │   Gait Phase Shifting   │
 │  (LF, RF, LR, RR φ_i )  │
 └──────────┬──────────────┘
            │
            ▼
 ┌─────────────────────────┐
 │     Foot Trajectory     │
 │   Stance (line) &       │
 │   Swing (circle arc)    │
 └──────────┬──────────────┘
            │  (x, y)
            ▼
 ┌─────────────────────────┐
 │   Inverse Kinematics    │
 │   hip  = IK_1(x, y)     │
 │   knee = IK_2(x, y)     │
 └──────────┬──────────────┘
            │  raw angles
            ▼
 ┌─────────────────────────┐
 │    Zeroing Offset Add   │
 │    angle += offset[i]   │
 └──────────┬──────────────┘
            │
            ▼
 ┌─────────────────────────┐
 │   Direction Correction  │
 │   if mirrored → 360 - θ │
 └──────────┬──────────────┘
            │
            ▼
 ┌─────────────────────────┐
 │   Send to Dynamixels    │
 │  dxl.setGoalPosition()  │
 └─────────────────────────┘
