Quadruped_EE579/
├─ firmware/
│  ├─ arduino/
│  │  └─ main/
        ├─ main.ino           # main loop (CPG, IMU, FSR gating, servo writes)
│  │  ├─ cpg.h                   # tiny Hopf-style oscillator (lite)
│  │  ├─ ids.h                   # servo IDs, pins, constants
│  │  └─ utils.h                 # helpers (clamp, deg↔rad)
│  │  └─ README.md                  # board/libraries + upload notes
│  └─ scripts/                      # Python log parser, gait generator, etc.
├─ hardware/
│  ├─ wiring/
│  │  ├─ wiring_map.md              # IMU/FSR/Power/shield notes
│  └─ mounting/                       # 3D-printed IMU/FSR brackets, LEGO builds notes
├─ docs/
│  ├─ assembly_guide.md
│  ├─ calibration.md                # IMU calibration + FSR thresholds
│  └─ gait_notes.md                 # how CPG maps to joints
├─ data/
│  ├─ logs/                         # serial logs captured during testing
│  └─ calib/                        # saved offsets, trims, thresholds
└─ folder_organization.md        # this file
