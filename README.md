# Subsystem & IO Layers Layout
```
robot
├─ shooter
│  ├─ flywheel
│  │  ├─ FlywheelIOReal     (TODO: ADD)
│  │  └─ FlywheelIOSim      (TODO: REFACTOR)
│  └─ hood
│     ├─ HoodIOReal         (TODO: ADD)
│     └─ HoodIOSim          (TODO: REFACTOR)
├─ hopper
│  ├─ intake
│  │  ├─ IntakeIOReal       (TODO: ADD)
│  │  └─ IntakeIOSim        (TODO: REFACTOR)
│  ├─ pivot
│  │  ├─ pivotIOReal  (TODO: ADD)
│  │  └─ pivotIOSim   (TODO: ADD)
│  └─ transition
│     ├─ TransistionIOReal  (TODO: ADD)
│     └─ TransistionIOSim   (TODO: REFACTOR)
├─ climber
│  ├─ ClimberIOReal         (TODO: ADD)
│  └─ ClimberIOSim          (TODO: ADD)
├─ drive
│  ├─ module (x4)
│  │  ├─ ModuleIOSpark
│  │  └─ ModuleIOSim
│  └─ gyro
│     ├─ GyroIONavX
│     ├─ GyroIOBoron
│     └─ GyroIOSim
└─ vision (x2)
   ├─ VisionIOLimelight
   ├─ VisionIOPhoton
   └─ VisionIOPhotonSim
```

# Subsystem States/Commands
```
            robot
            ├─ shooter
            │  ├─ flywheel
            │  │  ├─ idled
  [DEFAULT] │  │  └─ running
            │  ├─ hood
  [DEFAULT] │  │  ├─ idled
            │  │  └─ aimed(angle: double)
            │  │
            │  ├─ idled -> flywheel: idled, hood: idled
  [DEFAULT] │  ├─ hoodIdled -> flywheel: running, hood: idled
            │  └─ hoodAimed(angle: double) -> flywheel: running, hood: aimed(angle)
            ├─ hopper
            │  ├─ intake
  [DEFAULT] │  │  ├─ idled
            │  │  └─ running
            │  ├─ pivot
  [DEFAULT] │  │  ├─ idled
            │  │  ├─ up
            │  │  └─ down
            │  ├─ transition
  [DEFAULT] │  │  ├─ idled
            │  │  └─ running
            │  │
  [DEFAULT] │  ├─ idled -> intake: idled, pivot: idled, transition: idled
            │  ├─ compacted -> intake: idled, pivot: up, transition: idled
            │  ├─ intaking -> intake: pivot.down ? running : idled, pivot: down, transition: idled
            │  └─ transitioning -> intake: idled, pivot: down, transition: running
            ├─ climber
  [DEFAULT] │  ├─ idled
            │  └─ climbing
            ├─ drive
  [DEFAULT] │  ├─ idled
            │  ├─ driving(x: double, y: double, Ω: double)
            │  ├─ basicFFSysID
            │  └─ wheelRadiusCharacterization
            ├─ vision0
  [DEFAULT] │  ├─ aprilTags
            │  └─ otherPipeline(id: int)
            ├─ vision1
  [DEFAULT] │  ├─ aprilTags
            │  └─ otherPipeline(id: int)
            │
            ├─ shooting(x: double, y: double) ->
            │       shooter: hoodAimed(SOTM_HOOD_ANGLE),
            │       hopper: SOTM_ABLE_TO_HIT ? transitioning : idled,
            │       drive: driving(x, y, SOTM_DRIVE_ANGLE),
            │       vision0: aprilTags,
            │       vision1: aprilTags
            │
            ├─ manualShooting(x: double, y: double, Ω: double, hoodAngle: double) ->
            │       shooter: hoodAimed(hoodAngle),
            │       hopper: transitioning,
            │       drive: driving(x, y, Ω),
            │       vision0: aprilTags,
            │       vision1: aprilTags
            │
            ├─ intaking(x: double, y: double, Ω: double) ->
            │       shooter: hoodIdled,
            │       hopper: intaking,
            │       drive: driving(x, y, Ω),
            │       vision0: aprilTags,
            │       vision1: aprilTags
            │
  [DEFAULT] ├─ driving(x: double, y: double, Ω: double) ->
            │       shooter: hoodIdled,
            │       hopper: idled,
            │       drive: driving(x, y, Ω),
            │       vision0: aprilTags,
            │       vision1: aprilTags
            │
            └─ climbing ->
                    shooter: idled,
                    hopper: compacted,
                    drive: idled,
                    vision0: aprilTags,
                    vision1: aprilTags
```

# CAN IDs
1 - 8: Swerve [even steer, odd drive; FL, BL, BR, FR]
9 - 11: Shooter [L Flywheel, R Flywheel, Hood]
12, 13: Spindexer [Primary, Kicker]
14, 15: Intake [Pivot, Intake]
16: Climb
17: (maybe temp) Intake Kicker
20: Gyro

NOTE: Shooter L Flywheel & Intake Intake need to be updated to 2026 firmware

# AdvantageKit SparkMax Swerve Template with maple-sim

[Original Project](https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v4.0.0-beta-1/AdvantageKit_SparkSwerveTemplate.zip)

This is the AdvantageKit Swerve Template with REV SparkMax hardware, enhanced with maple-sim integration for improved chassis physics simulation.

Not many changes were made to the original project—only the necessary ones to implement maple-sim. See the [changelog from the original project](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/17cf2894008e4fa45492efa3937123494879ef81).

The control loops of the chassis run on REV motor controllers.  During simulation, they are replaced with WpiLib `PIDController`, so there might be slight differences between the real and simulated robot.

## Running the Simulation
Change `Constants.currentRobotMode` to `SIM` and run `simulateJava`.
Connect the robot from AdvantageScope, open [AdvantageScope Simulation Layout](./AdvantageScope%20Simulation.json), and drive your robot around!
