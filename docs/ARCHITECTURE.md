# 2026-nmr Architecture Reference

Team 6731 (Record Robotics) — **New Member Robot**, codename **Minivan**.
Repo: `recordrobotics/2026-nmr` · Season: 2026 _Rebuilt_ · WPILib 2026 / GradleRIO 2026.2.1 / Java

> **Scope of this document.** Written by reading the source at commit `51c81f4`
> ("Remove all climber code"). It describes the code as it actually is, not as
> the Sphinx docs in `docs/source/` describe it — those are **stale 2025 Reefscape
> content** (see [Known stale / dead](#known-stale--dead)). Claims are marked with
> confidence in [Verification status](#verification-status); nothing here was
> confirmed by running a build or deploying.

---

## 1. Orientation map

| If you want to…                          | Go to                                                      |
| ---------------------------------------- | ---------------------------------------------------------- |
| See how everything is wired together     | `RobotContainer.java`                                      |
| Change the loop / mode behavior          | `Robot.java`                                               |
| Change a tuning number                   | `Constants.java`                                           |
| Change a CAN ID or DIO port              | `RobotMap.java`                                            |
| Add/modify a mechanism                   | `subsystems/<Name>.java` + `subsystems/io/<Name>IO.java`   |
| Change how a mechanism is simulated      | `subsystems/io/sim/<Name>Sim.java`                         |
| Change how a mechanism talks to hardware | `subsystems/io/real/<Name>Real.java`                       |
| Change shooting logic                    | `subsystems/shootorchestrator/`                            |
| Change autos                             | `utils/AutoPath.java` + `src/main/deploy/pathplanner/`     |
| Change driver controls                   | `control/`                                                 |
| Change the physics/field simulation      | `utils/maplesim/`                                          |
| Change 3D visualization poses            | `subsystems/RobotModel.java`                               |
| Change vision                            | `utils/camera/` + `subsystems/drive/PoseSensorFusion.java` |

**Where the bytes are.** ~253 tracked files, but only **137 Java files totaling
~1.0 MB**. The repo's on-disk bulk is `build/` (~160 MB, untracked). `assets/` is
~2.9 MB of AdvantageScope robot models and controller images. This codebase is
small enough to read end to end — do not treat it as a "too big to read" repo.

---

## 2. Boot sequence and the robot loop

```
Main.main()                        ← stock WPILib stub + ~23 KB of ASCII art. Ignore it.
  └─ RobotBase.startRobot(Robot::new)
       └─ Robot (extends LoggedRobot)
            ├─ constructor: build metadata → logging → DS → motor logging → sim arena
            ├─ robotInit(): Pathfinding → initContainer(RobotContainer.class) → Elastic tab
            └─ robotPeriodic(): control.update()
                                 → pose/fieldState end+startCalculation()
                                 → RobotContainer.robotPeriodic()
                                 → CommandScheduler.run()
                                 → AutoLogLevelManager.periodic()
```

`RobotContainer` is **entirely static** and is instantiated **reflectively** by
`Robot.initContainer()` (its constructor is private on purpose). Subsystems are
public static fields: `RobotContainer.drivetrain`, `RobotContainer.shooter`, etc.
There is no instance to pass around — reach for the static field.

### Run modes

`Constants.RobotState.getMode()` returns one of four modes and is the master
switch for nearly all conditional behavior:

| Mode     | Selected when                              | Realtime?                     |
| -------- | ------------------------------------------ | ----------------------------- |
| `REAL`   | `RobotBase.isReal()`                       | yes (20 ms)                   |
| `SIM`    | default off-robot                          | yes (20 ms)                   |
| `REPLAY` | **never automatically**                    | no (runs as fast as possible) |
| `TEST`   | `RobotState.setRunningAsUnitTest()` called | no                            |

> **Gotcha:** `REPLAY` is unreachable without editing source. `getMode()` ends with
> `return Mode.SIM; // change to REPLAY when replaying`. To replay an AdvantageKit
> log you must hand-edit that line.

---

## 3. The subsystem contract (custom — read this before writing a subsystem)

This repo does **not** use WPILib's subsystem periodic model directly. It layers
its own manager on top, and getting this wrong produces silently dead code.

### `ManagedSubsystemBase`

`periodic()` and `simulationPeriodic()` are declared **`final`** and simply
delegate. You must override one of:

| Override                        | Called                          | Notes                                                      |
| ------------------------------- | ------------------------------- | ---------------------------------------------------------- |
| `periodicManaged()`             | by `SubsystemManager`           | **The normal choice.**                                     |
| `simulationPeriodicManaged()`   | by `SubsystemManager`, sim only |                                                            |
| `periodicUnmanaged()`           | by WPILib's `SubsystemBase`     | Not multithread-safe; "only if you know what you're doing" |
| `simulationPeriodicUnmanaged()` | by WPILib, sim only             | same caveat                                                |

> **Gotcha:** overriding `periodic()` is a compile error (it's `final`); overriding
> the wrong _managed_ variant compiles fine and never runs. Default to
> `periodicManaged()`.

Registration is automatic — the `ManagedSubsystemBase` constructor calls
`SubsystemManager.getInstance().registerSubsystem(this)`.

### `SubsystemManager`

Singleton extending `SubsystemBase`, holding a `LinkedHashMap` of managed
subsystems, a `Watchdog`, and a cached thread pool. **`USE_MULTITHREADING = false`**
currently, so periodic calls run sequentially — but the machinery for parallel
subsystem execution exists and the "unmanaged" warnings above exist because of it.

### The three mixin interfaces

| Interface                                                            | Contract                                                                                             | Used for                                  |
| -------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | ----------------------------------------- |
| `KillableSubsystem` (abstract class, extends `ManagedSubsystemBase`) | `setForceDisabled(bool, cyclesBeforeEnable)`, auto re-enables after N cycles                         | The driver "kill" trigger                 |
| `PositionedSubsystem`                                                | `resetToStartPosition()`, `getPositionStatus()` → `UNKNOWN`/`KNOWN`/`RESETTING_FAULT`/`SENSOR_FAULT` | Encoder-reset workflow + dashboard alerts |
| `PoweredSubsystem`                                                   | `getCurrentDraw()` → `Current`                                                                       | Feeds the maple-sim simulated battery     |

`PositionedSubsystemManager` publishes a `Positioned/<Name>/OverrideKnown`
NetworkTables boolean per subsystem and raises WPILib `Alert`s from the status.

---

## 4. The IO layer (diverges from stock AdvantageKit — important)

Each hardware-touching subsystem is split three ways:

```
subsystems/io/TurretIO.java          ← interface + @AutoLog TurretIOInputs
subsystems/io/real/TurretReal.java   ← implements TurretIO   (real TalonFX objects)
subsystems/io/sim/TurretSim.java     ← EXTENDS TurretReal    (adds physics)
```

> **This is the single most important architectural fact in the repo.**
> In stock AdvantageKit, `XReal` and `XSim` are _sibling_ implementations of `XIO`.
> Here **`XSim extends XReal`**. The sim class reuses the _real_ Phoenix 6 `TalonFX`
> objects and drives them through CTRE's `getSimState()`, feeding in `DCMotorSim` /
> `LinearSystemId` physics. So in simulation the real device objects are still
> constructed and still hold configuration.

Confirmed uniform across the codebase:

| IO family     | Real                                         | Sim                                                   |
| ------------- | -------------------------------------------- | ----------------------------------------------------- |
| Feeder        | `FeederReal implements FeederIO`             | `FeederSim extends FeederReal`                        |
| Indexer       | `IndexerReal implements IndexerIO`           | `IndexerSim extends IndexerReal`                      |
| Intake        | `IntakeReal implements IntakeIO`             | `IntakeSim extends IntakeReal`                        |
| Shooter       | `ShooterReal implements ShooterIO`           | `ShooterSim extends ShooterReal`                      |
| Turret        | `TurretReal implements TurretIO`             | `TurretSim extends TurretReal`                        |
| Swerve module | `SwerveModuleReal implements SwerveModuleIO` | `SwerveModuleSim extends SwerveModuleReal`            |
| IMU (Pigeon2) | `ImuPigeon2 implements ImuIO`                | `ImuSimPigeon2 extends ImuPigeon2`                    |
| IMU (NavX)    | `ImuNavX implements ImuIO`                   | **`ImuSimNavX implements ImuIO`** ← the one exception |

The NavX exception exists because NavX has no CTRE-style sim state to drive, so it
is a true sibling implementation.

**Selection happens in one place** — `RobotContainer.initialize()`:

```java
if (Constants.RobotState.getMode() == Mode.REAL) {
    intake = new Intake(new IntakeReal());  // …turret, shooter, indexer, feeder
} else {
    intake = new Intake(new IntakeSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
    // …
}
```

Note the sim constructors take `ROBOT_PERIODIC` (0.02) and, for the intake, the
maple-sim drive simulation — sim IO is coupled to the physics arena.

---

## 5. Subsystem inventory

| Subsystem                | Base / mixins                       | Hardware (see `RobotMap`)          | Notes                                                                                    |
| ------------------------ | ----------------------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------- |
| `Drivetrain`             | `ManagedSubsystemBase`              | swerve modules                     | Config loaded from `src/main/deploy/swerve/motors.json`; throws `InvalidConfigException` |
| `SwerveModule`           | plain class                         | drive + steer motors               | Per-module; see `utils/ModuleConstants`                                                  |
| `Imu`                    | `ManagedSubsystemBase`              | Pigeon 2 (ID 24) or NavX           | Two IO families                                                                          |
| `PoseSensorFusion`       | `ManagedSubsystemBase`              | cameras + IMU                      | Pose estimation; `start/endCalculation()` split                                          |
| `Intake`                 | `Killable`, `Powered`, `Positioned` | arm L/R 16/17, wheel 18            | State machine: `STARTING`/`INTAKE`/`OUT`/`RETRACTED`/`EJECT`                             |
| `Turret`                 | `Killable`, `Powered`, `Positioned` | motor 19; DIO limit switches 2/3/4 | 3 magnetic limit switches; >1 hit simultaneously = fault                                 |
| `Shooter`                | `Killable`, `Powered`, `Positioned` | hood 20, flywheels 21/22           |                                                                                          |
| `Indexer`                | `Killable`, `Powered`               | motor 14                           |                                                                                          |
| `Feeder`                 | `Killable`, `Powered`               | motor 15; beam breaks DIO 0/1      | Top + bottom beam break                                                                  |
| `PowerDistributionPanel` | `SubsystemBase`                     | PDP/PDH                            | Also registers sim devices                                                               |
| `RobotModel`             | `ManagedSubsystemBase`              | none                               | 3D mechanism poses + `FuelManager`                                                       |
| `FieldStateTracker`      | `ManagedSubsystemBase`              | object-detection cameras           | Multi-object tracking                                                                    |
| `ShootOrchestrator`      | `ManagedSubsystemBase`              | none                               | Shot solution + sequencing                                                               |
| `LedManager`             | —                                   | LEDs                               | `led/patterns/ChasePattern`                                                              |

### `ShootOrchestrator` — the game-logic core

Owns two `ShotCalculator` strategies: `HubRegressionCalculator` (scoring into the
Hub) and `PassingRegressionCalculator` (passing across the field). Modes:

- `FeedMode` — how fuel is fed to the shooter
- `FeedForwardSource` — chosen via `LoggedDashboardChooser`
- fixed mode — a hardcoded shot (`FIXED_SHOT_CALCULATION`, 90° turret) for when
  solving is unwanted

Extensive NetworkTables overrides exist for tuning: `SHOTFEED`, `SHOOT_OVERRIDE`,
`SHOOT_ANGLE_OFFSET`, plus hood-angle and velocity overrides. `isOnTarget()` and
`getShotTimeOfFlight()` are consumed by `RobotContainer.shouldBeShooting()`.

### `FieldStateTracker` — object tracking

Tracks detected field objects (fuel) over time. Associates new detections to
existing tracks using the **Hungarian algorithm** (`HungarianWorkspace`), with a
0.5 m association distance threshold, 5 s object timeout, and a 4.0 m/s max object
velocity gate. Objects blend measurements within a 0.01 s window.

### `RobotModel` — visualization + fuel

Not a control subsystem. Produces `Pose3d[] mechanismPoses` for AdvantageScope
(the `assets/Robot_Minivan/` model), via `IntakeModel` (2 poses, arm geometry) and
`ShooterModel` (2 poses, turret + hood). Also hosts the static `FuelManager`
(fuel diameter 0.15 m, preload, animations) shared with the simulation.

---

## 6. Simulation architecture

Simulation is **first-class here**, not an afterthought. Three layers stack:

1. **Motor/mechanism physics** — per-subsystem `io/sim/*Sim` classes using WPILib
   `DCMotorSim` / `LinearSystemId` driving CTRE `getSimState()`.
2. **Field physics** — [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim),
   a rigid-body field simulation, pinned to a **team fork**: `0.4.0-beta-rr-fix-3`.
3. **Vision simulation** — PhotonVision `VisionSystemSim` and/or maple-sim cameras.

### maple-sim customizations (`utils/maplesim/`)

| File                                          | Purpose                                                                                                                                                                                                            |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `ImprovedArena2026Rebuilt`                    | Subclass of maple-sim's arena. Installed via `SimulatedArena.overrideInstance(...)` in `Robot`'s constructor with ramp colliders **disabled** ("we can go over the bump") and efficiency mode off ("all the fuel") |
| `ImprovedRebuiltFuelOnField`                  | Fuel game-piece model                                                                                                                                                                                              |
| `ImprovedMapleMatch`                          | Match state; `periodic()`, `autonomousInit()`, `teleopInit()`, `disabledInit()` driven from `Robot`                                                                                                                |
| `OpponentRobot`                               | Simulated opposing robots — see below                                                                                                                                                                              |
| `BumpSim`                                     | Simulates driving over the field bump                                                                                                                                                                              |
| `SimulatedBatteryFactory`                     | Modifies `SimulatedBattery.ROBORIO_BATTERY`; subsystems register current draw via `RobotContainer.registerPoweredSubsystems(...)`                                                                                  |
| `CustomPathfindingCommand`                    | Pathfinding variant for sim                                                                                                                                                                                        |
| `ImprovedSelfControlledSwerveDriveSimulation` | **DEAD** — see [Known stale / dead](#known-stale--dead)                                                                                                                                                            |

### Opponent robots

`OpponentRobot.createDefaultOpponents()` runs in `robotInit()` under `SIM`.
Behavior per robot is read from **JVM system properties**:

```
-Dopponent.red1=defense  -Dopponent.red2=…  -Dopponent.red3=…
-Dopponent.blue1=…       -Dopponent.blue2=…  -Dopponent.blue3=…
-Dshootbps=15.4
```

The `simulateWithOpponents` Gradle task sets all six to `defense`.

### Vision simulation modes

`Constants.Vision.VisionSimulationMode` includes a `PHOTON_SIM_INACCURATE` mode
that deliberately perturbs the AprilTag layout (`SimpleMath.addNoiseToAprilTagFieldLayout`)
for all **non-Hub** tags — position σ 0.2 m, rotation σ 0.1 rad — to simulate
field-element misalignment. There are also `ObjectDetectionSimulationMode` and
`PhotonVisionSimPerformanceMode` enums.

---

## 7. Vision (`utils/camera/`)

```
GenericCamera
└─ PhysicalCamera
   └─ PositionedCamera
      ├─ poseestimation/PoseEstimationCamera   → io/{LimelightCamera, PhotonVisionCamera, MapleSimCamera}
      └─ objectdetection/ObjectDetectionCamera → io/{LimelightCamera, PhotonVisionCamera, MapleSimCamera}
```

Both branches have three interchangeable backends (Limelight, PhotonVision,
maple-sim). `Cameras.java` is the registry. `utils/libraries/LimelightHelpers.java`
is the vendored 68 KB Limelight helper — **third-party, do not hand-edit**.

Pose estimates flow into `PoseSensorFusion`. Match replay recording is supported
on LL4 cameras (`RobotContainer.recordMatchReplay()`, last 2:45).

---

## 8. Controls (`control/`)

`AbstractControl` is an interface of intent-named predicates, not raw buttons:

```
isIntakePressed()  isIntakeUpPressed()  isIntakeInvertPressed()  isReverseIntakePressed()
isDefenseModePressed()  isShooterPassPressed()  isShooterDisableShootPressed()
isUnstuckIndexerPressed()  isIntakeRelativePressed()  isSlowSpeedPressed()
isPoseResetTriggered()  isKillTriggered()
getDrivetrainControl()  getRawDriverInput()  vibrate()  hasUserInput()
```

Implementations: `GamepadControls` (wrapping `XboxController` or
`SwitchController`) and `JoystickControls`. Registered in `RobotContainer.addControls(...)`
and chosen at runtime through the `Drive Mode` `LoggedDashboardChooser`.

Ports: gamepad `0`, joystick `2` (`RobotContainer.CONTROL_GAMEPAD_PORT` /
`CONTROL_JOYSTICK_PORT`).

`AbstractControl.orientXY()` flips driver input by alliance — **alliance-relative
driving is handled here**, not in the drivetrain.

`utils/modifiers/` layers transformations onto drivetrain control
(`IDrivetrainControlModifier`, `AutoControlModifier`, `OneshotControlModifier`,
orchestrated by `ControlModifierService`).

### Button bindings

All bindings live in `RobotContainer.configureTriggers()` as WPILib `Trigger`s
polling the `AbstractControl` predicates. The kill trigger is notable:

```java
new Trigger(() -> getControl().isKillTriggered())
    .whileTrue(CommandUtils.setForceDisabledForAllCommand(true, 2, intake, shooter, feeder, indexer, turret).repeatedly())
    .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
```

Subsystems auto-re-enable after 2 periodic cycles unless the trigger is held.

---

## 9. Autonomous

PathPlanner-based. Assets in `src/main/deploy/pathplanner/`:

- **Autos** (8): `Center`, `LeftBump`, `LeftTrench`, `LeftTrenchBump`,
  `LeftTrenchWithDepot`, `RightBump`, `RightTrench`, `RightTrenchBump`
- **Paths** (14), plus `navgrid.json`, `settings.json`, and a `mirror.py` helper
  for mirroring paths left↔right

`AutoPath.initialize()` registers PathPlanner **named commands** — these strings
are what the `.auto` files reference:

| Named command | Effect                                              |
| ------------- | --------------------------------------------------- |
| `Intake`      | intake → `INTAKE` (continuous)                      |
| `IntakeOff`   | intake → `OUT` (once)                               |
| `IntakeDepot` | intake → `INTAKE`, 2.2 s timeout                    |
| `Mixer`       | alternates `RETRACTED`/`OUT` every 0.5 s, repeating |
| `Pass`        | enables shooting for the duration                   |

> **Gotcha:** the linkage is by string and fails at runtime, not compile time. If you
> rename a named command in Java you must update the deploy JSON too — and note that
> **most references live in `.path` event markers, not the `.auto` files.** Verified
> reference counts across `src/main/deploy/pathplanner/`: `Intake` 26, `IntakeOff` 8+,
> `Mixer` 7+, `Pass` 4, `IntakeDepot` 2 — with only `IntakeOff` and `Mixer` appearing
> in `.auto` files at all. Grep **both** directories:
>
> ```bash
> grep -rn '"MyCommand"' src/main/deploy/pathplanner/
> ```
>
> (`Constraints Zone` also appears in that grep — it is a PathPlanner built-in, not a
> named command.) All five registered commands are currently in use. HEAD's commit was
> preceded by `Delete obselete "Hopper" NamedCommand`, so this has bitten the team before.

`AutoBuilder.configure(...)` wires pose supply/reset to `PoseSensorFusion`.
Config comes from `RobotConfig.fromGUISettings()`, falling back to
`Constants.Swerve.PP_DEFAULT_CONFIG` on failure (logged, non-fatal).
`PlannedAuto` + `IAutoRoutine` + `AutoPath.setupAutoChooser()` drive selection.
Pathfinding uses `LocalADStarAK` (the AdvantageKit-compatible AD* pathfinder),
installed in `robotInit()`.

`SysIdManager` short-circuits `getAutonomousCommand()` when SysId is enabled —
**if autos mysteriously don't run, check SysId first.**

---

## 10. Logging and telemetry

- **AdvantageKit** (`LoggedRobot`) with `WPILOGWriter` + `NT4Publisher`.
  Logs → `/home/lvuser/logs` on the RIO, `logs/` in sim.
- **`AutoLogLevel` / `AutoLogLevelManager`** — a custom reflective logging layer.
  Objects register via `AutoLogLevelManager.addObject(...)` (both `Robot` and
  `RobotContainer` do). Levels: `REAL`, `SIM`, `SYSID`, chosen by
  `Constants.RobotState.AUTO_LOG_LEVEL`.
- **CTRE `SignalLogger`** — gated behind `Constants.RobotState.MOTOR_LOGGING_ENABLED`
  (currently `false`). Enabling it prints a 10× warning to delete old logs, and it
  must be **off during competition**.
- **Elastic** dashboard (`utils/libraries/Elastic.java`, vendored) — tab switching
  (`Autonomous`, `Overview`, `Defense`) and notifications. Layout served from
  `src/main/deploy/elastic-layout.json` over a webserver on **port 5800**.
  > **Gotcha, verbatim from the source:** `MAKE SURE FIRST CALL TO ELASTIC IS NOT
IN TELEOP OR AUTO INIT!!`
- **`ExtendedStatusSignalCollection`** — batched Phoenix 6 status-signal refresh,
  called once per loop from `RobotContainer.robotPeriodic()`.

---

## 11. Build, run, test

Standard GradleRIO:

```bash
./gradlew build          # compile (tests are NOT run as part of build)
./gradlew test           # unit tests — only run when named explicitly
./gradlew simulateJava   # run the simulation
./gradlew deploy         # deploy to the RoboRIO
```

Project-specific tasks:

```bash
./gradlew replayWatch                 # AdvantageKit ReplayWatch
./gradlew tuning --args "swerveencoders"   # write current swerve offsets into motors.json
./gradlew simulateWithVerbose         # sim + JVM -verbose:class
./gradlew simulateWithOpponents       # sim with 6 'defense' opponent robots
```

> **Gotcha (macOS/Linux):** `simulateWithVerbose` and `simulateWithOpponents`
> hardcode `HALSIM_EXTENSIONS` to `halsim_ds_socket.dll` — **Windows-only**. On a
> Mac these will not load the DS socket extension. Use `simulateJava` with
> `-Dopponent.*` properties instead, or fix the task to select the platform
> library extension.

### Testing

JUnit 5 (`6.1.0-M1`), `forkEvery = 1`, parallel forks. Tests live in
`src/test/java/`:

- `utils/TestRobot.java` — harness that runs the real `Robot` in `TEST` mode.
  `Robot.setPeriodicRunnable()` and `Robot.loopFunc()` exist solely for this.
- `frc/robot/tests/TestControlBridge.java` — injects a fake `AbstractControl`
  via `RobotContainer.setTestControl(...)`.
- `tests/simple/` — pure unit tests (`SimpleMathTests`,
  `FastPolygonIntersectionTests`, `FieldIntersectionTests`).

`Constants.RobotState.UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE` publishes test runs to
NetworkTables — **single test only, parallel unsupported.**

### Code quality gates

Spotless (formatting), Checkstyle 11.0.0 (`config/checkstyle/`), ErrorProne
2.42.0, SonarCloud, JaCoCo. A **pre-push hook** runs `spotlessCheck`:

```bash
./gradlew spotlessInstallGitPrePushHook
```

CI (`.github/workflows/ci.yml`) runs build → test → JaCoCo → Sonar on every push
and PR.

### Git LFS

`.gitattributes` tracks `*.glb` via LFS (the AdvantageScope robot models in
`assets/Robot_Minivan/`). A fresh clone needs `git lfs install`.

---

## 12. Sibling repo synchronization ⚠️

This repo is one of a **pair**. `sibling-sync.sh` / `sibling-sync.ps1` synchronize
shared code with **`recordrobotics/2026-robot`** (the competition robot).

How it works:

- Only commits whose subject starts with **`[shared]`** are synced.
- Sync is by `git cherry-pick -x`, so lineage is tracked via the
  `(cherry picked from commit <sha>)` trailer.
- The script errors out on double-cherry-picks and on same-subject collisions
  caused by manual cherry-picking.
- If an origin commit was picked from a sibling commit no longer in
  `sibling/main`'s lineage, the script **rebases it out and force-pushes `main`**.

> **This script force-pushes `main`.** Do not run it casually, and do not run it
> on behalf of a user without explicit confirmation.
>
> **Convention:** if you change code that both robots share, prefix the commit
> subject with `[shared]`. Robot-specific commits use `[minivan]` or no prefix.

---

## 13. Field/game vocabulary (2026 _Rebuilt_)

Useful when reading identifiers:

| Term                                         | Meaning                                                  |
| -------------------------------------------- | -------------------------------------------------------- |
| **Fuel**                                     | The game piece (0.15 m sphere)                           |
| **Hub**                                      | The scoring target; cycles **active/inactive** over time |
| **Shift**                                    | A time window during which the Hub is active or not      |
| **Trench**, **Bump**, **Depot**, **Outpost** | Field features; appear in auto/path names                |
| **Pass**                                     | Lobbing fuel across the field rather than scoring        |

`RobotContainer.shouldBeShooting()` encodes the shift timing: in `AUTO` shoot mode
it starts early if the _next_ shift will be active within the shot's time of
flight, and keeps shooting after deactivation for up to `HUB_SCORE_TIME` (1 s),
because fuel takes ~1.25 s (`HUB_SCORE_REGISTER_TIME`) to register after crossing
the Hub. `ShootMode` = `DISABLED` / `AUTO` / `FORCE` / `FIXED`.

---

## 14. Known stale / dead

Anything in this section is **out of date or unused** — do not trust it as
documentation of current behavior.

| Item                                                         | Status                                                                                                                                                                                                                                                                                                                                                              |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`docs/source/**` (Sphinx, 36 `.rst`)**                     | **Stale — 2025 Reefscape.** Documents `coral-intake`, `coral-detection`, `elevator`, `elevator-arm`, `elevator-head`, `algae-grabber-lights` — **none of which exist** in the 2026 code. The `toolchain/` and `contributing/` pages are still broadly valid; the `code-structure/` pages are not.                                                                   |
| `utils/maplesim/ImprovedSelfControlledSwerveDriveSimulation` | **Dead file, ~24 KB.** Has **no `.java` extension**, so it is never compiled. It declares `package org.ironmaple.simulation.drivesims` — a local patch shadowing a maple-sim internal. `OpponentRobot` imports the class from the vendor jar instead. Superseded by the team's forked `maple-sim 0.4.0-beta-rr-fix-3`. Safe to delete; confirm with the team first. |
| Climber                                                      | **Already removed** at HEAD (`51c81f4 Remove all climber code`). If you find a climber reference, it is a leftover.                                                                                                                                                                                                                                                 |
| Hopper `NamedCommand`                                        | Removed (`51c81f4`'s predecessor). `RobotModel.IntakeModel` still has `resetHopperExtension()` — that is _live_ (called from `Robot.autonomousInit()` and `RobotContainer.resetEncoders()`), a different thing from the deleted auto command.                                                                                                                       |
| CI container                                                 | `wpilib/roborio-cross-ubuntu:**2025**-24.04` on a 2026 project. Works, but is a version behind.                                                                                                                                                                                                                                                                     |
| README RTD badge                                             | Points at `readthedocs/2024-control` — a 2024 project slug.                                                                                                                                                                                                                                                                                                         |
| `.vscode/settings.mac.json`                                  | Contains Windows-style paths (`${workspaceFolder}\\docs\\build`) in the `esbonio.*` keys.                                                                                                                                                                                                                                                                           |

---

## 15. Conventions and gotchas

1. **Override `periodicManaged()`, never `periodic()`.** (§3)
2. **`XSim extends XReal`**, not a sibling implementation. (§4)
3. **`RobotContainer` is static.** Access subsystems as `RobotContainer.shooter`.
   Its constructor is private and called reflectively — don't `new` it.
4. **`REPLAY` mode requires editing `Constants.RobotState.getMode()`.** (§2)
5. **PathPlanner named commands are string-linked.** Renaming in Java without
   updating the `.auto` files fails at runtime. (§9)
6. **Elastic must not be first called in `teleopInit`/`autonomousInit`.** (§10)
7. **`MOTOR_LOGGING_ENABLED` must be `false` at competition.** (§10)
8. **`[shared]` commit prefix** governs sibling sync; the sync script force-pushes. (§12)
9. **Two custom Gradle sim tasks are Windows-only.** (§11)
10. **Units:** WPILib unit types (`Current`, `Distance`, `Time`) are used at
    boundaries, but raw doubles dominate internally — names carry the unit
    (`positionRotations`, `velocityRotationsPerSecond`, `...Radians`, `...Mps`).
    `utils/wrappers/` holds `ImmutableCurrent`, `ImmutableTime`, and immutable
    `Pose2d`/`Translation2d` wrappers.
11. **Vendored third-party files** — `utils/libraries/LimelightHelpers.java` and
    `utils/libraries/Elastic.java`. Upstream copies; don't hand-edit.
12. **`Main.java` is ~23 KB of ASCII art** wrapped around a 25-line stub. Skip it.
13. Sonar/Checkstyle suppressions (`@SuppressWarnings("java:S…")`) are used
    deliberately throughout; match the surrounding style rather than removing them.

---

## Verification status

**Verified by reading source at `51c81f4`:** run modes and `getMode()`; the
`ManagedSubsystemBase`/`SubsystemManager`/`KillableSubsystem`/`PositionedSubsystem`/
`PoweredSubsystem` contracts; the full `XSim extends XReal` table; `RobotContainer`
subsystem construction, triggers, and choosers; `Robot`'s lifecycle; `RobotMap` CAN/DIO
IDs; `Constants` structure and `RobotState`; PathPlanner named commands and the
deploy asset inventory; `build.gradle` tasks/plugins/dependencies; `ci.yml`;
`.gitattributes`; `sibling-sync.sh`; `AbstractControl`'s method set; class outlines of
`ShootOrchestrator`, `FieldStateTracker`, `RobotModel`, `AutoPath`.

**Surveyed but not read line by line** — treat summaries as directional and read
the file before relying on details: `Drivetrain`, `PoseSensorFusion`, `SwerveModule`,
`Imu` internals; `RuckigAlign`, `WaypointAlign`, `ShootTuning`, `Vibrate`;
`ShootOrchestrator.periodicManaged()` math; `FieldStateTracker`'s Hungarian
implementation; `RobotModel` geometry; `utils/camera/**` internals;
`utils/field/**`; `led/**`; `utils/modifiers/**`; `build.utils.tuning.**`;
all `Constants` numeric values.

**Not verified at all:** that `./gradlew build`, `test`, `simulateJava`, or
`deploy` succeed — **no build was executed while writing this document.** Behavior
on real hardware. Whether the 2025 Sphinx pages have any remaining accurate
`code-structure` content.

---

_Generated from source inspection at commit `51c81f4`. When this drifts, fix it or
mark the drifted section stale — an unmarked stale doc is worse than none._
