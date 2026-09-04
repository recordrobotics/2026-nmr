# 2026-nmr — Team 6731 New Member Robot ("Minivan")

FRC 2026 _Rebuilt_. Java, WPILib 2026 / GradleRIO 2026.2.1, AdvantageKit, maple-sim,
PathPlanner, Phoenix 6, PhotonVision + Limelight, Ruckig.

**Full architecture reference: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).** Read it
before non-trivial work. The section numbers below refer to it.

## Size

137 Java files, ~1.0 MB total. Small enough to read directly — `build/` (~160 MB) is
untracked noise. Don't treat this as a repo too large to explore.

## Non-obvious rules (violating these produces silently broken code)

1. **Override `periodicManaged()`, never `periodic()`.** `ManagedSubsystemBase` declares
   `periodic()`/`simulationPeriodic()` `final` and routes through a custom
   `SubsystemManager`. Overriding the wrong managed variant compiles and never runs. (§3)
2. **Sim IO _extends_ real IO** — `TurretSim extends TurretReal`, not a sibling
   implementation of `TurretIO`. Sim reuses the real Phoenix 6 `TalonFX` objects and
   drives them via `getSimState()`. Only `ImuSimNavX` is a true sibling. (§4)
3. **`RobotContainer` is entirely static** with a private constructor invoked
   reflectively by `Robot.initContainer()`. Use `RobotContainer.shooter`; never `new` it.
4. **Mode switching is `Constants.RobotState.getMode()`** → `REAL`/`SIM`/`REPLAY`/`TEST`.
   `REPLAY` is unreachable without hand-editing that method. (§2)
5. **PathPlanner named commands are string-linked** into `src/main/deploy/pathplanner/`.
   Renaming one in `AutoPath.initialize()` breaks autos at runtime, not compile time.
   Most references are in `.path` event markers, **not** `.auto` files — grep both. (§9)
6. **Never let Elastic be first called in `teleopInit`/`autonomousInit`.** (§10)
7. **`Constants.RobotState.MOTOR_LOGGING_ENABLED` must be `false` at competition.** (§10)

## Where things live

| Need                                            | File                                                           |
| ----------------------------------------------- | -------------------------------------------------------------- |
| Wiring, subsystem construction, button bindings | `RobotContainer.java`                                          |
| Lifecycle, logging setup, sim arena setup       | `Robot.java`                                                   |
| Tuning values                                   | `Constants.java`                                               |
| CAN IDs / DIO ports                             | `RobotMap.java`                                                |
| A mechanism                                     | `subsystems/<N>.java` + `subsystems/io/{,real/,sim/}<N>*.java` |
| Shooting logic                                  | `subsystems/shootorchestrator/`                                |
| Autos                                           | `utils/AutoPath.java` + `src/main/deploy/pathplanner/`         |
| Driver controls                                 | `control/` (intent-named predicates in `AbstractControl`)      |
| Field physics / opponents                       | `utils/maplesim/`                                              |
| Vision                                          | `utils/camera/` + `subsystems/drive/PoseSensorFusion.java`     |
| 3D viz poses + fuel                             | `subsystems/RobotModel.java`                                   |

Subsystems: `Drivetrain`, `PoseSensorFusion`, `Imu`, `Intake`, `Turret`, `Shooter`,
`Indexer`, `Feeder`, `PowerDistributionPanel`, `RobotModel`, `FieldStateTracker`,
`ShootOrchestrator`, `LedManager`. **There is no climber** — it was removed at HEAD.

## Commands

```bash
./gradlew build          # tests do NOT run as part of build
./gradlew test           # only runs when named explicitly
./gradlew simulateJava   # simulation
./gradlew deploy         # to the RoboRIO
```

`simulateWithOpponents` / `simulateWithVerbose` hardcode a Windows `.dll` and **do not
work on macOS/Linux**; use `simulateJava` with `-Dopponent.<red|blue><1-3>=defense`. (§11)

## Do not trust

- **`docs/source/**` (Sphinx) is stale 2025 Reefscape** — documents `elevator`,
  `coral-intake`, `algae-grabber` etc., none of which exist now. `toolchain/` and
  `contributing/` pages are still broadly valid; `code-structure/` pages are not.
- `utils/maplesim/ImprovedSelfControlledSwerveDriveSimulation` — **dead file**, no
  `.java` extension so it never compiles. Superseded by the forked maple-sim vendordep.
- `utils/libraries/{LimelightHelpers,Elastic}.java` are vendored third-party — don't edit.
- `Main.java` is a 25-line WPILib stub padded with ~23 KB of ASCII art.

## Conventions

- Commit subjects prefixed **`[shared]`** are synced to the sibling repo
  `recordrobotics/2026-robot` by `sibling-sync.sh`. Use it for code both robots share;
  `[minivan]` or no prefix for robot-specific work. **The sync script force-pushes
  `main` — never run it without explicit user confirmation.** (§12)
- Conventional-ish commit style; Spotless + Checkstyle + ErrorProne + Sonar all gate CI.
  A pre-push hook runs `spotlessCheck` (`./gradlew spotlessInstallGitPrePushHook`).
- `*.glb` assets are Git LFS.
- Existing `@SuppressWarnings("java:S…")` annotations are deliberate; match local style.
