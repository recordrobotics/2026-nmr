Drivetrain
=============================

Read through `swerve drive kinematics <https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html>`__

The ``SwerveModule`` class is used to run individual modules with desired position and velocity for each module calculated in drivetrain.

**public void drive(DriveCommandData driveCommandData)**
    Calculates and inputs swerve module positions based on drive inputs

**public void resetStartingPose()**
    Sets field zero to current location

**public void resetDriverPose()**
    Calibrates robot positon based on being in front of the speaker

**public ChassisSpeeds getChassisSpeeds()**
    Gets robot velocity from velocity and angle of each module

**public void setToPose(Pose2d pose)**
    Sets current location to inputed position
