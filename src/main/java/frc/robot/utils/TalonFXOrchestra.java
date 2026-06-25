package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TalonFXOrchestra extends SubsystemBase {

    public static final class Tracks {

        public static final int FL_DRIVE = 0;
        public static final int FR_DRIVE = 0;
        public static final int BL_DRIVE = 0;
        public static final int BR_DRIVE = 0;

        public static final int FL_TURN = 0;
        public static final int FR_TURN = 0;
        public static final int BL_TURN = 0;
        public static final int BR_TURN = 0;

        public static final int INTAKE_ARM_LEFT = 1;
        public static final int INTAKE_ARM_RIGHT = 1;
        public static final int INTAKE_WHEEL = 1;

        public static final int TURRET = 2;

        public static final int HOOD = 2;
        public static final int FLYWHEEL_LEFT = 1;
        public static final int FLYWHEEL_RIGHT = 1;

        public static final int SPINDEXER = 1;

        public static final int FEEDER = 1;

        public static final int CLIMBER = 2;

        private Tracks() {}
    }

    private static final LoggedNetworkBoolean playButton = new LoggedNetworkBoolean("TalonFXOrchestra/Play", false);
    private static final LoggedNetworkString musicPath = new LoggedNetworkString("TalonFXOrchestra/MusicPath", "");

    private Orchestra orchestra = new Orchestra();
    private String currentMusicPath = "";

    public TalonFXOrchestra() {
        new Trigger(playButton).onTrue(Commands.runOnce(orchestra::play)).onFalse(Commands.runOnce(orchestra::stop));
    }

    public void add(TalonFX motor, int trackNumber) {
        orchestra.addInstrument(motor, trackNumber);
    }

    @Override
    public void periodic() {
        if (!musicPath.get().equals(currentMusicPath)) {
            currentMusicPath = musicPath.get();
            if (!currentMusicPath.isEmpty() && !currentMusicPath.isBlank()) {
                orchestra.loadMusic(currentMusicPath);
            } else {
                orchestra.stop();
            }
        }

        Logger.recordOutput("TalonFXOrchestra/IsPlaying", orchestra.isPlaying());
    }
}
