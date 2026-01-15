package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led.patterns.ChasePattern;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PoweredSubsystem;
import java.util.function.DoubleSupplier;

public class LedManager extends ManagedSubsystemBase implements PoweredSubsystem {

    private static final int PORT = 9;
    private static final int LED_COUNT = 30;
    private static final Distance LED_SPACING = Meters.of(1 / 120.0);

    // from https://www.alldatasheet.com/datasheet-pdf/download/1134588/WORLDSEMI/WS2815B.html
    private static final double LED_QUIESCENT_CURRENT_MILLIAMPS = 2.1;
    private static final double LED_RGB_CHANNEL_CONSTANT_CURRENT_MILLIAMPS = 10.0;

    private static final double BPS_DECAY_RATE = 0.9;
    private static final double ON_TARGET_DECAY_RATE = 0.5;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    @AutoLogLevel(level = Level.SIM)
    private final String[] hexStrings = new String[LED_COUNT];

    private final LEDPattern chasePatternPingPong = withBPS(new ChasePattern(
            Color.kBlue, Color.kBlack, 3.0, 15.0, Hertz.of(4.0), ChasePattern.ChaseDirection.PING_PONG));
    private final LEDPattern chasePatternDirectional = withBPS(new ChasePattern(
            Color.kBlue, Color.kRed, 3.0, 30.0, Hertz.of(2.0), ChasePattern.ChaseDirection.LEFT_TO_RIGHT));
    private final LEDPattern rainbowBPSPattern =
            withOnTarget(withBPS(LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1.0))));

    private double shotRatio = 0.0;
    private double onTargetRatio = 0.0;

    public LedManager() {
        led = new AddressableLED(PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);
        led.setLength(LED_COUNT);
        led.setColorOrder(ColorOrder.kBGR);
        led.start();
    }

    private static LEDPattern blendCustom(LEDPattern pattern, LEDPattern other, DoubleSupplier blendRatio) {
        return (reader, writer) -> {
            double ratio = blendRatio.getAsDouble();
            pattern.applyTo(reader, writer);

            other.applyTo(reader, (i, r, g, b) -> {
                int blendedRGB = Color.lerpRGB(reader.getRed(i), reader.getGreen(i), reader.getBlue(i), r, g, b, ratio);

                writer.setRGB(
                        i,
                        Color.unpackRGB(blendedRGB, Color.RGBChannel.kRed),
                        Color.unpackRGB(blendedRGB, Color.RGBChannel.kGreen),
                        Color.unpackRGB(blendedRGB, Color.RGBChannel.kBlue));
            });
        };
    }

    private LEDPattern withBPS(LEDPattern pattern) {
        return blendCustom(pattern, LEDPattern.solid(Color.kWhite), () -> shotRatio);
    }

    private LEDPattern withOnTarget(LEDPattern pattern) {
        return blendCustom(pattern, LEDPattern.solid(Color.kRed), () -> onTargetRatio);
    }

    @Override
    public void periodicManaged() {
        if (RobotContainer.feeder.isTopBeamBroken() || RobotContainer.feeder.isBottomBeamBroken()) {
            shotRatio = 1.0;
        } else {
            shotRatio *= BPS_DECAY_RATE;
        }

        if (!RobotContainer.shootOrchestrator.isOnTarget()) {
            onTargetRatio = 1.0;
        } else {
            onTargetRatio *= ON_TARGET_DECAY_RATE;
        }

        if (RobotState.isEnabled()) {
            if (RobotContainer.shootOrchestrator.isShootingEnabled()) {
                rainbowBPSPattern.applyTo(buffer);
            } else {
                chasePatternPingPong.applyTo(buffer);
            }
        } else {
            chasePatternDirectional.applyTo(buffer);
        }

        led.setData(buffer);

        if (Constants.RobotState.getMode() == Mode.SIM) {
            for (int i = 0; i < buffer.getLength(); i++) {
                hexStrings[i] = String.format("#%02X%02X%02X", buffer.getRed(i), buffer.getGreen(i), buffer.getBlue(i));
            }
        }
    }

    @Override
    public Current getCurrentDraw() {
        double currentMilliamps = 0.0;

        for (int i = 0; i < buffer.getLength(); i++) {
            double red = buffer.getRed(i) / 255.0;
            double green = buffer.getGreen(i) / 255.0;
            double blue = buffer.getBlue(i) / 255.0;

            currentMilliamps += LED_QUIESCENT_CURRENT_MILLIAMPS
                    + red * LED_RGB_CHANNEL_CONSTANT_CURRENT_MILLIAMPS
                    + green * LED_RGB_CHANNEL_CONSTANT_CURRENT_MILLIAMPS
                    + blue * LED_RGB_CHANNEL_CONSTANT_CURRENT_MILLIAMPS;
        }

        return Milliamps.of(currentMilliamps);
    }
}
