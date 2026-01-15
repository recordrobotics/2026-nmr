package frc.robot.subsystems.led.patterns;

import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class ChasePattern implements LEDPattern {

    public enum ChaseDirection {
        LEFT_TO_RIGHT,
        RIGHT_TO_LEFT,
        PING_PONG
    }

    private final Color colorChaser;
    private final Color colorBackground;
    private final double chaserHalfLengthMin;
    private final double chaserHalfLengthMax;
    private final double periodMicros;
    private final ChaseDirection direction;

    /**
     * @param colorChaser The color of the chaser
     * @param colorBackground The color of the background
     * @param chaserLengthMin The minimum length of the chaser, in terms of how many LEDs it should cover, full chaser color
     * @param chaserLengthMax The maximum length of the chaser, in terms of how many LEDs it should cover, fades to background
     * @param velocity how fast the pattern should move, in terms of how long it takes for the chaser to cross from one end of the strip to the other i.e. Percent.per(Second).of(25)
     * @param direction the direction of the chaser
     */
    public ChasePattern(
            Color colorChaser,
            Color colorBackground,
            double chaserLengthMin,
            double chaserLengthMax,
            Frequency velocity,
            ChaseDirection direction) {
        this.colorChaser = colorChaser;
        this.colorBackground = colorBackground;
        this.chaserHalfLengthMin = chaserLengthMin / 2.0;
        this.chaserHalfLengthMax = chaserLengthMax / 2.0;
        this.periodMicros = velocity.asPeriod().in(Microseconds);
        this.direction = direction;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        long now = RobotController.getTime();
        int bufLen = reader.getLength();

        // index should move by (buf.length) / (period)
        double t = (now % (long) periodMicros) / periodMicros;
        double offset = 0.0;

        if (direction == ChaseDirection.LEFT_TO_RIGHT) {
            offset = t * bufLen;
        } else if (direction == ChaseDirection.RIGHT_TO_LEFT) {
            offset = bufLen - t * bufLen;
        } else if (direction == ChaseDirection.PING_PONG) {
            double pingFactor = (now % (long) (2 * periodMicros)) / periodMicros;
            if (pingFactor > 1.0) {
                offset = bufLen - (pingFactor - 1.0) * bufLen;
            } else {
                offset = pingFactor * bufLen;
            }
        }

        for (int led = 0; led < bufLen; led++) {
            double distance = Math.abs(led - offset);

            if (direction == ChaseDirection.LEFT_TO_RIGHT || direction == ChaseDirection.RIGHT_TO_LEFT) {
                // wrap around for left to right and right to left
                distance = Math.min(distance, bufLen - distance);
            }

            if (distance < chaserHalfLengthMin) {
                writer.setLED(led, colorChaser);
            } else if (distance < chaserHalfLengthMax) {
                double ratio = (distance - chaserHalfLengthMin) / (chaserHalfLengthMax - chaserHalfLengthMin);

                int blendedColor = Color.lerpRGB(
                        colorChaser.red,
                        colorChaser.green,
                        colorChaser.blue,
                        colorBackground.red,
                        colorBackground.green,
                        colorBackground.blue,
                        ratio);

                writer.setRGB(
                        led,
                        Color.unpackRGB(blendedColor, Color.RGBChannel.kRed),
                        Color.unpackRGB(blendedColor, Color.RGBChannel.kGreen),
                        Color.unpackRGB(blendedColor, Color.RGBChannel.kBlue));
            } else {
                writer.setLED(led, colorBackground);
            }
        }
    }
}
