package frc.robot.tests.simple;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.SimpleMath;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class SimpleMathTests {

    private static final double DELTA = 1e-9;

    @Test
    void testConstants() {
        assertEquals(SimpleMath.PI2, Math.PI * 2, DELTA);
        assertEquals(SimpleMath.SQRT2, Math.sqrt(2), DELTA);
        assertEquals(60, SimpleMath.SECONDS_PER_MINUTE);
    }

    @Test
    void testRemapBasicCase() {
        double result = SimpleMath.remap(5, 0, 10, 0, 100);
        assertEquals(50, result, DELTA);
    }

    @Test
    void testRemapNegativeRanges() {
        double result = SimpleMath.remap(-5, -10, 0, 0, 100);
        assertEquals(50, result, DELTA);
    }

    @Test
    void testRemapOutsideBounds() {
        // Test that it doesn't clamp - value outside input range
        double result = SimpleMath.remap(15, 0, 10, 0, 100);
        assertEquals(150, result, DELTA);
    }

    @Test
    void testRemapZeroRange() {
        // Edge case: same input min and max (would cause division by zero)
        assertThrows(IllegalArgumentException.class, () -> SimpleMath.remap(5, 10, 10, 0, 100));
    }

    @Test
    void testRemapInverseRanges() {
        double result = SimpleMath.remap(3, 0, 10, 100, 0);
        assertEquals(70, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityWithinThreshold() {
        double result = SimpleMath.applyThresholdAndSensitivity(0.1, 0.2, 1.0);
        assertEquals(0, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityAboveThreshold() {
        double result = SimpleMath.applyThresholdAndSensitivity(0.6, 0.2, 1.0);
        assertEquals(0.5, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityNegativeInput() {
        double result = SimpleMath.applyThresholdAndSensitivity(-0.6, 0.2, 1.0);
        assertEquals(-0.5, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityWithSensitivity() {
        double result = SimpleMath.applyThresholdAndSensitivity(0.6, 0.2, 0.5);
        assertEquals(0.25, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityAtThreshold() {
        double result = SimpleMath.applyThresholdAndSensitivity(0.2, 0.2, 1.0);
        assertEquals(0, result, DELTA);
    }

    @Test
    void testApplyThresholdAndSensitivityAtMaximum() {
        double result = SimpleMath.applyThresholdAndSensitivity(1.0, 0.2, 1.0);
        assertEquals(1.0, result, DELTA);
    }

    @Test
    void testIsInFieldWithPose2d() {
        Pose2d insideField = new Pose2d(8, 4, new Rotation2d());
        assertTrue(SimpleMath.isInField(insideField));
    }

    @Test
    void testIsInFieldWithPose2dOnBoundary() {
        Pose2d insideField = new Pose2d(FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY, new Rotation2d());
        assertTrue(SimpleMath.isInField(insideField));
    }

    @Test
    void testIsOutsideFieldWithPose2dPositive() {
        Pose2d outsideField =
                new Pose2d(FlippingUtil.fieldSizeX + 0.1, FlippingUtil.fieldSizeY + 0.1, new Rotation2d());
        assertFalse(SimpleMath.isInField(outsideField));
    }

    @Test
    void testIsOutsideFieldWithPose2dNegative() {
        Pose2d outsideField = new Pose2d(-0.1, -0.1, new Rotation2d());
        assertFalse(SimpleMath.isInField(outsideField));
    }

    @Test
    void testIsInFieldWithTranslation2d() {
        Translation2d insideField = new Translation2d(8, 4);
        assertTrue(SimpleMath.isInField(insideField));
    }

    @Test
    void testIsInFieldWithTranslation2dOnBoundary() {
        Translation2d insideField = new Translation2d(FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY);
        assertTrue(SimpleMath.isInField(insideField));
    }

    @Test
    void testIsOutsideFieldWithTranslation2dPositive() {
        Translation2d outsideField = new Translation2d(FlippingUtil.fieldSizeX + 0.1, FlippingUtil.fieldSizeY + 0.1);
        assertFalse(SimpleMath.isInField(outsideField));
    }

    @Test
    void testIsOutsideFieldWithTranslation2dNegative() {
        Translation2d outsideField = new Translation2d(-0.1, -0.1);
        assertFalse(SimpleMath.isInField(outsideField));
    }

    @Test
    void testSlewRateLimitLinearNoChange() {
        double result = SimpleMath.slewRateLimitLinear(5.0, 5.0, 0.02, 10.0);
        assertEquals(5.0, result, DELTA);
    }

    @Test
    void testSlewRateLimitLinearWithinLimit() {
        double result = SimpleMath.slewRateLimitLinear(0.0, 0.1, 0.02, 10.0);
        assertEquals(0.1, result, DELTA);
    }

    @Test
    void testSlewRateLimitLinearExceedsLimit() {
        double result = SimpleMath.slewRateLimitLinear(0.0, 1.0, 0.02, 10.0);
        assertEquals(0.2, result, DELTA); // 10 * 0.02 = 0.2
    }

    @Test
    void testSlewRateLimitLinearNegativeDirection() {
        double result = SimpleMath.slewRateLimitLinear(1.0, 0.0, 0.02, 10.0);
        assertEquals(0.8, result, DELTA); // 1.0 - (10 * 0.02) = 0.8
    }

    @Test
    void testSlewRateLimitLinearNegativeMaxVelocity() {
        double result = SimpleMath.slewRateLimitLinear(0.0, 1.0, 0.02, -5.0);
        assertEquals(1.0, result, DELTA); // Should return next value when maxVelocity < 0
    }

    @Test
    void testSlewRateLimitLinearZeroMaxVelocity() {
        double result = SimpleMath.slewRateLimitLinear(0.0, 1.0, 0.02, 0.0);
        assertEquals(0.0, result, DELTA); // Should not move when maxVelocity is 0
    }

    @Test
    void testSlewRateLimitLinearVerySmallDifference() {
        double result = SimpleMath.slewRateLimitLinear(1.0, 1.0000000001, 0.02, 10.0);
        assertEquals(1.0000000001, result, DELTA); // Difference < 1e-9, should return next
    }

    @Test
    void testPovToVectorAllDirections() {
        assertEquals(new Translation2d(0, 1), SimpleMath.povToVector(0));
        assertEquals(new Translation2d(1, 1), SimpleMath.povToVector(45));
        assertEquals(new Translation2d(1, 0), SimpleMath.povToVector(90));
        assertEquals(new Translation2d(1, -1), SimpleMath.povToVector(135));
        assertEquals(new Translation2d(0, -1), SimpleMath.povToVector(180));
        assertEquals(new Translation2d(-1, -1), SimpleMath.povToVector(225));
        assertEquals(new Translation2d(-1, 0), SimpleMath.povToVector(270));
        assertEquals(new Translation2d(-1, 1), SimpleMath.povToVector(315));
    }

    @Test
    void testPovToVectorInvalidPov() {
        assertEquals(new Translation2d(0, 0), SimpleMath.povToVector(-1));
        assertEquals(new Translation2d(0, 0), SimpleMath.povToVector(360));
        assertEquals(new Translation2d(0, 0), SimpleMath.povToVector(123));
    }

    @Test
    void testIsWithinToleranceExactMatch() {
        assertTrue(SimpleMath.isWithinTolerance(5.0, 5.0, 0.1));
    }

    @Test
    void testIsWithinToleranceWithinTolerance() {
        assertTrue(SimpleMath.isWithinTolerance(5.05, 5.0, 0.1));
        assertTrue(SimpleMath.isWithinTolerance(4.95, 5.0, 0.1));
    }

    @Test
    void testIsWithinToleranceOutsideTolerance() {
        assertFalse(SimpleMath.isWithinTolerance(5.15, 5.0, 0.1));
        assertFalse(SimpleMath.isWithinTolerance(4.85, 5.0, 0.1));
    }

    @Test
    void testIsWithinToleranceAtBoundary() {
        assertTrue(SimpleMath.isWithinTolerance(5.1, 5.0, 0.1));
        assertTrue(SimpleMath.isWithinTolerance(4.9, 5.0, 0.1));
    }

    @Test
    void testIsWithinToleranceZeroTolerance() {
        assertTrue(SimpleMath.isWithinTolerance(5.0, 5.0, 0.0));
        assertFalse(SimpleMath.isWithinTolerance(5.0000001, 5.0, 0.0));
    }

    @Test
    void testPoseNoiseAddsNoise() {
        Pose2d originalPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.PI / 4));
        Pose2d noisyPose = SimpleMath.poseNoise(originalPose, 0.1, 0.1);

        // The noisy pose should be different (with very high probability)
        // We can't test exact values due to randomness, but we can test that method doesn't crash
        assertNotNull(noisyPose);
        assertNotNull(noisyPose.getTranslation());
        assertNotNull(noisyPose.getRotation());
    }

    @Test
    void testPoseNoiseZeroStdDev() {
        Pose2d originalPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.PI / 4));
        Pose2d noisyPose = SimpleMath.poseNoise(originalPose, 0.0, 0.0);

        // With zero standard deviation, should be very close to original
        assertEquals(originalPose.getX(), noisyPose.getX(), 0.01);
        assertEquals(originalPose.getY(), noisyPose.getY(), 0.01);
        assertEquals(
                originalPose.getRotation().getRadians(), noisyPose.getRotation().getRadians(), 0.01);
    }

    @Test
    void testClosestTargetSameAngle() {
        double result = SimpleMath.closestTarget(0.0, 0.0);
        assertEquals(0.0, result, DELTA);
    }

    @Test
    void testClosestTargetCoterminalAngles() {
        double resultPositive = SimpleMath.closestTarget(Math.PI, Math.PI);
        double resultNegative = SimpleMath.closestTarget(Math.PI, -Math.PI);
        assertEquals(resultPositive, resultNegative, DELTA);
    }

    @Test
    void testClosestTargetPositiveRotation() {
        double result = SimpleMath.closestTarget(Math.PI * 3, 0.0);
        assertEquals(Math.PI * 4, result, DELTA);
    }

    @Test
    void testClosestTargetNegativeRotation() {
        double result = SimpleMath.closestTarget(-Math.PI * 3, 0.0);
        assertEquals(-Math.PI * 2, result, DELTA);
    }

    @Test
    void testClosestTargetHalfRotation() {
        double result = SimpleMath.closestTarget(Math.PI, Math.PI);
        assertEquals(Math.PI, result, DELTA);
    }

    @Test
    void testClosestTargetBetweenRotations() {
        double result = SimpleMath.closestTarget(Math.PI * 1.5, 0.0);
        assertEquals(Math.PI * 2, result, DELTA);
    }

    @Test
    void testNormalizeAngleZero() {
        double result = SimpleMath.normalizeAngle(0.0);
        assertEquals(0.0, result, DELTA);
    }

    @Test
    void testNormalizeAnglePi() {
        double result = SimpleMath.normalizeAngle(Math.PI);
        assertEquals(-Math.PI, result, DELTA); // wraps around PI == -PI
    }

    @Test
    void testNormalizeAngleNegativePi() {
        double result = SimpleMath.normalizeAngle(-Math.PI);
        assertEquals(-Math.PI, result, DELTA);
    }

    @Test
    void testNormalizeAngleGreaterThanPi() {
        double result = SimpleMath.normalizeAngle(Math.PI * 1.5);
        assertEquals(-Math.PI / 2, result, DELTA);
    }

    @Test
    void testNormalizeAngleLessThanNegativePi() {
        double result = SimpleMath.normalizeAngle(-Math.PI * 1.5);
        assertEquals(Math.PI / 2, result, DELTA);
    }

    @Test
    void testNormalizeAngleMultipleRotations() {
        double result = SimpleMath.normalizeAngle(Math.PI * 5);
        assertEquals(-Math.PI, result, DELTA); // wraps around PI == -PI
    }

    @Test
    void testSignEqBothPositive() {
        assertTrue(SimpleMath.signEq(5.0, 3.0));
    }

    @Test
    void testSignEqBothNegative() {
        assertTrue(SimpleMath.signEq(-5.0, -3.0));
    }

    @Test
    void testSignEqBothZero() {
        assertTrue(SimpleMath.signEq(0.0, 0.0));
    }

    @Test
    void testSignEqOnePositiveOneNegative() {
        assertFalse(SimpleMath.signEq(5.0, -3.0));
        assertFalse(SimpleMath.signEq(-5.0, 3.0));
    }

    @Test
    void testSignEqWithZero() {
        assertTrue(SimpleMath.signEq(0.0, -0.0)); // -0.0 and 0.0 have same signum
        assertFalse(SimpleMath.signEq(0.0, 1.0));
        assertFalse(SimpleMath.signEq(0.0, -1.0));
    }

    @Test
    void testAverageSingleValue() {
        double result = SimpleMath.average(5.0).orElseThrow();
        assertEquals(5.0, result, DELTA);
    }

    @Test
    void testAverageMultipleValues() {
        double result = SimpleMath.average(1.0, 2.0, 3.0, 4.0, 5.0).orElseThrow();
        assertEquals(3.0, result, DELTA);
    }

    @Test
    void testAverageNegativeValues() {
        double result = SimpleMath.average(-1.0, -2.0, -3.0).orElseThrow();
        assertEquals(-2.0, result, DELTA);
    }

    @Test
    void testAverageMixedValues() {
        double result = SimpleMath.average(-1.0, 0.0, 1.0).orElseThrow();
        assertEquals(0.0, result, DELTA);
    }

    @Test
    void testAverageEmptyArray() {
        assertEquals(SimpleMath.average(), OptionalDouble.empty());
    }

    @Test
    void testRangeNormal() {
        int[] result = SimpleMath.range(3, 7);
        assertArrayEquals(new int[] {3, 4, 5, 6, 7}, result);
    }

    @Test
    void testRangeSingleValue() {
        int[] result = SimpleMath.range(5, 5);
        assertArrayEquals(new int[] {5}, result);
    }

    @Test
    void testRangeInvalid() {
        assertThrows(IllegalArgumentException.class, () -> SimpleMath.range(7, 3));
    }

    @Test
    void testInterpolateRotation2dUnboundedZeroFraction() {
        Rotation2d start = new Rotation2d(Math.PI / 4);
        Rotation2d end = new Rotation2d(Math.PI / 2);
        Rotation2d result = SimpleMath.interpolateRotation2dUnbounded(start, end, 0.0);
        assertEquals(start.getRadians(), result.getRadians(), DELTA);
    }

    @Test
    void testInterpolateRotation2dUnboundedOneFraction() {
        Rotation2d start = new Rotation2d(Math.PI / 4);
        Rotation2d end = new Rotation2d(Math.PI / 2);
        Rotation2d result = SimpleMath.interpolateRotation2dUnbounded(start, end, 1.0);
        assertEquals(end.getRadians(), result.getRadians(), DELTA);
    }

    @Test
    void testInterpolateRotation2dUnboundedHalfFraction() {
        Rotation2d start = new Rotation2d(0.0);
        Rotation2d end = new Rotation2d(Math.PI);
        Rotation2d result = SimpleMath.interpolateRotation2dUnbounded(start, end, 0.5);
        assertEquals(Math.PI / 2, result.getRadians(), DELTA);
    }

    @Test
    void testInterpolateRotation2dUnboundedGreaterThanPi() {
        Rotation2d start = new Rotation2d(0.0);
        Rotation2d end = new Rotation2d(Math.PI * 6);
        Rotation2d result = SimpleMath.interpolateRotation2dUnbounded(start, end, 0.5);
        assertEquals(Math.PI * 3, result.getRadians(), DELTA);
    }

    @Test
    void testInterpolateRotation2dUnboundedLessThanPi() {
        Rotation2d start = new Rotation2d(Math.PI * 2);
        Rotation2d end = new Rotation2d(-Math.PI * 6);
        Rotation2d result = SimpleMath.interpolateRotation2dUnbounded(start, end, 0.5);
        assertEquals(-Math.PI * 2, result.getRadians(), DELTA);
    }
}
