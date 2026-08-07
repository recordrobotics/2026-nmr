package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.WeakHashMap;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.utils.mathutils.GeometryConvertor;

@SuppressWarnings("java:S110") /* library override, not abstraction */
public class ImprovedRebuiltFuelOnField extends RebuiltFuelOnField {

    private static final WeakHashMap<ImprovedRebuiltFuelOnField, ImprovedRebuiltFuelOnField> instances =
            new WeakHashMap<>();

    private Pose3d lastSimPose3d = new Pose3d();

    public ImprovedRebuiltFuelOnField(Translation2d initialPosition) {
        super(initialPosition);
        instances.put(this, this);
    }

    public ImprovedRebuiltFuelOnField(Pose2d initialPose, Translation2d initialVelocityMPS) {
        super(initialPose.getTranslation());

        super.setTransform(GeometryConvertor.toDyn4jTransform(initialPose));
        super.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocityMPS));

        instances.put(this, this);
    }

    @Override
    public Pose3d getPose3d() {
        return lastSimPose3d;
    }

    public void update(int subTickNum) {
        lastSimPose3d = new Pose3d(BumpSim.updateFuel(this), Rotation3d.kZero);
    }

    public static void updateAll(int subTickNum) {
        for (ImprovedRebuiltFuelOnField fuel : instances.keySet()) {
            fuel.update(subTickNum);
        }
    }
}
