package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.libraries.bumpsim.FuelBumpSim;
import java.util.WeakHashMap;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.utils.mathutils.GeometryConvertor;

@SuppressWarnings("java:S110") /* library override, not abstraction */
public class ImprovedRebuiltFuelOnField extends RebuiltFuelOnField {

    private static final WeakHashMap<ImprovedRebuiltFuelOnField, ImprovedRebuiltFuelOnField> instances =
            new WeakHashMap<>();

    private final FuelBumpSim fuelBumpSim;

    private Pose3d lastSimPose3d = new Pose3d();

    public ImprovedRebuiltFuelOnField(Translation2d initialPosition) {
        super(initialPosition);
        fuelBumpSim = new FuelBumpSim();
        instances.put(this, this);
    }

    public ImprovedRebuiltFuelOnField(Pose2d initialPose, Translation2d initialVelocityMPS) {
        super(initialPose.getTranslation());

        super.setTransform(GeometryConvertor.toDyn4jTransform(initialPose));
        super.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocityMPS));

        fuelBumpSim = new FuelBumpSim();
        instances.put(this, this);
    }

    @Override
    public Pose3d getPose3d() {
        return lastSimPose3d;
    }

    public void update(int subTickNum) {
        Pose2d simPose = getPoseOnField();

        lastSimPose3d = fuelBumpSim.update(
                simPose, getLinearVelocity(), SimulatedArena.getSimulationSubTicksIn1Period(), subTickNum);
        if (fuelBumpSim.isOnRamp()) {
            super.setTransform(GeometryConvertor.toDyn4jTransform(fuelBumpSim.getSimWorldPose(simPose)));
        }
    }

    public static void updateAll(int subTickNum) {
        for (ImprovedRebuiltFuelOnField fuel : instances.keySet()) {
            fuel.update(subTickNum);
        }
    }
}
