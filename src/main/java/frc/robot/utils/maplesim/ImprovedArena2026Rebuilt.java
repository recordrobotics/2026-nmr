package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.utils.mathutils.GeometryConvertor;

public class ImprovedArena2026Rebuilt extends Arena2026Rebuilt {

    public ImprovedArena2026Rebuilt(boolean addRampColliders) {
        super(addRampColliders);
        super.addCustomSimulation(ImprovedRebuiltFuelOnField::updateAll);
    }

    // Intercept game pieces to wrap them around the ImprovedRebuiltFuelOnField class
    @Override
    public synchronized void addGamePiece(GamePieceOnFieldSimulation gamePiece) {
        Pose2d initialPose = gamePiece.getPoseOnField();
        Translation2d initialVelocityMPS = GeometryConvertor.toWpilibTranslation2d(gamePiece.getLinearVelocity());

        ImprovedRebuiltFuelOnField wrappedGamePiece = new ImprovedRebuiltFuelOnField(initialPose, initialVelocityMPS);

        super.addGamePiece(wrappedGamePiece);
    }
}
