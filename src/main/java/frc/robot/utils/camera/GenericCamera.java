package frc.robot.utils.camera;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.wrappers.SafeAlert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A generic camera class that can be extended for specific camera types.
 * <p>Assumes the camera has an enabled/disabled state and can report connection status.
 */
public abstract class GenericCamera {

    private final LoggedNetworkBoolean enabledToggle;

    /**
     * The name of the camera.
     */
    private String name;

    /**
     * The physical camera type (describes properties like FOV, resolution, etc.).
     */
    private PhysicalCamera physicalCamera;

    /**
     * Whether the camera is enabled and being used.
     */
    private boolean enabled = true;

    /**
     * ERROR Alert for when the camera is disconnected.
     */
    private final SafeAlert disconnectedAlert;

    /**
     * WARNING Alert for when the camera is disabled.
     * <p>Less severe because the user intentionally disabled it.
     */
    private final SafeAlert disabledAlert;

    /**
     * Constructs a GenericCamera with the given name and physical camera type.
     * <p>Sets up dashboard entry for enabling/disabling the camera at the path "Camera/{name}/Enabled".
     * <p>Disconnected alert is set to show camera as disconnected.
     * @param name The name of the camera. This is used for network connection and logging.
     * @param physicalCamera The physical camera type.
     */
    protected GenericCamera(String name, PhysicalCamera physicalCamera) {
        this.name = name;
        this.physicalCamera = physicalCamera;

        this.disconnectedAlert = new SafeAlert("Camera " + name + " disconnected!", AlertType.kError);
        disconnectedAlert.set(true);

        this.disabledAlert = new SafeAlert("Camera " + name + " disabled!", AlertType.kWarning);
        disabledAlert.set(!enabled);

        // Allow camera enable/disable from dashboard
        enabledToggle = new LoggedNetworkBoolean(getPrefix() + "Enabled", enabled);
    }

    /**
     * Checks if the camera is enabled.
     * <p>When disabled, the camera is not used for any processing.
     * <p>For example if the camera is broken or the mount is bent it can be disabled.
     * @return True if the camera is enabled, false otherwise.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Sets whether the camera is enabled.
     * <p>When disabled, the camera is not used for any processing.
     * <p>For example if the camera is broken or the mount is bent it can be disabled.
     * @param enabled True to enable the camera, false to disable it.
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        enabledToggle.set(enabled);
    }

    /**
     * Gets the name of the camera. This is used for network connection and logging.
     * @return The name of the camera.
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the physical camera type. This describes properties like FOV, resolution, etc.
     * @return The physical camera type.
     */
    public PhysicalCamera getPhysicalCamera() {
        return physicalCamera;
    }

    /**
     * Checks if the camera is connected.
     * <p>This is different from being enabled. A camera can be enabled but not connected.
     * @return True if the camera is connected, false otherwise.
     */
    public abstract boolean isConnected();

    /**
     * Periodically updates the camera state.
     * <p>This method should be called regularly (e.g. in a main periodic loop).
     */
    public abstract void periodic();

    /**
     * Called after a match with the FMS connected ends to record a match replay if the camera supports it.
     * Can also be called manually during driver practice
     */
    public abstract void recordMatchReplay();

    /**
     * Gets the prefix used for logging camera values.
     * @return The prefix used for logging.
     */
    public final String getPrefix() {
        return "Camera/" + name + "/";
    }

    /**
     * Logs camera values and alerts.
     * Also updates the enabled state from dashboard.
     */
    public void logValues() {
        String prefix = getPrefix();

        Logger.recordOutput(prefix + "Connected", isConnected());
        Logger.recordOutput(prefix + "Enabled", isEnabled());

        // Update enabled state from dashboard
        enabled = enabledToggle.get();

        disconnectedAlert.set(!isConnected());
        disabledAlert.set(!isEnabled());
    }
}
