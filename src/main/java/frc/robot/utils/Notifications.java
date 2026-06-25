package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.ArrayList;
import java.util.List;

public final class Notifications {

    private static List<SafeAlert> alerts = new ArrayList<>();

    private Notifications() {}

    public static void send(NotificationLevel type, String title, String description) {
        send(type, title, description, 0); // Yes 0 does work it makes it infinite
    }

    public static void send(NotificationLevel level, String title, String description, int desplayTimeMillis) {
        Notification notification = new Notification(level, title, description, desplayTimeMillis);
        Elastic.sendNotification(notification);

        AlertType alertType =
                switch (level) {
                    case INFO -> AlertType.kInfo;
                    case WARNING -> AlertType.kWarning;
                    case ERROR -> AlertType.kError;
                    default -> AlertType.kInfo;
                };

        SafeAlert alert = new SafeAlert(title + " - " + description, alertType);
        alerts.add(alert);
        alert.set(true);
    }
}
