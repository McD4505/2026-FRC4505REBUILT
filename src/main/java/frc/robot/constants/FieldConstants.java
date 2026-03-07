package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static final double FIELD_LENGTH_M = 16.54;

    public static final Pose2d HUB_LOCATION = new Pose2d(
                !isRedSide() ? 4.75 : FIELD_LENGTH_M - 4.75, 4.035, Rotation2d.kZero);

    public static boolean isRedSide() {
        if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
                return false;
        }
        return true;
    }
}

