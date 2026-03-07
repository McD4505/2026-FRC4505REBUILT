package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.*;

import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static Command teleHalfShooterCommand(
        TurretSubsystem shooter,
        IntakeSubsystem hopper,
        CommandSwerveDrivetrain drive,
        DoubleSupplier joystickX,
        DoubleSupplier joystickY) {
        return Commands.parallel(
            hopper.setIntakeSpeedCommand(50),
            Commands.run(() -> {
                drive.pointTowardsPoint(HUB_LOCATION.getTranslation(), joystickX, joystickY);
        }, drive),

        setDesiredShootingStates(shooter, drive)
        );
    }
    public static Command setDesiredShootingStates(
        TurretSubsystem shooter,
        CommandSwerveDrivetrain drive) {

        return Commands.run(() -> {
            double distance = drive.getDistanceToHub(
                DriverStation.getAlliance().orElse(Alliance.Blue)
            );
            double speed = SHOOTER_VELOCITY_LOOKUP.get(distance);
            shooter.setTurretSpeed(speed);
        }, shooter);
    }
}
