package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeoFxSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.*;

import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static Command teleHalfShooterCommand(
        TurretSubsystem top_shooter,
        NeoFxSubsystem feeder,
        IntakeSubsystem hopper,
        CommandSwerveDrivetrain drive,
        DoubleSupplier joystickX,
        DoubleSupplier joystickY
    ) {
        return new ParallelCommandGroup(
            hopper.setIntakeSpeedCommand(50),
            // directly use the command returned by pointTowardsPoint
            drive.pointTowardsPoint(HUB_LOCATION.getTranslation(), joystickX, joystickY),
            setDesiredShootingStates(top_shooter, feeder, drive)
        );
    }

    public static Command setDesiredShootingStates(
        TurretSubsystem shooter,
        NeoFxSubsystem feeder,
        CommandSwerveDrivetrain drive) {

        return Commands.run(() -> {

            double distance = drive.getDistanceToHub(
                DriverStation.getAlliance().orElse(Alliance.Blue)
            );

            double targetRPS = SHOOTER_VELOCITY_LOOKUP.get(distance);

            shooter.setTurretSpeed(60);

            boolean spinReady = shooter.atRPS(60);
            SmartDashboard.putBoolean("spinready", spinReady);
            // boolean aimed = drive.isAimedAt(HUB_LOCATION.getTranslation());
            if (spinReady) {
                feeder.setNeoFXVelocity(-100);
            } else {
                feeder.setNeoFXVelocity(0);
            }

        }, shooter, feeder)
        .finallyDo(() -> {
            shooter.setTurretSpeed(0);
            feeder.setNeoFXVelocity(0);
        });
    }
}
