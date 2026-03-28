package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RevSubsystem;
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
        TurretSubsystem shooter,
        IntakeSubsystem indexer,
        CommandSwerveDrivetrain drive,
        DoubleSupplier joystickX,
        DoubleSupplier joystickY
    ) {
        return new ParallelCommandGroup(
            // directly use the command returned by pointTowardsPoint
            drive.pointTowardsPoint(HUB_LOCATION.getTranslation(), joystickX, joystickY),
            setDesiredShootingStates(shooter, indexer, drive, true)
        );
    }

    public static Command setDesiredShootingStates(
        TurretSubsystem shooter,
        IntakeSubsystem indexer,
        CommandSwerveDrivetrain drive,
        boolean isShoot
        ) {


        return Commands.runEnd(

            () -> {
                double distance = 0;
                if (isShoot) {
                    distance = drive.getDistanceToHub(
                    DriverStation.getAlliance().orElse(Alliance.Blue)
                    );
                } else {
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        distance = Math.abs(drive.getPose().getX() - BUMPER_LINE_BLUE) + PASS_OFFSET;
                    } else {
                        distance = Math.abs(drive.getPose().getX() - BUMPER_LINE_RED) + PASS_OFFSET;
                    }
                }

                // Changes
                // double avgVel = 

                double targetRPS = SHOOTER_VELOCITY_LOOKUP.get(distance);

                shooter.setTurretSpeed(targetRPS);
                indexer.setIntakeSpeed(80);

                boolean spinReady = shooter.atRPS(targetRPS-10);
                SmartDashboard.putBoolean("spinready", spinReady);
                SmartDashboard.putNumber("targetRPSVariable" , targetRPS);
                // boolean aimed = drive.isAimedAt(HUB_LOCATION.getTranslation());

                indexer.setIntakeSpeed(INDEXER_SHOOT_RPS);
            },
            () -> {
                shooter.setTurretSpeed(0);
                indexer.setIntakeSpeed(0);
            },
            shooter,
            indexer
        );
    }

    public static Command passCommand(
        TurretSubsystem shooter,
        IntakeSubsystem indexer,
        CommandSwerveDrivetrain drive,
        DoubleSupplier joystickX,
        DoubleSupplier joystickY
        ) {
            Pose2d robotPose = drive.getPose();
            Rotation2d facing = null;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                facing = Rotation2d.fromDegrees(180);
            } else {
                facing = Rotation2d.fromDegrees(0);
            }
            return new ParallelCommandGroup(

                drive.pointTowardsPoint(new Pose2d(robotPose.getX(), robotPose.getY(), facing).getTranslation(), joystickX, joystickY),
                setDesiredShootingStates(shooter, indexer, drive, false)

            );
    }
}
