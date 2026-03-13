// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.SubsystemConstants.INDEXER_SHOOT_RPS;
import static frc.robot.constants.SubsystemConstants.INTAKE_SHOOT_RPS;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShooterCommands;

import frc.robot.constants.PRTunerConstants;
import frc.robot.constants.ORTunerConstants;


import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeoFxSubsystem;
import frc.robot.subsystems.TalonFXSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    // private double MaxSpeed = 1.0 * PRTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = 1.0 * ORTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = PRTunerConstants.createDrivetrain();
    public final CommandSwerveDrivetrain drivetrain = ORTunerConstants.createDrivetrain();


    
    private final SendableChooser<Command> autoChooser;

    public final Vision vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPose);

    // private final TurretSubsystem turret = new TurretSubsystem(1, 15); // You can check the IDs of NEO motors by connecting to their CAN with USB C and opening REV Hardware Client
    // private final IntakeSubsystem intake = new IntakeSubsystem(2); // You can check the IDs of NEO motors by connecting to their CAN with USB C and opening REV Hardware Client

    private final IntakeSubsystem intake = new IntakeSubsystem(52);
    private final NeoFxSubsystem indexer = new NeoFxSubsystem(15);
    private final NeoFxSubsystem extender = new NeoFxSubsystem(4);

    // private final IntakeSubsystem intake = new IntakeSubsystem(51); // Change IDs in code
    // private final ConveyorSubsystem conveyor = new ConveyorSubsystem(52, 55); // You can check the IDs of the Kraken motors and change spin direction by connecting to the Robot and opening Phoenix Tuner X
    private final TurretSubsystem turret = new TurretSubsystem(54, 55); // You can check the IDs of the Kraken motors and change spin direction by connecting to the Robot and opening Phoenix Tuner X

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand( //intake named command for pathplanning auto
            "intake",
             new InstantCommand(() ->
              intake.setIntakeSpeedCommand(5))
            );
        NamedCommands.registerCommand( //shooting named command which runs the belt while running the turret
            "shoot",
            ShooterCommands.teleHalfShooterCommand(turret, indexer, drivetrain, joystick::getLeftX, joystick::getLeftY)
            );
        configureBindings();
    }

    private void configureBindings() {


        joystick.y().onTrue(indexer.setNeoFXVelocityCommand(INDEXER_SHOOT_RPS));
        joystick.y().onFalse(indexer.setNeoFXVelocityCommand(0));

        joystick.leftTrigger().onTrue(intake.setIntakeSpeedCommand(INTAKE_SHOOT_RPS)); //intake the stuff
        joystick.leftTrigger().onFalse(intake.setIntakeSpeedCommand(0));

        joystick.leftBumper().onTrue(intake.setIntakeSpeedCommand(-INTAKE_SHOOT_RPS)); //push stuff out of the intake
        joystick.leftBumper().onFalse(intake.setIntakeSpeedCommand(0));



        joystick.rightBumper().whileTrue(ShooterCommands.passCommand(turret, indexer, drivetrain, joystick::getLeftX, joystick::getLeftY));
        joystick.rightTrigger().whileTrue(ShooterCommands.teleHalfShooterCommand(turret, indexer, drivetrain, joystick::getLeftX, joystick::getLeftY));


        // Note that X is defined as forward according to WPILib convention,                                                                                                                                                                                                                                                                                                                ;/
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.povLeft().onTrue(pathFindToAprilTag());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povUp().onTrue(extender.setNeoFXVelocityCommand(20));
        joystick.povUp().onFalse(extender.setNeoFXVelocityCommand(0));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command pathFindToAprilTag() {
        // Defer allows us to calculate the target Pose WHEN the button is pressed
        return Commands.defer(() -> {
            
            int visibleTagID = vision.getTagId();

            // 1. Safety Check: Did we see a tag?
            if (visibleTagID <= 0) {
                // Blink LEDs or print error, don't move
                System.out.println("No AprilTag visible!");
                return Commands.none(); 
            }

            Pose2d tagPose = vision.getTagPose(visibleTagID);
            
            // 2. Safety Check: Is the ID valid in the layout?
            if (tagPose == null) {
                return Commands.none();
            }

            // 3. Create the pathfinding command
            // Inside the Defer block
            Transform2d offset = new Transform2d(new Translation2d(1.0, 0), new Rotation2d(0));
            // This calculates a point 1 meter "out" from the tag's face
            Pose2d safeScoringPose = tagPose.transformBy(offset); 
            
            return drivetrain.pathfindToPose(safeScoringPose);

        }, Set.of(drivetrain)); // Require the drivetrain
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
