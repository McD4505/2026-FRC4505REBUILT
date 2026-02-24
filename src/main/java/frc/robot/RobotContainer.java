// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    // Shooter shooter = new Shooter(0, 0);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Vision vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPose);

    private final SendableChooser<Command> autoChooser;

    public final Boolean isTestingShooter = false;
    public Shooter shooter;
    public final Boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    public RobotContainer() {
                // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
  
        // shooter = new Shooter(15, 1);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        if (isTestingShooter) {

        } else {
            drivetrain.setDefaultCommand(
              // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() ->
                  drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              )
          );

          // Idle while the robot is disabled. This ensures the configured
          // neutral mode is applied to the drive motors while disabled.
          final var idle = new SwerveRequest.Idle();
          RobotModeTriggers.disabled().whileTrue(
              drivetrain.applyRequest(() -> idle).ignoringDisable(true)
          );
          joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
          joystick.b().whileTrue(drivetrain.applyRequest(() ->
              point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
          ));
          joystick.povUp().onTrue(followPathCommand());
          joystick.povDown().onTrue(pathFindToAprilTag());

          // Run SysId routines when holding back/start and X/Y.
          // Note that each routine should be run exactly once in a single log.
          joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
          joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
          joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
          joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

          // Reset the field-centric heading on left bumper press.
          joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

          drivetrain.registerTelemetry(logger::telemeterize);
        }
    }
private Command pathFindToAprilTag() {
    // Defer allows us to calculate the target Pose WHEN the button is pressed
    return Commands.defer(() -> {
        
        int visibleTagID = vision.getTagId(isRed);

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
    public Command followPathCommand() {
      try{
          // Load the path you want to follow using its name in the GUI
          PathPlannerPath path = PathPlannerPath.fromPathFile("go to neutral zone");
          // Create a path following command using AutoBuilder. This will also trigger event markers.
          return AutoBuilder.followPath(path);
      } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
          return Commands.none();
      }
    }
}
