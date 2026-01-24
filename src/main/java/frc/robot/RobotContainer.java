// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.VisNew;
import frc.robot.subsystems.*;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Vision vision = new Vision((pose, timestamp, stdDevs) -> {});
  // private Shooter shooter = new Shooter(1);

    // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
      // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  //swerve input stream for controlling robot swervedrive 
   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> controller.getLeftY() * -1,  
                                                                () -> controller.getLeftX() * -1)
                                                            .withControllerRotationAxis(controller::getRightX) //right joystick
                                                            .deadband(OperatorConstants.DEADBAND) //deadzone
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true); //robot movement aligns with field orientation
  //makes sure heading is controlled with stick then angular velocity
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controller::getRightX, //know where its turning
                                                                                             controller::getRightY)
                                                           .headingWhile(true);
  //copys heading making sure head is robot relative
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
                                                             SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -controller.getLeftY(),
                                                                        () -> -controller.getLeftX())
                                                                    .withControllerRotationAxis(() -> controller.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                 
                                                                   .allianceRelativeControl(true);
  // Derive the heading axis with math
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  controller.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  controller.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //autoChooser USE THIS FOR AUTO COMMANDS
    autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance).andThen(Commands.none()));
        //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                                                .andThen(drivebase.driveForward().withTimeout(1)));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null ) {
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //sets commands as variables to use
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    if (RobotBase.isSimulation()) { //sets default commands
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard); //heading axis derive in sim
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); //use ang velocity with irl
    }

     if (Robot.isSimulation())
    {//if in sim
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90)); //sets where target pose is at
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target, 
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      controller.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      controller.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      controller.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      controller.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly()); //x is locking drivebase?
      controller.start().onTrue((Commands.runOnce(drivebase::zeroGyro))); //zeros gyro when controller starts
      controller.back().whileTrue(drivebase.centerModulesCommand());
      controller.leftBumper().onTrue(Commands.none());
      controller.rightBumper().onTrue(Commands.none());
    } else
    {
      controller.a().onTrue((Commands.runOnce(drivebase::zeroGyro))); //a is zero gyro
      controller.start().whileTrue(Commands.none()); //
      controller.back().whileTrue(Commands.none());
      controller.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly()); //left bumper locks drivebase
      controller.rightBumper().onTrue(Commands.none());
    }


    // controller.a().onTrue(shooter.setShooterCommand(1.0));
    // controller.a().onFalse(shooter.setShooterCommand(0));

    // controller.b().onTrue(shooter.setShooterCommand(-1.0));
    // controller.b().onFalse(shooter.setShooterCommand(0));
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
    public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }
  // Method to set motor brake mode
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
  
}
