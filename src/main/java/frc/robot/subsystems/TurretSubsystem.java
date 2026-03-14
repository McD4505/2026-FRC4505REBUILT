// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.opencv.dnn.TextDetectionModel_DB;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX topTurretMotor;
  private final TalonFX bottomTurretMotor;
  private final VelocityVoltage topTurretVV;
  private final VelocityVoltage bottomTurretVV;
  private final DutyCycleOut topTurretDC;
  private final DutyCycleOut bottomTurretDC;
  private final TalonFXConfiguration topTurretConfig;
  private final TalonFXConfiguration bottomTurretConfig;

  private boolean topTurretOverTemp;
  private boolean bottomTurretOverTemp;

  public TurretSubsystem(int topTurretID, int bottomTurretID) {
    canBus = new CANBus("rio");
    topTurretMotor = new TalonFX(topTurretID, canBus);
    bottomTurretMotor = new TalonFX(bottomTurretID, canBus);
    topTurretVV = new VelocityVoltage(0).withSlot(0);
    bottomTurretVV = new VelocityVoltage(0).withSlot(0);
    topTurretDC = new DutyCycleOut(0);
    bottomTurretDC = new DutyCycleOut(0);

    topTurretConfig = new TalonFXConfiguration();
    
    topTurretConfig.Slot0.kS = 0.04; // Add 0.04 V output to overcome static friction
    topTurretConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    topTurretConfig.Slot0.kP = 0.18; // An error of 1 rps results in 0.18 V output
    topTurretConfig.Slot0.kI = 0; // no output for integrated error
    topTurretConfig.Slot0.kD = 0; // no output for error derivative

    topTurretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topTurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    topTurretConfig.CurrentLimits.SupplyCurrentLimit = 50;
    topTurretConfig.CurrentLimits.StatorCurrentLimit = 60;
    topTurretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topTurretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    topTurretMotor.getConfigurator().apply(topTurretConfig);

    topTurretMotor.setPosition(0);

    bottomTurretConfig = new TalonFXConfiguration();

    bottomTurretConfig.Slot0.kS = 0.04; // Add 0.1 V output to overcome static friction
    bottomTurretConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    bottomTurretConfig.Slot0.kP = 0.18; // An error of 1 rps results in 0.11 V output
    bottomTurretConfig.Slot0.kI = 0; // no output for integrated error
    bottomTurretConfig.Slot0.kD = 0; // no output for error derivative

    bottomTurretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomTurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    bottomTurretConfig.CurrentLimits.SupplyCurrentLimit = 50;
    bottomTurretConfig.CurrentLimits.StatorCurrentLimit = 80;
    bottomTurretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomTurretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    bottomTurretMotor.getConfigurator().apply(bottomTurretConfig);

    bottomTurretMotor.setPosition(0);

    SmartDashboard.setDefaultNumber(this.getName() + " Top Turret Target RPS", 0);
    SmartDashboard.setDefaultNumber(this.getName() + " Bottom Turret Target RPS", 0);
    SmartDashboard.setDefaultBoolean(this.getName() + " Control Mode", false);
    
    SmartDashboard.setDefaultNumber(this.getName() + " Top Turret Temperature", 0);
    SmartDashboard.setDefaultNumber(this.getName() + " Bottom Turret Temperature", 0);
  }

  public void setTurretSpeed(double RotationsPerSecond){
    if (RotationsPerSecond == 0){
      topTurretMotor.setControl(new NeutralOut());
      bottomTurretMotor.setControl(new NeutralOut());
    } else {
      topTurretMotor.setControl(topTurretVV.withVelocity(RotationsPerSecond)); // Change Spin Directions in Phoenix Tuner X
      bottomTurretMotor.setControl(bottomTurretVV.withVelocity(RotationsPerSecond));
    }
  }

  public void setTurretSpeed(double topTurretRotationsPerSecond, double bottomTurretRotationsPerSecond){
    if (topTurretRotationsPerSecond == 0){
      topTurretMotor.setControl(new NeutralOut());
    } else {
      topTurretMotor.setControl(topTurretVV.withVelocity(topTurretRotationsPerSecond)); // Change Spin Directions in Phoenix Tuner X
    }
    if (bottomTurretRotationsPerSecond == 0){ 
      bottomTurretMotor.setControl(new NeutralOut());      
    } else {
      bottomTurretMotor.setControl(bottomTurretVV.withVelocity(bottomTurretRotationsPerSecond));
    } 
  }

  public Command setTurretSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setTurretSpeed(RotationsPerSecond), this); 
  }

  public Command setTurretSpeedCommand(double topTurretRotationsPerSecond, double bottomTurretRotationsPerSecond){
    return new InstantCommand(() -> setTurretSpeed(topTurretRotationsPerSecond, bottomTurretRotationsPerSecond)); 
  }
  public double getRPS(TalonFX motor) {
      return motor.getVelocity().getValueAsDouble();
  }
  public boolean atRPS(double rpm) {
    double currTopRPM = getRPS(topTurretMotor);
    double currBottomRPM = getRPS(bottomTurretMotor);
    if (currTopRPM >= rpm && currBottomRPM >= rpm){
      return true;
    }
    return false;
  }

  public void score(){
    // Will need robot relative field velocity
    // Will get these later
    double x_robotVelocity = 0;
    double z_robotVelocity = 0;
    Pose3d targetPose3d = new Pose3d(0, 0, 0, Rotation3d.kZero);
    Pose3d turretPose3d = new Pose3d(0, 0, 0, Rotation3d.kZero);

    //Implementing without drag or spin
    double g = 9.8; // meters per second squrared
    double wheelRadius = 0.0508; // meters
    double e_velocity = (topTurretMotor.getVelocity().getValue().in(Units.RotationsPerSecond)*wheelRadius + bottomTurretMotor.getVelocity().getValue().in(Units.RotationsPerSecond)*wheelRadius)*0.5;
    double e_theta = Math.PI/6; // Need to fix

    double x_velocity = x_robotVelocity + e_velocity * Math.cos(e_theta); // assuming +x is distance forward
    double y_velocity = e_velocity * Math.sin(e_theta); // assuming +y is distance up
    double z_velocity = z_robotVelocity; // assuming +z is distance right

    double t_x = (targetPose3d.getX() - turretPose3d.getX())/x_velocity;
    double t_y = (y_velocity + (Math.pow(y_velocity, 2) - 2 * g * targetPose3d.getY() - turretPose3d.getY()))/-1 * g;
    double t_z = (targetPose3d.getZ() - turretPose3d.getZ())/z_velocity;

    
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Top Turret Stator Current",
        topTurretMotor.getStatorCurrent().getValueAsDouble());

      SmartDashboard.putNumber("Top Turret Supply Current",
        topTurretMotor.getSupplyCurrent().getValueAsDouble());

      SmartDashboard.putNumber("Bottom Turret Stator Current",
          bottomTurretMotor.getStatorCurrent().getValueAsDouble());

      SmartDashboard.putNumber("Bottom Turret Supply Current",
          bottomTurretMotor.getSupplyCurrent().getValueAsDouble());

      SmartDashboard.putNumber(this.getName() + " Top Turret Actual RPS", getRPS(topTurretMotor));
      SmartDashboard.putNumber(this.getName() + " Bottom Turret Actual RPS", getRPS(bottomTurretMotor));
      if (SmartDashboard.getBoolean(this.getName() + " Control Mode", false)){
        setTurretSpeed(SmartDashboard.getNumber(this.getName() + " Top Turret Target RPS", 0), SmartDashboard.getNumber(this.getName() + " Bottom Turret Target RPS", 0));
      }

      SmartDashboard.putNumber(this.getName() + " Top Turret Temperature", topTurretMotor.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber(this.getName() + " Bottom Turret Temperature", bottomTurretMotor.getDeviceTemp().getValueAsDouble());

      double topTurretTemp = topTurretMotor.getDeviceTemp().getValueAsDouble();
      if (topTurretTemp > 75) topTurretOverTemp = true;
      if (topTurretTemp < 65) topTurretOverTemp = false;

      // if (topTurretOverTemp){
      //   topTurretMotor.setControl(new NeutralOut());
      // }

      double bottomTurretTemp = bottomTurretMotor.getDeviceTemp().getValueAsDouble();
      if (bottomTurretTemp > 75) bottomTurretOverTemp = true;
      if (bottomTurretTemp < 65) bottomTurretOverTemp = false;

      // if (bottomTurretOverTemp){
      //   bottomTurretMotor.setControl(new NeutralOut());
      // }

      if (topTurretOverTemp || bottomTurretOverTemp){
        topTurretMotor.setControl(new NeutralOut());
        bottomTurretMotor.setControl(new NeutralOut());
      }
  }

  @Override
  public void simulationPeriodic() {
  }
}
