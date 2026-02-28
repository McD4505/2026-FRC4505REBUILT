// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX turretFeedMotor;
  private final TalonFX topTurretMotor;
  private final TalonFX bottomTurretMotor;
  private final VelocityVoltage turretFeedVV;
  private final VelocityVoltage topTurretVV;
  private final VelocityVoltage bottomTurretVV;
  private final DutyCycleOut turretFeedDC;
  private final DutyCycleOut topTurretDC;
  private final DutyCycleOut bottomTurretDC;
  private final TalonFXConfiguration turretFeedConfig;
  private final TalonFXConfiguration topTurretConfig;
  private final TalonFXConfiguration bottomTurretConfig;

  public TurretSubsystem(int turretFeedID, int topTurretID, int bottomTurretID) {
    canBus = new CANBus("rio");
    turretFeedMotor = new TalonFX(turretFeedID, canBus);
    topTurretMotor = new TalonFX(topTurretID, canBus);
    bottomTurretMotor = new TalonFX(bottomTurretID, canBus);
    turretFeedVV = new VelocityVoltage(0).withSlot(0);
    topTurretVV = new VelocityVoltage(0).withSlot(0);
    bottomTurretVV = new VelocityVoltage(0).withSlot(0);
    turretFeedDC = new DutyCycleOut(0);
    topTurretDC = new DutyCycleOut(0);
    bottomTurretDC = new DutyCycleOut(0);

    turretFeedConfig = new TalonFXConfiguration();
    
    turretFeedConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    turretFeedConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    turretFeedConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    turretFeedConfig.Slot0.kI = 0; // no output for integrated error
    turretFeedConfig.Slot0.kD = 0; // no output for error derivative

    turretFeedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretFeedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    turretFeedMotor.getConfigurator().apply(turretFeedConfig);

    turretFeedMotor.setPosition(0);

    topTurretConfig = new TalonFXConfiguration();
    
    topTurretConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    topTurretConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    topTurretConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    topTurretConfig.Slot0.kI = 0; // no output for integrated error
    topTurretConfig.Slot0.kD = 0; // no output for error derivative

    topTurretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topTurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topTurretMotor.getConfigurator().apply(topTurretConfig);

    topTurretMotor.setPosition(0);

    bottomTurretConfig = new TalonFXConfiguration();

    bottomTurretConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    bottomTurretConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    bottomTurretConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    bottomTurretConfig.Slot0.kI = 0; // no output for integrated error
    bottomTurretConfig.Slot0.kD = 0; // no output for error derivative

    bottomTurretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomTurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    bottomTurretMotor.getConfigurator().apply(bottomTurretConfig);

    bottomTurretMotor.setPosition(0);
  }

  public void setTurretSpeed(double RotationsPerSecond){
    turretFeedMotor.setControl(turretFeedVV.withVelocity(RotationsPerSecond));
    topTurretMotor.setControl(topTurretVV.withVelocity(RotationsPerSecond)); // Change Spin Directions in Phoenix Tuner X
    bottomTurretMotor.setControl(bottomTurretVV.withVelocity(RotationsPerSecond));
  }

  public void setTurretSpeed(double turretFeedRotationsPerSecond, double topTurretRotationsPerSecond, double bottomTurretRotationsPerSecond){
    turretFeedMotor.setControl(turretFeedVV.withVelocity(turretFeedRotationsPerSecond));
    topTurretMotor.setControl(topTurretVV.withVelocity(topTurretRotationsPerSecond)); // Change Spin Directions in Phoenix Tuner X
    bottomTurretMotor.setControl(bottomTurretVV.withVelocity(bottomTurretRotationsPerSecond));
  }

  public Command setTurretSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setTurretSpeed(RotationsPerSecond)); 
  }

  public Command setTurretSpeedCommand(double turretFeedRotationsPerSecond, double topTurretRotationsPerSecond, double bottomTurretRotationsPerSecondn){
    return new InstantCommand(() -> setTurretSpeed(turretFeedRotationsPerSecond, topTurretRotationsPerSecond, bottomTurretRotationsPerSecondn)); 
  }

  @Override
  public void periodic() {
}

  @Override
  public void simulationPeriodic() {
  }
}
