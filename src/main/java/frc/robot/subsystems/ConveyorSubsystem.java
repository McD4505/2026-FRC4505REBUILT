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
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX beltMotor;
  private final TalonFX turretFeedMotor;
  private final VelocityVoltage beltVV;
  private final VelocityVoltage turretFeedVV;
  private final DutyCycleOut beltDC;
  private final DutyCycleOut turretFeedDC;
  private final TalonFXConfiguration beltConfig;
  private final TalonFXConfiguration turretFeedConfig;

  private boolean beltOverTemp;
  private boolean turretFeedOverTemp;

  public ConveyorSubsystem(int beltID, int turretFeedID) {
    canBus = new CANBus("rio");
    beltMotor = new TalonFX(beltID, canBus);
    turretFeedMotor = new TalonFX(turretFeedID, canBus);
    beltVV = new VelocityVoltage(0).withSlot(0);
    turretFeedVV = new VelocityVoltage(0).withSlot(0);
    beltDC = new DutyCycleOut(0);
    turretFeedDC = new DutyCycleOut(0);

    beltConfig = new TalonFXConfiguration();

    beltConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    beltConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    beltConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    beltConfig.Slot0.kI = 0; // no output for integrated error
    beltConfig.Slot0.kD = 0; // no output for error derivative

    beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    beltConfig.CurrentLimits.SupplyCurrentLimit = 30;
    beltConfig.CurrentLimits.StatorCurrentLimit = 40;
    beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    beltConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    beltMotor.getConfigurator().apply(beltConfig);

    beltMotor.setPosition(0);

    turretFeedConfig = new TalonFXConfiguration();
    
    turretFeedConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    turretFeedConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    turretFeedConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    turretFeedConfig.Slot0.kI = 0; // no output for integrated error
    turretFeedConfig.Slot0.kD = 0; // no output for error derivative

    turretFeedConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretFeedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    turretFeedConfig.CurrentLimits.SupplyCurrentLimit = 35;
    turretFeedConfig.CurrentLimits.StatorCurrentLimit = 50;
    turretFeedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretFeedConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretFeedMotor.getConfigurator().apply(turretFeedConfig);

    turretFeedMotor.setPosition(0);

    SmartDashboard.setDefaultNumber(this.getName() + " Belt Temperature", 0);
    SmartDashboard.setDefaultNumber(this.getName() + " Turret Feed Temperature", 0);
  }

  public void setBeltSpeed(double RotationsPerSecond){
    if (RotationsPerSecond == 0){
      beltMotor.setControl(new NeutralOut());
      turretFeedMotor.setControl(new NeutralOut());
    } else {
      beltMotor.setControl(beltVV.withVelocity(RotationsPerSecond));
      turretFeedMotor.setControl(turretFeedVV.withVelocity(RotationsPerSecond));
    }
  }

  public void setBeltSpeed(double beltRotationsPerSecond, double turretFeedRotationsPerSecond){
    if (beltRotationsPerSecond == 0){
      beltMotor.setControl(new NeutralOut());
    } else {
      beltMotor.setControl(beltVV.withVelocity(beltRotationsPerSecond));
    }
    if (turretFeedRotationsPerSecond == 0){
      turretFeedMotor.setControl(new NeutralOut());
    } else {
      turretFeedMotor.setControl(turretFeedVV.withVelocity(turretFeedRotationsPerSecond));
    }
  }

  public Command setBeltSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setBeltSpeed(RotationsPerSecond), this); 
  }

  public Command setBeltSpeedCommand(double beltRotationsPerSecond, double turretFeedRotationsPerSecond){
    return new InstantCommand(() -> setBeltSpeed(beltRotationsPerSecond, turretFeedRotationsPerSecond), this); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(this.getName() + " Belt Temperature", beltMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(this.getName() + " Turret Feed Temperature", turretFeedMotor.getDeviceTemp().getValueAsDouble());

    double beltTemp = beltMotor.getDeviceTemp().getValueAsDouble();
    if (beltTemp > 75) beltOverTemp = true;
    if (beltTemp < 65) beltOverTemp = false;
    
    if (beltOverTemp){
      beltMotor.setControl(new NeutralOut());
    }

    double turretFeedTemp = turretFeedMotor.getDeviceTemp().getValueAsDouble();
    if (turretFeedTemp > 75) turretFeedOverTemp = true;
    if (turretFeedTemp < 65) turretFeedOverTemp = false;

    if (turretFeedOverTemp){
      turretFeedMotor.setControl(new NeutralOut());
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}