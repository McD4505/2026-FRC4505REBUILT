// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX intakeMotor;
  private final VelocityVoltage intakeVV;
  private final DutyCycleOut intakeDC;
  private final TalonFXConfiguration intakeConfig;
  private final StatusSignal<Temperature> intakeTempSignal;

  private boolean intakeOverTemp;

  public IntakeSubsystem(int intakeID, boolean inverted) {
    canBus = new CANBus("rio");
    intakeMotor = new TalonFX(intakeID, canBus);
    intakeTempSignal = intakeMotor.getDeviceTemp();
    intakeTempSignal.setUpdateFrequency(10);


    intakeVV = new VelocityVoltage(0).withSlot(0);
    intakeDC = new DutyCycleOut(0);

    intakeConfig = new TalonFXConfiguration();

    intakeConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    intakeConfig.Slot0.kV = 0.06; // A velocity target of 1 rps results in 0.12 V output
    intakeConfig.Slot0.kP = 0.05; // An error of 1 rps results in 0.11 V output
    intakeConfig.Slot0.kI = 0; // no output for integrated error
    intakeConfig.Slot0.kD = 0; // no output for error derivative
    if (inverted) {
      intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 25;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 30;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setPosition(0);

    SmartDashboard.setDefaultNumber("Device ID: " + intakeMotor.getDeviceID() + " Target RPS", 0);
    SmartDashboard.setDefaultNumber("Device ID: " + intakeMotor.getDeviceID() + " Actual RPS", 0);
    SmartDashboard.setDefaultBoolean("Device ID: " + intakeMotor.getDeviceID() + " Control Mode", false);
    
    SmartDashboard.setDefaultNumber("Device ID: " + intakeMotor.getDeviceID() +  " Temperature C", 0);
  }

  public void setIntakeSpeed(double RotationsPerSecond){
    if (RotationsPerSecond == 0){
      intakeMotor.setControl(new NeutralOut());
    } else {
      intakeMotor.setControl(intakeVV.withVelocity(RotationsPerSecond));
    }
  }

  public Command setIntakeSpeedCommand(double speed) {
      return Commands.startEnd(
          () -> setIntakeSpeed(speed),
          () -> setIntakeSpeed(0),
          this
      );
  }
  @Override
  public void periodic() {
    intakeTempSignal.refresh();
    double intakeTemp = intakeTempSignal.getValueAsDouble();
    SmartDashboard.putNumber("Device ID: " + intakeMotor.getDeviceID() +  " Temperature C", intakeMotor.getDeviceTemp().getValueAsDouble());

    if (intakeTemp > 75) intakeOverTemp = true;
    if (intakeTemp < 65) intakeOverTemp = false;

    if (intakeOverTemp){
      intakeMotor.setControl(new NeutralOut());
    }
  } 
}
