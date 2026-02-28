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

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX intakeMotor;
  private final VelocityVoltage intakeVV;
  private final DutyCycleOut intakeDC;
  private final TalonFXConfiguration intakeConfig;


  public IntakeSubsystem(int intakeID) {
    canBus = new CANBus("rio");
    intakeMotor = new TalonFX(intakeID, canBus);
    intakeVV = new VelocityVoltage(0).withSlot(0);
    intakeDC = new DutyCycleOut(0);

    intakeConfig = new TalonFXConfiguration();

    intakeConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    intakeConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    intakeConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    intakeConfig.Slot0.kI = 0; // no output for integrated error
    intakeConfig.Slot0.kD = 0; // no output for error derivative

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setPosition(0);
  }

  public void setIntakeSpeed(double RotationsPerSecond){
    intakeMotor.setControl(intakeVV.withVelocity(RotationsPerSecond));
  }

  public Command setIntakeSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setIntakeSpeed(RotationsPerSecond)); 
  }

  @Override
  public void periodic() {
}

  @Override
  public void simulationPeriodic() {
  }
}
