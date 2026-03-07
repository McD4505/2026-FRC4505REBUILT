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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX elevatorMotor;
  private final PositionVoltage elevatorPV;
  private final TalonFXConfiguration elevatorConfig;


  public ElevatorSubsystem(int elevatorID) {
    canBus = new CANBus("rio");
    elevatorMotor = new TalonFX(elevatorID, canBus);
    elevatorPV = new PositionVoltage(0).withSlot(0);

    elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.Slot0.kG = 0.35;
    elevatorConfig.Slot0.kS = 0.2; // Add 0.1 V output to overcome static friction
    elevatorConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elevatorConfig.Slot0.kP = 5.0; // An error of 1 rps results in 0.11 V output
    elevatorConfig.Slot0.kI = 0; // no output for integrated error
    elevatorConfig.Slot0.kD = 0.1; // no output for error derivative

    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // elevator limits
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    // enable supply and stator current limit
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorMotor.getConfigurator().apply(elevatorConfig);

    elevatorMotor.setPosition(0);
  }

  public void setElevatorPosition(double Rotations){
    elevatorMotor.setControl(elevatorPV.withPosition(Rotations));
  }

  public Command setElevatorPositionCommand(double Rotations){
    return new InstantCommand(() -> setElevatorPosition(Rotations), this); 
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
