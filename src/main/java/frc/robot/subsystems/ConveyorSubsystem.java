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

public class ConveyorSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX beltMotor;
  private final VelocityVoltage beltVV;
  private final DutyCycleOut beltDC;
  private final TalonFXConfiguration beltConfig;


  public ConveyorSubsystem(int beltID) {
    canBus = new CANBus("rio");
    beltMotor = new TalonFX(beltID, canBus);
    beltVV = new VelocityVoltage(0).withSlot(0);
    beltDC = new DutyCycleOut(0);

    beltConfig = new TalonFXConfiguration();

    beltConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    beltConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    beltConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    beltConfig.Slot0.kI = 0; // no output for integrated error
    beltConfig.Slot0.kD = 0; // no output for error derivative

    beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    beltMotor.getConfigurator().apply(beltConfig);

    beltMotor.setPosition(0);
  }

  public void setBeltSpeed(double RotationsPerSecond){
    beltMotor.setControl(beltVV.withVelocity(RotationsPerSecond));
  }

  public Command setBeltSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setBeltSpeed(RotationsPerSecond)); 
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
