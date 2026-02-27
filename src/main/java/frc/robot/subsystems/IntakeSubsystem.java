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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX intakeMotor;
  private final DutyCycleOut intakeDC;
  private final TalonFXConfiguration intakeConfig;


  public IntakeSubsystem(int intakeID) {
    canBus = new CANBus("rio");
    intakeMotor = new TalonFX(intakeID, canBus);
    intakeDC = new DutyCycleOut(0);

    intakeConfig = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    intakeConfig.Slot0.kP = 1;
    intakeConfig.Slot0.kI = 0;
    intakeConfig.Slot0.kD = 10;
    intakeConfig.Slot0.kV = 2;

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setPosition(0);
  }

  public void setIntakeSpeed(double speedProportion){
    intakeMotor.setControl(intakeDC.withOutput(speedProportion));
  }

  public Command setTalonFXSpeedCommand(double speedProportion){
    return new InstantCommand(() -> setIntakeSpeed(speedProportion)); 
  }

  @Override
  public void periodic() {
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
