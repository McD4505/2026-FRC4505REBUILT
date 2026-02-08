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

public class TalonFXSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX talonFX;
  private final DutyCycleOut dc;
  private final TalonFXConfiguration talonFXConfig;


  public TalonFXSubsystem(int talonFXID) {
    canBus = new CANBus("subsystem");
    talonFX = new TalonFX(talonFXID, canBus);
    dc = new DutyCycleOut(0);

    talonFXConfig = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    talonFXConfig.Slot0.kP = 1;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 10;
    talonFXConfig.Slot0.kV = 2;

    talonFX.getConfigurator().apply(talonFXConfig);

    talonFX.setPosition(0);
  }

  public void setTalonFXSpeed(double speedProportion){
    talonFX.setControl(dc.withOutput(speedProportion));
  }

  public Command setTalonFXSpeedCommand(double speedProportion){
    return new InstantCommand(() -> setTalonFXSpeed(speedProportion)); 
  }

  @Override
  public void periodic() {
      // SmartDashboard.putNumber("TalonFX Actual Position", talonFX.getPosition());
      // SmartDashboard.putNumber("Intake Actual Velocity", talonFX.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
