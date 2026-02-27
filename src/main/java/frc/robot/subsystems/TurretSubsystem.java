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

public class TurretSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX turretFeedMotor;
  private final TalonFX topTurretMotor;
  private final TalonFX bottomTurretMotor;
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
    turretFeedDC = new DutyCycleOut(0);
    topTurretDC = new DutyCycleOut(0);
    bottomTurretDC = new DutyCycleOut(0);

    turretFeedConfig = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    turretFeedConfig.Slot0.kP = 1;
    turretFeedConfig.Slot0.kI = 0;
    turretFeedConfig.Slot0.kD = 10;
    turretFeedConfig.Slot0.kV = 2;

    turretFeedMotor.getConfigurator().apply(turretFeedConfig);

    turretFeedMotor.setPosition(0);

    topTurretConfig = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    topTurretConfig.Slot0.kP = 1;
    topTurretConfig.Slot0.kI = 0;
    topTurretConfig.Slot0.kD = 10;
    topTurretConfig.Slot0.kV = 2;

    topTurretMotor.getConfigurator().apply(topTurretConfig);

    topTurretMotor.setPosition(0);

    bottomTurretConfig = new TalonFXConfiguration();
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    bottomTurretConfig.Slot0.kP = 1;
    bottomTurretConfig.Slot0.kI = 0;
    bottomTurretConfig.Slot0.kD = 10;
    bottomTurretConfig.Slot0.kV = 2;

    bottomTurretMotor.getConfigurator().apply(bottomTurretConfig);

    bottomTurretMotor.setPosition(0);
  }

  public void setTurretSpeed(double speedProportion){
    turretFeedMotor.setControl(turretFeedDC.withOutput(speedProportion));
    topTurretMotor.setControl(topTurretDC.withOutput(speedProportion)); // Change Spin Directions in Phoenix Tuner X
    bottomTurretMotor.setControl(bottomTurretDC.withOutput(speedProportion));
  }

  public void setTurretSpeed(double turretFeedSpeedProportion, double topTurretSpeedProportion, double bottomTurretSpeedProportion){
    turretFeedMotor.setControl(turretFeedDC.withOutput(turretFeedSpeedProportion));
    topTurretMotor.setControl(topTurretDC.withOutput(topTurretSpeedProportion)); // Change Spin Directions in Phoenix Tuner X
    bottomTurretMotor.setControl(bottomTurretDC.withOutput(bottomTurretSpeedProportion));
  }

  public Command setTurretSpeedCommand(double speedProportion){
    return new InstantCommand(() -> setTurretSpeed(speedProportion)); 
  }

  @Override
  public void periodic() {
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
