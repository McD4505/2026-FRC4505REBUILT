// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFXSubsystem extends SubsystemBase {
  
  private final CANBus canBus;
  private final TalonFX talonFX;
  private final VelocityVoltage vv;
  private final DutyCycleOut dc;
  private final TalonFXConfiguration talonFXConfig;

  private boolean talonFXOverTemp;

  public TalonFXSubsystem(int talonFXID) {
    canBus = new CANBus("rio");
    talonFX = new TalonFX(talonFXID, canBus);
    vv = new VelocityVoltage(0).withSlot(0);
    dc = new DutyCycleOut(0);

    talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    talonFXConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    talonFXConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    talonFXConfig.Slot0.kI = 0; // no output for integrated error
    talonFXConfig.Slot0.kD = 0; // no output for error derivative

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable supply and stator current limit
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFX.getConfigurator().apply(talonFXConfig);

    talonFX.setPosition(0);

    SmartDashboard.setDefaultNumber("Device ID: " + talonFX.getDeviceID() + " Actual RPS", 0);
    SmartDashboard.setDefaultNumber("Device ID: " + talonFX.getDeviceID() + " Target RPS", 0);
    SmartDashboard.setDefaultBoolean("Stop Device ID: " + talonFX.getDeviceID(), false);

    SmartDashboard.setDefaultNumber("Device ID: " + talonFX.getDeviceID() + " Temperature C", 0);
  }

  public void setTalonFXSpeed(double RotationsPerSecond){
    if (RotationsPerSecond == 0){
      talonFX.setControl(new NeutralOut());
    } else {
      talonFX.setControl(vv.withVelocity(RotationsPerSecond));
    }
  }

  public Command setTalonFXSpeedCommand(double RotationsPerSecond){
    return new InstantCommand(() -> setTalonFXSpeed(RotationsPerSecond)); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Device ID: " + talonFX.getDeviceID() + " Actual RPS", talonFX.getVelocity().getValue().in(Units.RotationsPerSecond));
    if (SmartDashboard.getBoolean("Stop Device ID: " + talonFX.getDeviceID(), false)) {
      SmartDashboard.putBoolean("Stop Device ID: " + talonFX.getDeviceID(), false);
      SmartDashboard.putNumber("Device ID: " + talonFX.getDeviceID() + " Target RPS", 0);
    }

    double targetRPS = SmartDashboard.getNumber("Device ID: " + talonFX.getDeviceID() + " Target RPS", 0);
    setTalonFXSpeed(targetRPS);

    SmartDashboard.putNumber("Device ID: " + talonFX.getDeviceID() + " Temperature C", talonFX.getDeviceTemp().getValueAsDouble());

    double talonFXTemp = talonFX.getDeviceTemp().getValueAsDouble();
    if (talonFXTemp > 75) talonFXOverTemp = true;
    if (talonFXTemp < 65) talonFXOverTemp = false;

    if (talonFXOverTemp){
      talonFX.setControl(new NeutralOut());
    }
}

  @Override
  public void simulationPeriodic() {
  }
}
