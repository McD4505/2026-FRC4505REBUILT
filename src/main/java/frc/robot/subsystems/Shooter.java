// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Shooter extends SubsystemBase {
  
  //Motors
  private final SparkMax shooter;

  public Shooter(int shooterID) {
    this.shooter = new SparkMax(shooterID, MotorType.kBrushless);
  }

  public void setShooterSpeed(double speed){
    shooter.set(speed);
  }

  public Command setShooterCommand(double speed) {
    return new InstantCommand(() -> setShooterSpeed(speed), this);
  }
}
