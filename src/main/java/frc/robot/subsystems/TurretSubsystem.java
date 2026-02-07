package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax shooter;

    public TurretSubsystem(int shooterID) {
        this.shooter = new SparkMax(shooterID, MotorType.kBrushless);
    }

    public void setShooterSpeed(double speed) {
        shooter.set(speed);
    }
    public Command setShooterCommand(double speed) {
        return new InstantCommand(() -> setShooterSpeed(speed), this);
    }
}