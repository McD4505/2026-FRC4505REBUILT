package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax shooter_1;
    private final SparkMax shooter_2;

    public Shooter(int shooterID_1, int shooterID_2) {
        this.shooter_1 = new SparkMax(shooterID_1, MotorType.kBrushless);
        this.shooter_2 = new SparkMax(shooterID_1, MotorType.kBrushless);
    }

    public void setShooterSpeed(double speed) {
        shooter_1.set(speed);
        shooter_2.set(speed);
    }
    public Command setShooterCommand(double speed) {
        return new InstantCommand(() -> setShooterSpeed(speed), this);
    }
}
