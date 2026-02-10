package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor1;
    private final SparkMaxConfig turretMotorConfig;
    private final SparkClosedLoopController turretClosedLoopController1;
    private final RelativeEncoder turretEncoder1;

    private final SparkMax turretMotor2;
    private final SparkClosedLoopController turretClosedLoopController2;
    private final RelativeEncoder turretEncoder2;

    private final double gearRatio = 1.00; //Don't forget to change Elastic to New Max and Min Speeds (5767.0 / gearRatio)

    public TurretSubsystem(int turretID1, int turretID2) {
        turretMotor1 = new SparkMax(turretID1, MotorType.kBrushless);
        turretClosedLoopController1 = turretMotor1.getClosedLoopController();
        turretEncoder1 = turretMotor1.getEncoder();

        turretMotor2 = new SparkMax(turretID2, MotorType.kBrushless);
        turretClosedLoopController2 = turretMotor2.getClosedLoopController();
        turretEncoder2 = turretMotor2.getEncoder();

        turretMotorConfig = new SparkMaxConfig();

        turretMotorConfig.encoder
            .velocityConversionFactor(1.0 / gearRatio);

        turretMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control.
            // We don't need to pass a closed loop slot, as it will default to slot 0.
            .p(0.0001)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 5767);

            turretMotor1.configure(
                turretMotorConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );

            turretMotor2.configure(
                turretMotorConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );

            SmartDashboard.setDefaultNumber("Turret 1 Target Velocity", 0);

            SmartDashboard.setDefaultNumber("Turret 2 Target Velocity", 0);

            SmartDashboard.setDefaultBoolean("STOP", false);
    }

    // Run intake in velocity control (slot 0)
    public void setVelocity1(double velocity1) {
        turretClosedLoopController1.setSetpoint(
            velocity1,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0
        );
    }

    public void setVelocity2(double velocity2) {
        turretClosedLoopController2.setSetpoint(
            velocity2,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret 1 Actual Velocity", turretEncoder1.getVelocity());
        SmartDashboard.putNumber("Turret 2 Actual Velocity", turretEncoder2.getVelocity());

        if (SmartDashboard.getBoolean("STOP", false)){
            SmartDashboard.putBoolean("STOP", false);
            SmartDashboard.putNumber("Turret 1 Target Velocity", 0);
            SmartDashboard.putNumber("Turret 2 Target Velocity", 0);
        }

        double turretTargetVelocity1 = SmartDashboard.getNumber("Turret 1 Target Velocity", 0);
        setVelocity1(turretTargetVelocity1);
        
        double turretTargetVelocity2 = SmartDashboard.getNumber("Turret 2 Target Velocity", 0);
        setVelocity2(turretTargetVelocity2);
    }

    @Override
    public void simulationPeriodic() {
       
    }

}