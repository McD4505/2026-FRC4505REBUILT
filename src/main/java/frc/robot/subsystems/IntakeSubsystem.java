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

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeMotorConfig;
    private final SparkClosedLoopController intakeClosedLoopController;
    private final RelativeEncoder intakeEncoder;
    

    public IntakeSubsystem(int intakeID) {
        intakeMotor = new SparkMax(intakeID, MotorType.kBrushless);
        intakeClosedLoopController = intakeMotor.getClosedLoopController();
        intakeEncoder = this.intakeMotor.getEncoder();

        intakeMotorConfig = new SparkMaxConfig();

        intakeMotorConfig.encoder
            .velocityConversionFactor(1);

        intakeMotorConfig.closedLoop
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

            intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


            SmartDashboard.setDefaultNumber("Intake Target Velocity", 0);
    }

    // Run intake in velocity control (slot 0)
    public void setVelocity(double velocity) {
        intakeClosedLoopController.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Actual Velocity", intakeEncoder.getVelocity());

        double intakeTargetVelocity = SmartDashboard.getNumber("Intake Target Velocity", 0);
        setVelocity(intakeTargetVelocity);
    }

    @Override
    public void simulationPeriodic() {
       
    }

}