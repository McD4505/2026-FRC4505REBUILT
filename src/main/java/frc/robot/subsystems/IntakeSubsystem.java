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
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        intakeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control.
            // We don't need to pass a closed loop slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

            intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


            SmartDashboard.setDefaultNumber("Intake Target Position", 0);
            SmartDashboard.setDefaultNumber("Intake Target Velocity", 0);
            SmartDashboard.setDefaultBoolean("Intake Control Mode", false);
            SmartDashboard.setDefaultBoolean("Reset Intake Encoder", false);

    }

    public void resetIntakeEncoder(){
        intakeEncoder.setPosition(0);
    }

    // Run intake in position control (slot 0)
    public void setPosition(double position) {
        intakeClosedLoopController.setSetpoint(
            position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

    // Run intake in velocity control (slot 1)
    public void setVelocity(double velocity) {
        intakeClosedLoopController.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot1
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Actual Position", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Intake Actual Velocity", intakeEncoder.getVelocity());

        if (SmartDashboard.getBoolean("Intake Control Mode", false)){
            double intakeTargetVelocity = SmartDashboard.getNumber("Intake Target Velocity", 0);
            setVelocity(intakeTargetVelocity);
        } else {
            double intakeTargetPosition = SmartDashboard.getNumber("Intake Target Position", 0);
            setPosition(intakeTargetPosition);
        }

        if (SmartDashboard.getBoolean("Reset Intake Encoder", false)) {
            SmartDashboard.putBoolean("Reset Intake Encoder", false);
            resetIntakeEncoder();
        }
    }

    @Override
    public void simulationPeriodic() {
       
    }

}