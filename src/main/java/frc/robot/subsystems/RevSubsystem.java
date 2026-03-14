package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.annotation.Target;
import java.time.Period;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class RevSubsystem extends SubsystemBase{
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public RevSubsystem(int motorID){
        motor = new SparkMax(motorID, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        // /*
        // * Create a new SPARK MAX configuration object. This will store the
        // * configuration parameters for the SPARK MAX that we will set below.
        // */
        motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(35);
        motorConfig.closedLoopRampRate(0.2);
        motorConfig.idleMode(IdleMode.kBrake);

        // /*
        // * Configure the encoder. For this specific example, we are using the
        // * integrated encoder of the NEO, and we don't need to configure it. If
        // * needed, we can adjust values like the position or velocity conversion
        // * factors.
        // */
        motorConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        // /*
        // * Configure the closed loop controller. We want to make sure we set the
        // * feedback sensor as the primary encoder.
        // */
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
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

        // /*
        // * Apply the configuration to the SPARK MAX.
        // *
        // * kResetSafeParameters is used to get the SPARK MAX to a known state. This
        // * is useful in case the SPARK MAX is replaced.
        // *
        // * kPersistParameters is used to ensure the configuration is not lost when
        // * the SPARK MAX loses power. This is useful for power cycles that may occur
        // * mid-operation.
        // */
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); //changed from no persist to persist

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
        SmartDashboard.setDefaultNumber("Actual Position", 0);
        SmartDashboard.setDefaultNumber("Actual Velocity", 0);
    }

    public Command setMotorVelocity(double velocity){
        SmartDashboard.putBoolean("Control Mode", true);
        return new InstantCommand(() -> SmartDashboard.putNumber("Target Velocity", velocity), this);
    }

    // public Command setMotorPosition(double position){
    //     SmartDashboard.putBoolean("Control Mode", false);
    //     return new InstantCommand(() -> SmartDashboard.putNumber("Target Position", position), this);
    // }

    public void setMotorPosition(double position){
        closedLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command setMotorPositionCommand(double position){
        return new InstantCommand(() -> setMotorPosition(position), this);
    }

    public Command setMotorPercent(double val) {
        return run(() -> motor.set(val));
    }
    // public void setMotorPercent(double val) {
    //     motor.set(val);
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        // if (SmartDashboard.getBoolean("Control Mode", false)) {
        //     /*
        //     * Get the target velocity from SmartDashboard and set it as the setpoint
        //     * for the closed loop controller.
        //     */
        // double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
        // closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        // } else {
        //     /*
        //     * Get the target position from SmartDashboard and set it as the setpoint
        //     * for the closed loop controller.
        //     */
        //     double targetPosition = SmartDashboard.getNumber("Target Position", 0);
        //     closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // }
    }
}
