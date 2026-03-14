package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoFxSubsystem extends SubsystemBase {

    private final SparkMax neoFXMotor;
    private final SparkMaxConfig neoFXConfig;
    private final SparkClosedLoopController neoFXClosedLoopController;
    private final RelativeEncoder neoFXRelativeEncoder;

    private final double gearRatio = 3.00; //Don't forget to change Elastic to New Max and Min Speeds (5767.0 / gearRatio)

    public NeoFxSubsystem(int neoFXID) {
        neoFXMotor = new SparkMax(neoFXID, MotorType.kBrushless);
        neoFXClosedLoopController = neoFXMotor.getClosedLoopController();
        neoFXRelativeEncoder = neoFXMotor.getEncoder();
        


        neoFXConfig = new SparkMaxConfig();

        
        neoFXConfig.smartCurrentLimit(35);
        neoFXConfig.closedLoopRampRate(0.2);

        neoFXConfig.inverted(true);

        neoFXConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        neoFXConfig.closedLoop
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
          
        neoFXMotor.configure(
            neoFXConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );


            SmartDashboard.setDefaultNumber(this.getName() + " Actual Speed", 0);
            SmartDashboard.setDefaultNumber(this.getName() + " Target Speed", 0);
    }

    public void setNeoFXPosition(double position) {
        neoFXClosedLoopController.setSetpoint(
            position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }


    
    public void setNeoFXVelocity(double velocity) {
        neoFXClosedLoopController.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot1
        );
    }

    public Command setNeoFXPositionCommand(double position) {
        return new InstantCommand( () -> setNeoFXPosition(position), this);
    }

    public Command setNeoFXVelocityCommand(double velocity) {
        return new InstantCommand(() -> SmartDashboard.putNumber(this.getName() + " Target Speed", velocity), this);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber(this.getName() + " Actual Speed", neoFXRelativeEncoder.getVelocity());
        SmartDashboard.putNumber(this.getName() + " Output", neoFXMotor.getAppliedOutput());

        setNeoFXVelocity(SmartDashboard.getNumber(this.getName() + " Target Speed", 0));
    }
}