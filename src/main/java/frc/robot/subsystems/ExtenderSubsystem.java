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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {

    private final SparkMax neoFXMotor;
    private final SparkMaxConfig neoFXConfig;
    private final SparkClosedLoopController neoFXClosedLoopController;
    private final RelativeEncoder neoFXRelativeEncoder;


    public ExtenderSubsystem(int neoFXID) {
        neoFXMotor = new SparkMax(neoFXID, MotorType.kBrushless);
        neoFXClosedLoopController = neoFXMotor.getClosedLoopController();
        neoFXRelativeEncoder = neoFXMotor.getEncoder();

        neoFXConfig = new SparkMaxConfig();

        neoFXConfig.inverted(true);

        neoFXConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        neoFXConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.002, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);   // correct for 3:1
            neoFXConfig.closedLoop.maxMotion
            .cruiseVelocity(1)     // units per second
            .maxAcceleration(0.5) // units per second^2
            .allowedProfileError(0.02);
                
        neoFXMotor.configure(
            neoFXConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void setNeoFXPosition(double position) {
        neoFXClosedLoopController.setSetpoint(
            position,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot1
        );
    }



    public Command setNeoFXPositionCommand(double position) {
        return new InstantCommand( () -> setNeoFXPosition(position), this);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("EXTENDER CURRENT POSITION", neoFXRelativeEncoder.getPosition());
    }
}