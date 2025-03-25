package frc.robot.subsystems;
    
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Armature extends SubsystemBase {
    
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController motorClosedLoopEncoder;
    private RelativeEncoder motorRelEncoder;

    public Armature() {
        motor = new SparkMax(Constants.ARMATURE_MOTOR_CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorClosedLoopEncoder = motor.getClosedLoopController();
        motorRelEncoder = motor.getEncoder();

        motorConfig.encoder
            .positionConversionFactor(1.0);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1)
            .i(0)
            .d(0.1)
            .outputRange(-1, 1);
        
        // Max Velocity for Neo Motor is 5676 RPM
        motorConfig.closedLoop.maxMotion
            .maxVelocity(2000)
            .maxAcceleration(2000)
            .allowedClosedLoopError(0.5);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void gotoPosition(double position) {
        motorClosedLoopEncoder.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public void resetEncoderPosition() {
        motorRelEncoder.setPosition(0);
    }

    public void showData() {
        SmartDashboard.putNumber("Armature Actual Position", motorRelEncoder.getPosition());
        SmartDashboard.putNumber("Armature Actual Velocity", motorRelEncoder.getVelocity()); 
    }
}