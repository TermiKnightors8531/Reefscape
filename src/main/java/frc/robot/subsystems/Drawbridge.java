package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

public class Drawbridge extends SubsystemBase {
    
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController motorClosedLoopEncoder;
    private RelativeEncoder motorRelEncoder;

    public Drawbridge() {
        motor = new SparkMax(Constants.DRAWBRIDGE_MOTOR_CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        motorClosedLoopEncoder = motor.getClosedLoopController();
        motorRelEncoder = motor.getEncoder();


        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1)
            .i(0)
            .d(0.1)
            .outputRange(-1, 1);
        
        // Max Velocity for Neo Motor is 5676 RPM
        motorConfig.closedLoop.maxMotion
            .maxVelocity(5000)
            .maxAcceleration(2500)
            .allowedClosedLoopError(1.5);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void resetEncoderPosition() {
        motorRelEncoder.setPosition(0);
    }

    public void gotoPosition() {
        if (motorRelEncoder.getPosition() >5) {
            motorClosedLoopEncoder.setReference(Constants.DRAWBRIDGE_HOME, ControlType.kMAXMotionPositionControl);
        } else {
            motorClosedLoopEncoder.setReference(Constants.DRAWBRIDGE_OPEN, ControlType.kMAXMotionPositionControl);
        }

    }

    public void manualOperation(double speed) {
        motor.set(speed);
    }

    public Command commandDrawbridgeManual(double speed) {
        return this.runOnce(() -> manualOperation(speed));
    }
    public Command commandDrawbridge() {
        return this.runOnce(() -> gotoPosition());
    }
    public Command commandResetEncoder() {
        return this.runOnce(() -> resetEncoderPosition());
    }

    public void showData() {
        SmartDashboard.putNumber("Drawbridge Actual Position", motorRelEncoder.getPosition());
        SmartDashboard.putNumber("Drawbridge Actual Velocity", motorRelEncoder.getVelocity());
    }
}

