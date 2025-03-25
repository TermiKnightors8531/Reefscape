package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private static SparkMax motor;
    private static SparkMaxConfig motorConfig;
    private static SparkClosedLoopController motorClosedLoopEncoder;
    private static RelativeEncoder motorRelEncoder;
    static DigitalInput bottomlimitSwitch = new DigitalInput(0);

    public Elevator() {
        motor = new SparkMax(Constants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
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
            .maxVelocity(5676)
            .maxAcceleration(2500)
            .allowedClosedLoopError(0.5);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void gotoPosition(double position) {
        motorClosedLoopEncoder.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public static void resetEncoderPosition() {
            motorRelEncoder.setPosition(0);
        }
    
    public static void checkBottomLimitSwitch() {
        if (bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
            motor.set(0);
            resetEncoderPosition();
        }
    }

    public static void showData() {
        SmartDashboard.putNumber("Elevator Position", motorRelEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Velocity", motorRelEncoder.getVelocity()); 
        SmartDashboard.putBoolean("Bottom Limit Switch", bottomlimitSwitch.get());
    }

}

