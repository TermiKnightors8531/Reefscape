package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private SparkMax climbMotor;
    private SparkMaxConfig climbConfig;
    private SparkClosedLoopController climbClosedLoopEncoder;
    private AbsoluteEncoder motorAbsEncoder;
    private RelativeEncoder motorRelEncoder;

    public Climber() {

        climbMotor = new SparkMax(Constants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();
        climbClosedLoopEncoder = climbMotor.getClosedLoopController();
        motorRelEncoder = climbMotor.getEncoder();
        motorAbsEncoder = climbMotor.getAbsoluteEncoder();

        climbConfig.idleMode(IdleMode.kBrake); // Set motor to brake mode when not in use

        climbConfig.encoder
            .positionConversionFactor(1.0);

        climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .p(1)
            .i(0)
            .d(0.1)
            .outputRange(-1, 1);
        
        // Max Velocity for Neo Motor is 5676 RPM
        climbConfig.closedLoop.maxMotion
            .maxVelocity(10)
            .maxAcceleration(5)
            .allowedClosedLoopError(0.1);

        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
        motor = new SparkMax(Constants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        
        motorClosedLoopEncoder = motor.getClosedLoopController();
        motorAbsEncoder = motor.getAbsoluteEncoder();
        motor.configure(Configs.MAXSwerveModule.ClimberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
*/
    }

    public void gotoPosition(double position) {

        climbClosedLoopEncoder.setReference(position, ControlType.kPosition);
    }


    public void resetEncoderPosition() {
        motorRelEncoder.setPosition(0);
    }

    //public double getPosition() {
    //    return motorAbsEncoder.getPosition();
    //}

    public Command commandClimber(double position) {
        return this.runOnce(() -> gotoPosition(position));
    }
                
    public Command commandResetEncoder() {
        return this.runOnce(() -> resetEncoderPosition());
    }
            
        
    public void showData() {
        SmartDashboard.putNumber("Climber Actual Position", motorRelEncoder.getPosition());
        SmartDashboard.putNumber("Climber Actual Velocity", motorRelEncoder.getVelocity()); 
        SmartDashboard.putNumber("Clibmer absolute Position", motorAbsEncoder.getPosition());
        SmartDashboard.putNumber("Climber abs Velocity", motorAbsEncoder.getVelocity());
    }
    
}
