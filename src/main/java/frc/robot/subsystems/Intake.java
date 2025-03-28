package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Intake extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController motorClosedLoopEncoder;
    private RelativeEncoder motorRelEncoder;

    public Intake() {
        motor = new SparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
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
            .maxVelocity(500)
            .maxAcceleration(500)
            .allowedClosedLoopError(0.5);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

   
    public void goForward() {
        System.out.println("INTAKE GO FORWARD button pressed");
        motor.set(0.6);
        System.out.println("motor speed: " + motor.get());
    }
    public void goBackward() {
        motor.set(-0.6);
        System.out.println("INTAKE GO BACKWARD button pressed");
    }
    public void rotate(double percent) {
        motor.set(percent);
    }

    public void stop() {
        motor.set(0);
        System.out.println("INTAKE STOP button pressed");
    }

    public Command commandRotate(double percent) {
        return this.runOnce(() -> rotate(percent));
    }
    
    public Command commandGoForward() {
        return this.runOnce(() -> goForward());
    }
    public Command commandGoBackward() {
        return this.runOnce(() -> goBackward());
    }
    public Command commandStop() {
        return this.runOnce(() -> stop());
    }

    public void showData() {
        SmartDashboard.putNumber("Intake Speed", motor.get());
    
}
}
