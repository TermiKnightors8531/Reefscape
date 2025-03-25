// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Armature;
import frc.robot.subsystems.Drawbridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

@SuppressWarnings("unused")

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  static Elevator m_Elevator = new Elevator();
  Armature m_Armature = new Armature();
  Drawbridge m_Drawbridge = new Drawbridge();
  Intake m_Intake = new Intake();
  private final SendableChooser<Command> autoChooser;







  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController = new XboxController(0);
  Joystick j = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    //path planner thingy?
    NamedCommands.registerCommand("Elevator Shortest", elevatorUpShortest()); 
    NamedCommands.registerCommand("Elevator Normal", elevatorNormal());
    NamedCommands.registerCommand("Drawbridge Open", drawbridgeOpen());
    NamedCommands.registerCommand("Drawbridge Close", drawbridgeClose());
    //???



    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            /*() -> m_robotDrive.drive(

                -MathUtil.applyDeadband(m_driverController.getLeftY()*.1, OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX()*.1, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*.1, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
            */
            () -> m_robotDrive.drive(

                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    m_Elevator.setDefaultCommand(
        new RunCommand(
            () -> {
                if (j.getRawButtonPressed(2)) {
                    m_Elevator.gotoPosition(Constants.ELEVATOR_MAX);
                  //System.out.println("GoUpAndDown button pressed");
                } else if (j.getRawButtonReleased(7)) {
                    m_Elevator.gotoPosition(Constants.ELEVATOR_TOP_ALGAE);
                  //System.out.println("GoUpAndDown button pressed");
                } else if (j.getRawButtonPressed(11)) {
                    m_Elevator.gotoPosition(Constants.ELEVATOR_HOME);
                    //System.out.println("GoUpAndDown button pressed");
                //} else if (j.getRawButtonPressed(12)) {
                    //Elevator.resetEncoderPosition();
                } else if (j.getRawButtonPressed(9)) {
                    m_Elevator.gotoPosition(Constants.ELEVATOR_PROCESSOR);
                }
            }, m_Elevator)
    );
    
    m_Armature.setDefaultCommand(
      new RunCommand(
        () -> {
          if (j.getRawButtonPressed(5)) {
            m_Armature.gotoPosition(Constants.ARMATURE_REEF);
          } else if (j.getRawButtonReleased(4)) {
            m_Armature.gotoPosition(Constants.ARMATURE_FLOOR);
          } else if (j.getRawButtonPressed(6)) {
            m_Armature.gotoPosition(Constants.ARMATURE_HOME);
          } else if (j.getRawButtonPressed(3)) {
            
            m_Armature.gotoPosition(Constants.ARMATURE_FLOOR_2);
          //} else if (j.getRawButtonPressed(12)) {
            //m_Armature.resetEncoderPosition();
          }
        }, m_Armature)
    );

    m_Drawbridge.setDefaultCommand(
      new RunCommand(
        () -> {
          if (j.getRawButtonPressed(1)) {
            m_Drawbridge.gotoPosition();
          } else if (j.getRawButtonPressed(12)) {
            m_Drawbridge.resetEncoderPosition();
          } else if(j.getRawButtonPressed(8)) {
            m_Drawbridge.goUp();
          } else if(j.getRawButtonReleased(8)){
            m_Drawbridge.stopBridge();
          } else if(j.getRawButtonPressed(10)) {
            m_Drawbridge.goDown();
          } else if(j.getRawButtonReleased(10)){
            m_Drawbridge.stopBridge();
          }
        }, m_Drawbridge)
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_driverController, Button.kR1.value)
    //    .whileTrue(new RunCommand(
    //        () -> m_robotDrive.setX(),
    //        m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
          () -> m_Intake.goForward(), 
          m_Intake));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
          () -> m_Intake.goBackward(),
          m_Intake));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileFalse(new RunCommand(
          () -> m_Intake.stop(), 
          m_Intake));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileFalse(new RunCommand(
          () -> m_Intake.stop(), 
          m_Intake));     
  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("CenterAuto");
  }

  //new things that probably won't work - gabe
  public Command elevatorUpShortest() {
    return new RunCommand(
      () -> m_Elevator.gotoPosition(-25),
      m_Elevator);
  }
  public Command elevatorNormal() {
    return new RunCommand(
      () -> m_Elevator.gotoPosition(0),
      m_Elevator);
  }
  public Command drawbridgeOpen() {
    return new RunCommand(
      () -> m_Drawbridge.gotoPosition(),
      m_Drawbridge);
  }
  public Command drawbridgeClose() {
    return new RunCommand(
      () -> m_Drawbridge.gotoPosition(),
      m_Drawbridge);
  }
  //end of things :)
  



  public void showData() {
    Elevator.showData();
    m_Armature.showData(); 
    m_Drawbridge.showData();
    m_Intake.showData();
  }

  public void checkBottomLimitSwitch() {
    Elevator.checkBottomLimitSwitch();
  }
}


