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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Armature;
import frc.robot.subsystems.Drawbridge;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import static edu.wpi.first.units.Units.Newton;

import java.util.List;

//import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;

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
  private static Elevator m_Elevator = new Elevator();
  private static Armature m_Armature = new Armature();
  private static Drawbridge m_Drawbridge = new Drawbridge();
  private static Intake m_Intake = new Intake();
  private static Climber m_Climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  //private Limelight3A limelight3A = new Limelight3A("limelight"); // Create a Limelight3A instance with the camera name
  //private final LimelightHelpers limelight = new LimelightHelpers();


  // The driver's controller
  XboxController m_driverController = new XboxController(0);
  Joystick j = new Joystick(1);

  Trigger xLB = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  Trigger xRB = new JoystickButton(m_driverController, Button.kRightBumper.value);
  Trigger xX = new JoystickButton(m_driverController, Button.kX.value);
  Trigger xY = new JoystickButton(m_driverController, Button.kY.value);
  Trigger xB = new JoystickButton(m_driverController, Button.kB.value);
  Trigger j1 = new JoystickButton(j, 1);
  Trigger j2 = new JoystickButton(j, 2);
  Trigger j3 = new JoystickButton(j, 3);
  Trigger j4 = new JoystickButton(j, 4);
  Trigger j5 = new JoystickButton(j, 5);
  Trigger j6 = new JoystickButton(j, 6);
  Trigger j7 = new JoystickButton(j, 7);
  Trigger j8 = new JoystickButton(j, 8);
  Trigger j9 = new JoystickButton(j, 9);
  Trigger j10 = new JoystickButton(j, 10);
  Trigger j11 = new JoystickButton(j, 11);
  Trigger j12 = new JoystickButton(j, 12);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand("autoElevatorReefDropoff", 
        m_Elevator.commandMoveElevator(Constants.ELEVATOR_L1_DROP));
    NamedCommands.registerCommand("autoElevatorToProcessor", 
        m_Elevator.commandMoveElevator(Constants.ELEVATOR_PROCESSOR));
    NamedCommands.registerCommand("autoDrawbridge", 
        m_Drawbridge.commandDrawbridge());
    NamedCommands.registerCommand("autoArmaturePick",
        m_Armature.commandArmature(Constants.ARMATURE_REEF));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(

                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //LimelightHelpers.setCamera("limelight"); // Set the camera name for LimelightHelpers
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

    xLB.onTrue(m_Intake
      .commandRotate(Constants.INTAKE_REV_SPEED))
      .onFalse(m_Intake.commandRotate(Constants.INTAKE_STOP_SPEED));
    xRB.onTrue(m_Intake
      .commandRotate(Constants.INTAKE_FWD_SPEED))
      .onFalse(m_Intake.commandRotate(Constants.INTAKE_STOP_SPEED));
    xX.onTrue(m_Climber.commandClimber(Constants.CLIMBER_POSITION_HANG));
    xY.onTrue(m_Climber.commandClimber(Constants.CLIMBER_POSITION_HOME));
    xB.onTrue(m_Climber.commandClimber(Constants.CLIMBER_POSITION_GRAB));
    j1.onTrue(m_Drawbridge.commandDrawbridge());
    j2.onTrue(m_Elevator.commandMoveElevator(Constants.ELEVATOR_MAX));
    j3.onTrue(m_Armature.commandArmature(Constants.ARMATURE_FLOOR_2));
    j4.onTrue(m_Armature.commandArmature(Constants.ARMATURE_FLOOR));
    j5.onTrue(m_Armature.commandArmature(Constants.ARMATURE_REEF));
    j6.onTrue(m_Armature.commandArmature(Constants.ARMATURE_HOME));
    j7.onTrue(m_Elevator.commandMoveElevator(Constants.ELEVATOR_TOP_ALGAE));
    j8.whileTrue(m_Drawbridge
      .commandDrawbridgeManual(Constants.DRAWBRIDGE_MANUAL_UP))
      .whileFalse(m_Drawbridge.commandDrawbridgeManual(Constants.DRAWBRIDGE_MANUAL_STOP));
    j9.onTrue(m_Elevator.commandMoveElevator(Constants.ELEVATOR_PROCESSOR));
    j10.whileTrue(m_Drawbridge
      .commandDrawbridgeManual(Constants.DRAWBRIDGE_MANUAL_DOWN))
      .whileFalse(m_Drawbridge.commandDrawbridgeManual(Constants.DRAWBRIDGE_MANUAL_STOP));
    j11.onTrue(m_Elevator.commandMoveElevator(Constants.ELEVATOR_HOME));
    j12.onTrue(m_Armature.commandArmatureReset());
    //j12.onTrue(m_Drawbridge.commandResetEncoder());
    //j12.onTrue(m_Climber.commandResetEncoder());      
  }


  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("CenterAuto");
    return autoChooser.getSelected();
  }


  public void showData() {
    Elevator.showData();
    m_Armature.showData(); 
    m_Drawbridge.showData();
    m_Intake.showData();
    m_Climber.showData();
  }

  public void checkBottomLimitSwitch() {
    Elevator.checkBottomLimitSwitch();
  }
}


