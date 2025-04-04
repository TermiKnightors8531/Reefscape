// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static RobotConfig config;
 static{
    try{
      config = RobotConfig.fromGUISettings();
    }catch(Exception e){
      e.printStackTrace();
    }
  }
  
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    //public static final double kMaxSpeedMetersPerSecond = .10;
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    //public static final double kMaxAngularSpeed = .04 * Math.PI; // radians per second
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26 );
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      };

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 56;
    public static final int kRearLeftDrivingCanId = 58;
    public static final int kFrontRightDrivingCanId = 55;
    public static final int kRearRightDrivingCanId = 57;

    public static final int kFrontLeftTurningCanId = 51;
    public static final int kRearLeftTurningCanId = 53;
    public static final int kFrontRightTurningCanId = 52;
    public static final int kRearRightTurningCanId = 54;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    //public static final double kDrivingMotorFreeSpeedRps = .01;
    public static final double kDrivingMotorFreeSpeedRps = 5676;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    //public static final double kDriveWheelFreeSpeedRps = .01;
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps / 60 * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 100;
  }

  public static final double MAX_AUTO_SPEED = 1.0;

  public static final int INTAKE_MOTOR_CAN_ID = 61;
  public static final int ELEVATOR_MOTOR_CAN_ID = 59;
  public static final int ARMATURE_MOTOR_CAN_ID = 60;
  public static final int DRAWBRIDGE_MOTOR_CAN_ID = 62;
  public static final int CLIMBER_MOTOR_CAN_ID = 11;

  public static final double ELEVATOR_MAX = -200;
  public static final double ELEVATOR_L1_DROP = -135;
  public static final double ELEVATOR_TOP_ALGAE = -50;
  public static final double ELEVATOR_PROCESSOR = -25;
  public static final double ELEVATOR_HOME = 0;

  //public static final double ARMATURE_FLOOR = -Math.PI*5/6;//18.5;
  //public static final double ARMATURE_FLOOR_2 = -Math.PI*3/4;//17;
  //public static final double ARMATURE_REEF = -Math.PI*4/6;//12;
  public static final double ARMATURE_FLOOR = -18.5;
  public static final double ARMATURE_FLOOR_2 = -17;
  public static final double ARMATURE_REEF = -12;
  public static final double ARMATURE_HOME = 0;

  public static final double INTAKE_FWD_SPEED = 0.6;
  public static final double INTAKE_REV_SPEED = -0.6;
  public static final double INTAKE_STOP_SPEED = 0.0;

  public static final double DRAWBRIDGE_HOME = 0.0;
  public static final double DRAWBRIDGE_OPEN = 30.0;
  public static final double DRAWBRIDGE_MANUAL_STOP = 0.0;
  public static final double DRAWBRIDGE_MANUAL_DOWN = -0.4;
  public static final double DRAWBRIDGE_MANUAL_UP = 0.4;

  public static final double CLIMBER_POSITION_HOME = 0.0; // Driving position
  public static final double CLIMBER_POSITION_HANG = 29.0; // Hang position
  public static final double CLIMBER_POSITION_GRAB = -40.0; // Grab cage position  

}
