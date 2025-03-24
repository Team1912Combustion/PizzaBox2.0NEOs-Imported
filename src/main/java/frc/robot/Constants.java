// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants {
    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 10000.; //2 * 2 * Math.PI;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQUARED = 10000.; //2 * 2 * Math.PI;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_GEAR_RATIO = 6.75;
    public static final double DRIVE_ENCODER_DISTANCE_PER_ROTATION =
      (WHEEL_DIAMETER_METERS * Math.PI) / WHEEL_GEAR_RATIO;

    public static final double P_MODULE_TURNING_CONTROLLER = 0.5;
    public static final double I_MODULE_TURNING_CONTROLLER = 0.0;
    public static final double D_MODULE_TURNING_CONTROLLER = 0.0067;

  }

  public static class VisionConstants {
    public static final double TARGET_AREA_THRESHHOLD = 1.5;
    public static final double TOTAL_TARGET_AREA_THRESHHOLD = 3.0;
  }

  public static class DriveConstants {

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 2;
    public static final int FRONT_LEFT_TURNING_ENCODER_ID = 1;
    public static final double FRONT_LEFT_MAGNET_OFFSET = -0.132568359375;

    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_LEFT_TURNING_MOTOR_ID = 4;
    public static final int REAR_LEFT_TURNING_ENCODER_ID = 2;
    public static final double REAR_LEFT_MAGNET_OFFSET = -0.395263671875;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 7;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 8;
    public static final int FRONT_RIGHT_TURNING_ENCODER_ID = 4;
    public static final double FRONT_RIGHT_MAGNET_OFFSET = -0.323974609375;

    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 5;
    public static final int REAR_RIGHT_TURNING_MOTOR_ID = 6;
    public static final int REAR_RIGHT_TURNING_ENCODER_ID = 3;
    public static final double REAR_RIGHT_MAGNET_OFFSET = -0.495849609375;

    public static final int GYROSCOPE_ID = 0;

    public static final double DRIVE_SPEED = 1;

    public static final double TRACK_WIDTH = Units.inchesToMeters(19);
    public static final double WHEEL_BASE = Units.inchesToMeters(19);
    /*public static final double FRONT_WHEEL_X = Units.inchesToMeters(6.456);
    public static final double REAR_WHEEL_X = Units.inchesToMeters(12.293);*/
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    //FL
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   //RL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),  //RR
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)   //FR
            );

    public static final boolean GYRO_REVERSED = false;
    public static final double OFFSET_X_TO_ROT = 0.05;

    public static final double S_VOLTS = 1;
    public static final double V_VOLT_SECONDS_PER_METER = 0.8;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.15;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
  }

  public static class DriverController {
    public static final double XY_DEADBAND = .05;
    public static final double ROT_DEADBAND = .05;

    public static final int DRIVER_JOYSTICK = 0;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
  }

  public static class OperatorController {
    public static final int OPERATOR_JOYSTICK = 1;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_JOYSTICK = 9;
  }

}