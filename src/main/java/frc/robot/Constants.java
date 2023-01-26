// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6;
public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 50;
public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 4.99;
public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 52;
public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 5;
public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
public static final int BACK_LEFT_MODULE_STEER_ENCODER = 53;
public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0.6139;
public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 51;
public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 3.9;
}
