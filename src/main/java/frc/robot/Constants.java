// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class ShooterConstants
  {
    public static final int LEFT_SHOOTER_CAN_ID = 14;
    public static final int RIGHT_SHOOTER_CAN_ID = 15;
    public static final double SCALING_FACTOR_SHOOTER = 1;
    public static final double SHOOTER_SPEED = 0.2;
  }

  public static final class ClimberConstants
  {
    public static final int LEFT_CLIMBER_CAN_ID = 17;
    public static final int RIGHT_CLIMBER_CAN_ID = 18;
    public static final double SCALING_FACTOR_CLIMBER = 0.3;
  }


  public static final class IntakeConstants
  {
    public static final int INTAKE_CAN_ID = 16;
    public static final double SCALING_FACTOR_INTAKE = 0.3;
    public static final double INTAKE_STANDARD_UP = 0.5;
    public static final double INTAKE_STANDARD_DOWN = 0.2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Xbox Controller #s
    public static final int XBOX_A_BUTTON = 1;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_L_BUMPER = 5;
    public static final int XBOX_R_BUMPER = 6;

    // Joystick Deadband
    public static final double LEFT_Y_DEADBAND_CLIMBER = 0.2;
    public static final double RIGHT_Y_DEADBAND_CLIMBER = 0.2;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double LT_DEADBAND = 0.1;
    public static final double RT_DEADBAND = 0.1;


  }
}
