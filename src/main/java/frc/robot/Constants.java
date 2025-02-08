// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

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
  public static final double MAX_SPEED  = Units.feetToMeters(12.66);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  
  public static class CANIds {
    public static final int kArmPivotID = 15;
    public static final int kWristID = 16;
    public static final int kCoralIntakeID = 17;
    public static final int kLeftElevatorID = 13;
    public static final int kRightElevatorID = 14;
    public static final int kHangID = 20;
    public static final int kAlgaePivotID = 18;
    public static final int kAlgaeIntakeID = 19;
  }

  public static class Arm {
    public static final double armGearRatio = 300;
    public static final double kArmRotationsToDeg = (1/armGearRatio)*360;
    public static final double kArmRPMtoDegPerSec = (360/(armGearRatio*60));
    public static final double C_STOW_ANGLE = 0;
    public static final double C_L1_ANGLE = 0;
    public static final double C_L2_ANGLE = 0;
    public static final double C_L3_ANGLE = 0;
    public static final double C_L4_ANGLE = 0;
    public static final double C_LOADING_ANGLE = 0;
  }

  public static class Wrist {
    public static final double wristGearRatio = 72;
    public static final double kWristRotationsToDeg = (1/wristGearRatio)*360;
    public static final double kWristRPMtoDegPerSec = (360/wristGearRatio*60);
    public static final double C_STOW_ANGLE = 0;
    public static final double C_L1_ANGLE = 0;
    public static final double C_L2_ANGLE = 0;
    public static final double C_L3_ANGLE = 0;
    public static final double C_L4_ANGLE = 0;
    public static final double C_LOADING_ANGLE = 0;
  }

  public static class Elevator {
    //everything in inches
    public static final double gearRatio = 10.20408;
    public static final Distance wheelDiameter = Inches.of(1.756);
    public static final double kElevatorRotationsToInches = ((Math.PI*wheelDiameter.in(Inches))/gearRatio)*2;
    public static final Distance elevatorHeightOffGround = Inches.of(9);
    public static final double C_STOW_POS = 0;
    public static final double C_L1_POS = Centimeters.of(46).in(Inches)-9;
    public static final double C_L2_POS = Centimeters.of(81).in(Inches)-9;
    public static final double C_L3_POS = Centimeters.of(121).in(Inches)-9;
    public static final double C_L4_POS = Centimeters.of(183).in(Inches)-9;
    public static final double C_LOADING_POS = 0;
  }

  public static class Algae {
    public static final double gearRatio = 75;
    public static final double kAlgaePivotRotationsToDeg = (1/gearRatio)*360;
    public static final double A_STOW_ANGLE = 0;
    public static final double A_PROCCESSOR_ANGLE = 0;
    public static final double A_LOADING_ANGLE = 0;
  }

  public static class RobotStates {
    public enum CoralStates {
      C_STOW,
      C_L1,
      C_L2,
      C_L3,
      C_L4,
      C_LOAD;
    }

    public enum AlgaeStates {
      A_STOW,
      A_PROCCESSOR,
      A_LOAD
    }
  }
}
