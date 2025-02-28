// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public static final double ROBOT_MASS = (120) * 0.453592; // 32lbs * kg per pound
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

  public static class Testing {
    //keep all as false unless testing positions
    public static final boolean testingArm = false;
    public static final boolean testingWrist = false;
    public static final boolean testingElevator = false;
    public static final boolean testingCoralIntake = false;
  }

  public static class Arm {
    public static final double armGearRatio = 300;
    public static final double kArmRotationsToDeg = (1/armGearRatio)*360;
    public static final double kArmRPMtoDegPerSec = (360/(armGearRatio*60));
    public static final double C_STOW_ANGLE = 0;
    public static final double C_L1_ANGLE = 35;
    public static final double C_L2_ANGLE = 15;
    public static final double C_L3_ANGLE = 20;
    public static final double C_L4_ANGLE = 25;
    public static final double C_LOADING_ANGLE = 10;
    public static final double C_GROUND_ANGLE = 83;
  }

  public static class Wrist {
    public static final double wristGearRatio = 72;
    public static final double kWristRotationsToDeg = (1/wristGearRatio)*360;
    public static final double kWristRPMtoDegPerSec = (360/(wristGearRatio*60));
    public static final double C_STOW_ANGLE = 0;
    public static final double C_L1_ANGLE = 40;
    public static final double C_L2_ANGLE = 90;
    public static final double C_L3_ANGLE = 90;
    public static final double C_L4_ANGLE = 80;
    public static final double C_LOADING_ANGLE = 10;
    public static final double C_GROUND_ANGLE = 60;
  }

  public static class Elevator {
    //everything in inches
    public static final double gearRatio = 10.20408;
    public static final Distance wheelDiameter = Inches.of(1.756);
    public static final double kElevatorRotationsToInches = ((Math.PI*wheelDiameter.in(Inches))/gearRatio)*2;
    public static final double kElevatorRPMToInPerSec = kElevatorRotationsToInches/60;
    public static final Distance elevatorHeightOffGround = Inches.of(9);
    public static final Distance kMinElevatorHeight = Inches.of(0);
    public static final Distance kMaxElevatorHeight = Inches.of(68);
    public static final double C_STOW_POS = 0;
    public static final double C_L1_POS = 0;//Centimeters.of(46).in(Inches)-9;
    public static final double C_L2_POS = 15;//Centimeters.of(81).in(Inches)-9; //15
    public static final double C_L3_POS = 35;//Centimeters.of(121).in(Inches)-9; //35
    public static final double C_L4_POS = 68;//Centimeters.of(183).in(Inches)-9; //68
    public static final double C_LOADING_POS = 0;
    public static final double C_GROUND_POS = 0;
  }

  public static class Algae {
    public static final double gearRatio = 75;
    public static final double kAlgaePivotRotationsToDeg = (1/gearRatio)*360;
    public static final double A_STOW_ANGLE = 0;
    public static final double A_PROCCESSOR_ANGLE = 0;
    public static final double A_LOADING_ANGLE = 0;
  }

  public static class Hang {
    public static final double gearRatio = 125;
    public static final double kHangPivotRotationsToDeg = (1/gearRatio)*360;
    public static final double maxHangAngle = 0;
    public static final double minHangAngle = 0;
  }

  public static class RobotStates {
    public enum CoralStates {
      C_STOW,
      C_L1,
      C_L2,
      C_L3,
      C_L4,
      C_LOAD,
      C_GROUND;
    }

    public enum AlgaeStates {
      A_STOW,
      A_PROCCESSOR,
      A_LOAD
    }
  }

  public static class Drive {
    public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Meters.of(10);
    public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Meters.of(1);
    public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Meters.of(5);
  }

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static final Distance FIELD_LENGTH = Feet.of(57).plus(Inches.of(6 + 7 / 8));
    public static final Distance FIELD_WIDTH = Feet.of(26).plus(Inches.of(5));

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    /*
     * All poses on the field, defined by their location on the BLUE Alliance
     */
    public static class POSES {
      public static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());
      public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

      // BRANCH POSES
      public static final Pose2d REEF_A = new Pose2d(2.860, 4.187, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_B = new Pose2d(2.860, 3.857, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_C = new Pose2d(3.527, 2.694, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_D = new Pose2d(3.813, 2.535, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_E = new Pose2d(5.160, 2.529, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_F = new Pose2d(5.445, 2.694, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_G = new Pose2d(6.119, 3.857, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_H = new Pose2d(6.119, 4.187, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_I = new Pose2d(5.452, 5.343, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_J = new Pose2d(5.166, 5.527, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_K = new Pose2d(3.826, 5.508, Rotation2d.fromDegrees(-60));
      public static final Pose2d REEF_L = new Pose2d(3.534, 5.368, Rotation2d.fromDegrees(-60));

      // CORAL STATION POSES
      public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(125));
      public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(125));
      public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(-125));
      public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(-125));

      // processor poses
      public static final Pose2d PROCESSOR = new Pose2d(6, .77, Rotation2d.fromDegrees(-90));

      private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
      private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

      private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

      private static final Pose2d[] RED_POSES = getRedAlliancePoses();

      private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
          LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
      private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

      private static final Pose2d BLUE_PROCESSOR_POSE = PROCESSOR;
      private static final Pose2d RED_PROCESSOR_POSE = getRedProcessorPose();

      private static final List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR_POSE, RED_PROCESSOR_POSE);
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH.in(Meters) - (bluePose.getX()),
          FIELD_WIDTH.in(Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private static Pose2d[] getRedAlliancePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

      for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
      }
      return returnedPoses;
    }

    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

      for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    private static List<Pose2d> getRedCoralStationPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

      for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
    }

    private static Pose2d getRedProcessorPose() {
      Pose2d returnedPose = POSES.BLUE_PROCESSOR_POSE;

      returnedPose = getRedAlliancePose(POSES.BLUE_PROCESSOR_POSE);

      return returnedPose;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions
     */
    public static Supplier<Pose2d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_POSES;

      }
      return () -> POSES.BLUE_POSES;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of the reef branches for your alliance
     */
    public static Supplier<List<Pose2d>> getReefPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_REEF_POSES;

      }
      return () -> POSES.BLUE_REEF_POSES;
    }

    public static Supplier<List<Pose2d>> getCoralStationPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_CORAL_STATION_POSES;
      }
      return () -> POSES.BLUE_CORAL_STATION_POSES;
    }

    public static Supplier<List<Pose2d>> getProcessorPositions() {
      return () -> POSES.PROCESSOR_POSES;
    }
  }
}
