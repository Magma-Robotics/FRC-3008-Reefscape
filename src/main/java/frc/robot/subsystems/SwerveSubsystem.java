// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.constField;

//import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
//import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;
  /**
   * AprilTag field layout.
   */
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean             visionDriveTest     = false;
  public boolean b_IsPositionCameraInitalized = false;

  public final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight", this);
  /**
   * PhotonVision class to keep an accurate odometry.
   */
//  private       Vision              vision;

  private Field2d field2d = new Field2d();
  Pose2d desiredAlignmentPose = Pose2d.kZero;

  private Pose2d desiredLeftReef = Pose2d.kZero;
  private Pose2d desiredRightReef = Pose2d.kZero;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(0),
                                                                                               Meter.of(0)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
//    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    if (visionDriveTest)
    {
//      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

//  /**
//   * Setup the photon vision class.
//   */
//  public void setupPhotonVision()
//  {
//    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
//  }

  @Override
  public void periodic()
  {
    //Check to see if have been camera initalized
    if(!b_IsPositionCameraInitalized){
      //If here, robot position was not initalized by the camera yet
      LimelightHelpers.PoseEstimate initEstimate = visionSubsystem.GetVisionEstimate();
      if(initEstimate != null){ //Check if subsystem is givng actual values
        //Reset swerve drive odometry to camera pose
        if (initEstimate.tagCount > 0) {
          swerveDrive.resetOdometry(initEstimate.pose);
          b_IsPositionCameraInitalized = true;
        }
      }
    }

    SmartDashboard.putNumber("X Pos", getPose().getX());
    SmartDashboard.putNumber("Y Pos", getPose().getY());
    SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("DesiredLeftReef", getDesiredReef(true).getX());
    SmartDashboard.putNumber("DesiredRightReef", getDesiredReef(false).getX());
    field2d.setRobotPose(getPose());
    SmartDashboard.putData("Field", field2d);
  }

  public Pose2d getDesiredLeftReef () {
    return desiredLeftReef;
  }
  
  public Pose2d getDesiredRightReef () {
    return desiredRightReef;
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.01, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Returns the closest reef branch to the robot.
   * 
   * @param leftBranchRequested If we are requesting to align to the left or right
   *                            branch
   * @return The desired reef branch face to align to
   */
  public Pose2d getDesiredReef(boolean leftBranchRequested) {
    // Get the closest reef branch face using either branch on the face
    List<Pose2d> reefPoses = constField.getReefPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredReef = currentPose.nearest(reefPoses);
    int closestReefIndex = reefPoses.indexOf(desiredReef);

    // Invert faces on the back of the reef so they're always relative to the driver
    if (closestReefIndex > 3 && closestReefIndex < 10) {
      leftBranchRequested = !leftBranchRequested;
    }

    // If we were closer to the left branch but selected the right branch (or
    // vice-versa), switch to our desired branch
    if (leftBranchRequested && (closestReefIndex % 2 == 1)) {
      desiredReef = reefPoses.get(closestReefIndex - 1);
    } else if (!leftBranchRequested && (closestReefIndex % 2 == 0)) {
      desiredReef = reefPoses.get(closestReefIndex + 1);
    }
    return desiredReef;
  }

  //
  public Pose2d getDesiredCoralStation(boolean farCoralStationRequested) {
    // Get the closest coral station
    List<Pose2d> coralStationPoses = constField.getCoralStationPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredCoralStation = currentPose.nearest(coralStationPoses);
    int closestCoralStationIndex = coralStationPoses.indexOf(desiredCoralStation);

    // If we were closer to the left branch but selected the right branch (or
    // vice-versa), switch to our desired branch
    if (farCoralStationRequested && (closestCoralStationIndex % 2 == 1)) {
      desiredCoralStation = coralStationPoses.get(closestCoralStationIndex - 1);
    } else if (!farCoralStationRequested && (closestCoralStationIndex % 2 == 0)) {
      desiredCoralStation = coralStationPoses.get(closestCoralStationIndex + 1);
    }

    return desiredCoralStation;
  }

  public Pose2d getDesiredProcessor() {
    // Get the closest processor
    List<Pose2d> processorPoses = constField.getProcessorPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredProcessor = currentPose.nearest(processorPoses);

    return desiredProcessor;
  }

  /**
   * Returns the rotational velocity calculated with PID control to reach the
   * given rotation. This must be called every loop until you reach the given
   * rotation.
   * 
   * @param desiredYaw The desired yaw to rotate to
   * @return The desired velocity needed to rotate to that position.
   */
  public AngularVelocity getVelocityToRotate(Rotation2d desiredYaw) {
    double yawSetpoint = Constants.Drive.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.getThetaController()
        .calculate(getHeading().getRadians(), desiredYaw.getRadians());

    // limit the PID output to our maximum rotational speed
    yawSetpoint = MathUtil.clamp(yawSetpoint, -Constants.Drive.TURN_SPEED.in(Units.RadiansPerSecond),
        Constants.Drive.TURN_SPEED.in(Units.RadiansPerSecond));

    return Units.RadiansPerSecond.of(yawSetpoint);
  }

  /**
   * Calculate the ChassisSpeeds needed to align the robot to the desired pose.
   * This must be called every loop until you reach the desired pose.
   * 
   * @param desiredPose The desired pose to align to
   * @return The ChassisSpeeds needed to align the robot to the desired pose
   */
  public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose) {
    desiredAlignmentPose = desiredPose;
    // TODO: This might run better if instead of 0, we use
    // constDrivetrain.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond);.
    // I dont know why. it might though
    return Constants.Drive.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(getPose(), desiredPose, 
      Constants.Drive.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond),
      desiredPose.getRotation());
  }

  public void rotationalAutoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, double elevatorMultiplier, Distance maxAutoDriveDistance) {

    //int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;

    // Rotational-only auto-align
    drive(
        new Translation2d(xVelocity.in(Units.MetersPerSecond),
            yVelocity.in(Units.MetersPerSecond)),
        getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), true);
  }

  public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, double elevatorMultiplier, Distance maxAutoDriveDistance) {
    desiredAlignmentPose = desiredTarget;
    //int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;

    if (distanceFromTarget.gte(maxAutoDriveDistance)) {
      // Rotational-only auto-align
      drive(
          new Translation2d(xVelocity.in(Units.MetersPerSecond),
              yVelocity.in(Units.MetersPerSecond)),
          getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), true);
    } else {
      // Full auto-align
      ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget);

      // Speed limit based on elevator height
      LinearVelocity linearSpeedLimit = Constants.Drive.OBSERVED_DRIVE_SPEED.times(elevatorMultiplier);
      AngularVelocity angularSpeedLimit = Constants.Drive.TURN_SPEED.times(elevatorMultiplier);

      if (!RobotState.isAutonomous()) {
        if ((desiredChassisSpeeds.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (desiredChassisSpeeds.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (desiredChassisSpeeds.omegaRadiansPerSecond > angularSpeedLimit.in(Units.RadiansPerSecond))) {

          desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vxMetersPerSecond, 0,
              linearSpeedLimit.in(MetersPerSecond));
          desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vyMetersPerSecond, 0,
              linearSpeedLimit.in(MetersPerSecond));
          desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredChassisSpeeds.omegaRadiansPerSecond, 0,
              angularSpeedLimit.in(RadiansPerSecond));
        }
      }

      drive(desiredChassisSpeeds);
    }
  }

  public boolean isAtRotation(Rotation2d desiredRotation) {
    return (getHeading().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(Constants.Drive.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0) &&
        getHeading().getMeasure()
            .compareTo(desiredRotation.getMeasure().plus(Constants.Drive.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
  }

  public boolean isAtPosition(Pose2d desiredPose2d) {
    return Units.Meters
        .of(getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
        .lte(Constants.Drive.TELEOP_AUTO_ALIGN.AT_POINT_TOLERANCE);
  }

  public Boolean isAligned() {
    return (desiredAlignmentPose.getTranslation().getDistance(
        getPose().getTranslation()) <= Constants.Drive.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_TOLERANCE.in(Units.Meters))
        && isAtRotation(desiredAlignmentPose.getRotation());
  }

  public boolean atPose(Pose2d desiredPose) {
    return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
  }


  public boolean resetPoseWithAprilTag() {
    Pose2d currentPose = getPose();
      try {
        Pose2d visionEstimatePose = visionSubsystem.GetVisionEstimate().pose;
        resetOdometry(new Pose2d(visionEstimatePose.getX(), visionEstimatePose.getY(), getPose().getRotation()));
        Commands.print("Updated POSE");
      }
      catch(Exception e) {
        resetOdometry(currentPose);
        return false;
      }
    return true;
  }

  public Command driveToReef(boolean leftBranchRequested) {
    Pose2d desiredReef = getDesiredReef(leftBranchRequested);
    return driveToPose(desiredReef);
  }

  public Command alignToReef(boolean leftBranchRequested, LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity rVelocity) {
    Pose2d desiredReef = getDesiredReef(leftBranchRequested);
    Distance reefDistance = Meters.of(getPose().getTranslation().getDistance(desiredReef.getTranslation()));
    return run(() -> autoAlign(reefDistance, desiredReef, MetersPerSecond.of(0), 
    MetersPerSecond.of(0), RadiansPerSecond.of(0), 
    1, Constants.Drive.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE));
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 2,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.Degrees.of(720).in(Radians));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    swerveDrive.addVisionMeasurement(pose, timestamp, VecBuilder.fill(.7,.7,9999999));
  }

  public double getGyroRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().baseUnitMagnitude();
  }

  public void updateOdomForPathPlanning() {
    b_IsPositionCameraInitalized = false;
  }
}
