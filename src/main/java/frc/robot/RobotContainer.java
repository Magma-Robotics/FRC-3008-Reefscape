// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.constField;
import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.CoralAuto;
import frc.robot.commands.DriveManual;
import frc.robot.commands.OAPathCoral;
import frc.robot.commands.PathToCoral;
import frc.robot.commands.SetArmSetpointCommand;
import frc.robot.commands.SetCoralState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.util.List;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController driverPartnerXbox = new CommandXboxController(1);
  final CommandXboxController testXbox = new CommandXboxController(3);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/main"));
  private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight", drivebase);
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Hang hang = new Hang();
  private final Wrist wrist = new Wrist();
  private final SendableChooser<Command> autoChooser;
  private double slowMultiplier = 1;

  public Command C_L2() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L2).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L2)).
      alongWith(new SetArmSetpointCommand(arm, CoralStates.C_LOAD)).
      alongWith(Commands.waitUntil(elevator.atHeight(Constants.Elevator.C_L2_POS, 5)).andThen(arm.setArmSetpointCommand(CoralStates.C_L2)))
      .withTimeout(1.5);
  }

  public Command C_L3() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L3).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L3)).
      alongWith(new SetArmSetpointCommand(arm, CoralStates.C_LOAD)).
      alongWith(Commands.waitUntil(elevator.atHeight(Constants.Elevator.C_L3_POS, 15)).andThen(arm.setArmSetpointCommand(CoralStates.C_L3)))
      .withTimeout(1.5);
  }

  public Command C_L4() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L4).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L4)).
      alongWith(new SetArmSetpointCommand(arm, CoralStates.C_LOAD)).
      alongWith(Commands.waitUntil(elevator.atHeight(Constants.Elevator.C_L4_POS, 15)).andThen(arm.setArmSetpointCommand(CoralStates.C_L4)))
      .withTimeout(1.5);
  }

  public Command C_L2WithoutArm() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L2).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L2)).
      alongWith(arm.setArmSetpointCommand(CoralStates.C_LOAD))
      .withTimeout(1.5);
  }

  public Command C_L3WithoutArm() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L3).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L3)).
      alongWith(arm.setArmSetpointCommand(CoralStates.C_LOAD))
      .withTimeout(1.5);
  }

  public Command C_L4WithoutArm() {
    return elevator.setElevatorSetpointCommand(CoralStates.C_L4).
      alongWith(wrist.setWristSetpointCommand(CoralStates.C_L4)).
      alongWith(arm.setArmSetpointCommand(CoralStates.C_LOAD))
      .withTimeout(1.5);
  }


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * slowMultiplier,
                                                                () -> driverXbox.getLeftX() * slowMultiplier)
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);//, () -> driverXbox.rightBumper().getAsBoolean());

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(drivebase, visionSubsystem)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

  public void resetOdometry(Pose2d pose) {
    drivebase.resetOdometry(new Pose2d(pose.getTranslation(), drivebase.getPose().getRotation()));
  }

  public LimelightHelpers.PoseEstimate getVisionEstimate() {
    return visionSubsystem.GetVisionEstimate();
  }

  Pose2d[] SELECTED_AUTO_PREP_MAP;
  String SELECTED_AUTO_PREP_MAP_NAME = "none"; // For logging :p
  int AUTO_PREP_NUM = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    configureAutoBindings();
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("limelightCenter", limelightCenter());
    autoChooser.addOption("limelightLeft", limelightLeft());

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    visionSubsystem.setMegaTag2(true);

    SmartDashboard.putData(autoChooser);

    selectAutoMap();

    //drivebase.centerModulesCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    testXbox
      .povUp()
      .whileTrue(elevator.runSysIdRoutine());

    testXbox
      .povLeft()
      .whileTrue(wrist.runWristSysIdRoutine());
    
    testXbox
      .povDown()
      .whileTrue(arm.runArmSysIdRoutine());

    Trigger YAxisJoystickTrigger = new Trigger(() -> {
      if (MathUtil.applyDeadband(driverPartnerXbox.getLeftY(), 0.01) > 0.01|| 
          MathUtil.applyDeadband(driverPartnerXbox.getLeftY(), 0.01) < -0.01 ||
          MathUtil.applyDeadband(driverPartnerXbox.getRightY(), 0.01) > 0.01 || 
          MathUtil.applyDeadband(driverPartnerXbox.getRightY(), 0.01) < -0.01) {
        return true;
      } else {
        return false;
      }
    });

    YAxisJoystickTrigger
      .onTrue(arm.setManualArm(() -> MathUtil.applyDeadband(-driverPartnerXbox.getLeftY(), 0.01))
        .alongWith(wrist.setManualWrist(() -> MathUtil.applyDeadband(-driverPartnerXbox.getRightY(), 0.01))))
      .onFalse(arm.stopPIDArm().alongWith(wrist.stopPIDWrist()));

      /* //if limiter is bugging out, then uncomment this and comment out the thing above this
    YAxisJoystickTrigger
      .onTrue(arm.setManualArmVoltage(() -> MathUtil.applyDeadband(-driverPartnerXbox.getLeftY(), 0.01), 
                                      () -> MathUtil.applyDeadband(-driverPartnerXbox.getRightY(), 0.01)))
      .onFalse(arm.stopWholeArm());
*/
    /*drivebase.setDefaultCommand(
      new DriveManual(
        drivebase, () -> driverXbox.getLeftX(), () -> driverXbox.getLeftY(), 
        () -> driverXbox.getRightX(), () -> driverXbox.rightBumper().getAsBoolean(), 
        () -> driverXbox.leftTrigger(0.01).getAsBoolean(), 
        () -> driverXbox.rightTrigger(0.01).getAsBoolean(),
        () -> driverXbox.a().getAsBoolean(),
        () -> driverXbox.b().getAsBoolean(),
        () -> driverXbox.leftBumper().getAsBoolean(),
        () -> driverXbox.povUp().getAsBoolean()));*/
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
    driverXbox
      .start()
      .onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverXbox
      .back()
      .onTrue(Commands.runOnce(() -> {
        if (drivebase.resetPoseWithAprilTag()) {
          driverXbox.setRumble(RumbleType.kLeftRumble, 0.4);
        }
        }))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kLeftRumble, 0);
      }));

    //hang
    driverXbox
      .y()
      .onTrue(hang.hangDown())
      .onFalse(hang.stopHang());

    driverXbox
      .a()
      .onTrue(hang.hangUp())
      .onFalse(hang.stopHang());

    driverXbox
      .rightBumper()
      .onTrue(Commands.run(() -> slowMultiplier = 0.5))
      .onFalse(Commands.run(() -> slowMultiplier = 1));

    driverXbox
      .leftTrigger()
      .and(driverXbox.povDown())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(0))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .rightTrigger()
      .and(driverXbox.povDown())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(1))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .leftTrigger()
      .and(driverXbox.povRight())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(2))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .rightTrigger()
      .and(driverXbox.povRight())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(3))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .rightTrigger()
      .and(driverXbox.b())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(4))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .leftTrigger()
      .and(driverXbox.b())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(5))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .rightTrigger()
      .and(driverXbox.povUp())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(6))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .leftTrigger()
      .and(driverXbox.povUp())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(7))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

      driverXbox
      .rightTrigger()
      .and(driverXbox.x())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(8))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .leftTrigger()
      .and(driverXbox.x())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(9))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .leftTrigger()
      .and(driverXbox.povLeft())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(10))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverXbox
      .rightTrigger()
      .and(driverXbox.povLeft())
      .whileTrue(new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(11))
        .andThen(Commands.runOnce(() -> {
          driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
          driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0.5);
        })))
      .onFalse(Commands.runOnce(() -> {
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        driverPartnerXbox.setRumble(RumbleType.kBothRumble, 0);
      }));

    driverPartnerXbox
      .povUp()
      .onTrue(elevator.elevatorUp())
      .onFalse(elevator.stopElevator());

    driverPartnerXbox
      .povDown()
      .onTrue(elevator.elevatorDown())
      .onFalse(elevator.stopElevator());

    driverPartnerXbox
      .back()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW));

    driverPartnerXbox
      .povLeft()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.C_LOAD));

    driverPartnerXbox
      .povRight()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.C_GROUND));

    driverPartnerXbox
      .a()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.C_L1));
    
    driverPartnerXbox
      .b()
      .onTrue(C_L2WithoutArm())
      .onFalse(arm.setArmSetpointCommand(CoralStates.C_L2));

    driverPartnerXbox
      .y()
      .onTrue(C_L3WithoutArm())
      .onFalse(arm.setArmSetpointCommand(CoralStates.C_L3));

    driverPartnerXbox
      .x()
      .onTrue(C_L4WithoutArm())
      .onFalse(arm.setArmSetpointCommand(CoralStates.C_L4));
    
    driverPartnerXbox
      .rightStick()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.A_GROUND));

    driverPartnerXbox
      .start()
      .onTrue(new SetCoralState(arm, wrist, elevator, CoralStates.A_BARGE));
    
    driverPartnerXbox
      .leftBumper()
      .onTrue(coralIntake.outtakeCoral())
      .onFalse(coralIntake.stopIntake());
    
    driverPartnerXbox
      .rightBumper()
      .onTrue(coralIntake.intakeCoral())
      .onFalse(coralIntake.stopIntake());

    driverPartnerXbox
      .leftTrigger()
      .onTrue(coralIntake.slowOuttakeCoral())
      .onFalse(coralIntake.stopIntake());
    
    driverPartnerXbox
      .rightTrigger()
      .onTrue(coralIntake.slowIntakeCoral())
      .onFalse(coralIntake.stopIntake());

    //sys id tests on swerve drive
    /*driverXbox
      .a()
      .and(driverXbox.leftBumper())
      .whileTrue(drivebase.sysIdAngleMotorCommand());
    driverXbox
      .a()
      .and(driverXbox.rightBumper())
      .whileTrue(drivebase.sysIdDriveMotorCommand());*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    AUTO_PREP_NUM = 0;
    selectAutoMap();
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void configureAutoBindings() {
    Command driveAutoAlign = Commands.runOnce(() -> drivebase.autoAlign(
      Meters.of(drivebase.getPose().getTranslation().getDistance(SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM].getTranslation())), 
      SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM],
      MetersPerSecond.of(0), MetersPerSecond.of(0), 
      DegreesPerSecond.of(0), 1, Meters.of(40)));


    NamedCommands.registerCommand("C_STOW", new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW));
    NamedCommands.registerCommand("C_LOAD", new SetCoralState(arm, wrist, elevator, CoralStates.C_LOAD));
    NamedCommands.registerCommand("C_L1", new SetCoralState(arm, wrist, elevator, CoralStates.C_L1));
    NamedCommands.registerCommand("C_L2", C_L2());//new SetCoralState(arm, wrist, elevator, CoralStates.C_L2));
    NamedCommands.registerCommand("C_L3", C_L3());//new SetCoralState(arm, wrist, elevator, CoralStates.C_L3));
    NamedCommands.registerCommand("C_L4", C_L4());//new SetCoralState(arm, wrist, elevator, CoralStates.C_L4));

    NamedCommands.registerCommand("C_INTAKE", coralIntake.intakeCoral());
    NamedCommands.registerCommand("C_OUTTAKE", coralIntake.outtakeCoral());
    NamedCommands.registerCommand("C_STOPINTAKE", coralIntake.stopIntake());

    NamedCommands.registerCommand("CLIMB_IN", hang.hangUp());
    NamedCommands.registerCommand("CLIMB_OUT", hang.hangDown());
    NamedCommands.registerCommand("CLIMB_STOP", hang.stopHang());

    NamedCommands.registerCommand("autoAlign", Commands.deferredProxy(() -> new PathToCoral(drivebase, SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM])).withTimeout(5));
  }

  /**
   * Populates the selected AutoMap for your autonomous command.
   */
  private void selectAutoMap() {
    SELECTED_AUTO_PREP_MAP = configureAutoPrepMaps(autoChooser.getSelected().getName());
    SELECTED_AUTO_PREP_MAP_NAME = autoChooser.getSelected().getName();
  }

  private Pose2d[] configureAutoPrepMaps(String selectedAuto) {
    List<Pose2d> fieldPositions = constField.getReefPositions().get();

    switch (selectedAuto) {
      case "Middle L4 Auto":
        Pose2d[] middleL4Auto = new Pose2d[1];
        middleL4Auto[0] = fieldPositions.get(6); // G
        return middleL4Auto;

      case "Left 2 Coral Auto":
        Pose2d[] left2CoralAuto = new Pose2d[2];
        left2CoralAuto[0] = fieldPositions.get(9); // J
        left2CoralAuto[1] = fieldPositions.get(11); // L
        return left2CoralAuto;

      default:
        Pose2d[] noAutoSelected = new Pose2d[1];
        noAutoSelected[0] = new Pose2d();
        return noAutoSelected;
    }
  }

  public Command limelightCenter() {
    return new SequentialCommandGroup(
                                      hang.hangDown(),
                                      Commands.waitSeconds(2),
                                      hang.stopHang(),
                                      new CoralAuto(drivebase),
                                      new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(6)),
                                      C_L4(),
                                      Commands.waitSeconds(1.5),
                                      new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW));
  }

  public Command limelightLeft() {
    return new SequentialCommandGroup(
      hang.hangDown(),
      new CoralAuto(drivebase),
      new ParallelCommandGroup(
        new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(9)),
        new SequentialCommandGroup(
          Commands.waitSeconds(2),
          hang.stopHang()
        )
      ),
      C_L4(),
      coralIntake.outtakeCoral(),
      Commands.waitSeconds(0.4),
      coralIntake.intakeCoral(),
      new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW),
      new OAPathCoral(drivebase, Constants.constField.getCoralStationPositions().get().get(0)),
      Commands.waitSeconds(1),
      coralIntake.stopIntake(),
      new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(10)),
      C_L4(),
      coralIntake.outtakeCoral(),
      Commands.waitSeconds(0.4),
      new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW)
    );
  }

  public Command limelightRight() {
    return new SequentialCommandGroup(
      hang.hangDown(),
      new CoralAuto(drivebase),
      new ParallelCommandGroup(
        new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(4)),
        new SequentialCommandGroup(
          Commands.waitSeconds(2),
          hang.stopHang()
        )
      ),
      C_L4(),
      coralIntake.outtakeCoral(),
      Commands.waitSeconds(0.4),
      coralIntake.intakeCoral(),
      new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW),
      new OAPathCoral(drivebase, Constants.constField.getCoralStationPositions().get().get(2)),
      Commands.waitSeconds(1),
      coralIntake.stopIntake(),
      new OAPathCoral(drivebase, Constants.constField.getReefPositions().get().get(10)),
      C_L4(),
      coralIntake.outtakeCoral(),
      Commands.waitSeconds(0.4),
      new SetCoralState(arm, wrist, elevator, CoralStates.C_STOW)
    );
  }
}
