// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.constField;
import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.DriveManual;
import frc.robot.commands.SetCoralState;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController driverPartnerXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/main"));
  private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight");
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Hang hang = new Hang();
  private final AlgaePivot algaePivot = new AlgaePivot();
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
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
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);//, () -> driverXbox.rightBumper().getAsBoolean());

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    visionSubsystem.setMegaTag2(true);

    SmartDashboard.putData(autoChooser);

    drivebase.centerModulesCommand();
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
    Trigger wristGreaterThan80DegreesTrigger = new Trigger(() -> {
      if (arm.getWristEncoderPos() > 59) {
        return true;
      }
      return false;
    });

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
      .onTrue(arm.setManualArmVoltageWithLimiter(() -> MathUtil.applyDeadband(-driverPartnerXbox.getLeftY(), 0.01), 
                                                 () -> MathUtil.applyDeadband(-driverPartnerXbox.getRightY(), 0.01)))
      .onFalse(arm.stopWholeArm());

      /* //if limiter is bugging out, then uncomment this and comment out the thing above this
    YAxisJoystickTrigger
      .onTrue(arm.setManualArmVoltage(() -> MathUtil.applyDeadband(-driverPartnerXbox.getLeftY(), 0.01), 
                                      () -> MathUtil.applyDeadband(-driverPartnerXbox.getRightY(), 0.01)))
      .onFalse(arm.stopWholeArm());
*/
    drivebase.setDefaultCommand(
      new DriveManual(
        drivebase, () -> driverXbox.getLeftX(), () -> driverXbox.getLeftY(), 
        () -> driverXbox.getRightX(), () -> driverXbox.rightTrigger(0.01).getAsBoolean(), 
        () -> driverXbox.leftBumper().getAsBoolean(), () -> driverXbox.rightBumper().getAsBoolean(),
        () -> driverXbox.a().getAsBoolean()));
    
/* 
    driverXbox
      .a()
      .whileTrue(elevator.runSysIdRoutine());
*/
    driverXbox
      .start()
      .onTrue(Commands.runOnce(drivebase::zeroGyro));

    //hang
    driverXbox
      .x()
      .onTrue(hang.hangDown())
      .onFalse(hang.stopHang());

    driverXbox
      .y()
      .onTrue(hang.hangUp())
      .onFalse(hang.stopHang());

    /*driverXbox
      .leftBumper()
      .onTrue(drivebase.driveToPose(drivebase.getDesiredReef(true)));

    driverXbox
      .rightBumper()
      .onTrue(drivebase.driveToPose(drivebase.getDesiredReef(false)));
*/
    /*driverXbox
      .rightBumper()
      .onTrue(drivebase.driveToReef(false));

    driverXbox
      .leftBumper()
      .onTrue(drivebase.driveToReef(true));

    driverXbox
      .povUp()
      .onTrue(drivebase.driveToPose(constField.getReefPositions().get().get(1)));*/
    
    driverPartnerXbox
      .povUp()
      .onTrue(elevator.elevatorUp())
      .onFalse(elevator.stopElevator());

    driverPartnerXbox
      .povDown()
      .onTrue(elevator.elevatorDown())
      .onFalse(elevator.stopElevator());
    
    /*driverPartnerXbox
      .povLeft()
      .onTrue(new ParallelCommandGroup(
        arm.setArmPivotTarget(Constants.Arm.C_LOADING_ANGLE),
        elevator.setElevatorTarget(Constants.Elevator.C_LOADING_POS)
      ));*/

    /*driverPartnerXbox
      .rightStick()
      .onTrue(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            arm.setWristTarget(Constants.Wrist.C_GROUND_ANGLE),
            elevator.setElevatorTarget(Constants.Elevator.C_GROUND_POS)
          ),
          Commands.waitSeconds(0.3),
          arm.setArmPivotTarget(Constants.Wrist.C_GROUND_ANGLE)
        )
      );*/

    driverPartnerXbox
      .rightStick()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_GROUND));

    driverPartnerXbox
      .povLeft()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_LOAD));

    driverPartnerXbox
      .povRight()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_STOW));

    driverPartnerXbox
      .a()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_L1));
    
    driverPartnerXbox
      .b()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_L2));

    driverPartnerXbox
      .y()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_L3));

    driverPartnerXbox
      .x()
      .onTrue(new SetCoralState(arm, elevator, CoralStates.C_L4));
    
    driverPartnerXbox
      .leftBumper()
      .onTrue(coralIntake.outtakeCoral())
      .onFalse(coralIntake.stopIntake());
    
    driverPartnerXbox
      .rightBumper()
      .onTrue(coralIntake.intakeCoral())
      .onFalse(coralIntake.stopIntake());

    driverPartnerXbox
      .rightTrigger(0.01)
      .onTrue(algaeIntake.intakeAlgae())
      .onFalse(algaeIntake.stopAlgaeIntake());

    driverPartnerXbox
      .leftTrigger(0.01)
      .onTrue(algaeIntake.outtakeAlgae())
      .onFalse(algaeIntake.stopAlgaeIntake());

    //algae positions
    driverPartnerXbox
      .back()
      .onTrue(algaePivot.algaePivotUp())
      .onFalse(algaePivot.stopAlgaePivot());

    driverPartnerXbox
      .start()
      .onTrue(algaePivot.algaePivotDown())
      .onFalse(algaePivot.stopAlgaePivot());

    //sys id tests on swerve drive
    /*driverXbox
      .a()
      .and(driverXbox.leftBumper())
      .whileTrue(drivebase.sysIdAngleMotorCommand());
    driverXbox
      .a()
      .and(driverXbox.rightBumper())
      .whileTrue(drivebase.sysIdDriveMotorCommand());*/

     
    /* 
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedAnglularVelocitySim);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("C_LOAD", new SetCoralState(arm, elevator, CoralStates.C_LOAD));
    NamedCommands.registerCommand("C_L1", new SetCoralState(arm, elevator, CoralStates.C_L1));
    NamedCommands.registerCommand("C_L2", new SetCoralState(arm, elevator, CoralStates.C_L2));
    NamedCommands.registerCommand("C_L3", new SetCoralState(arm, elevator, CoralStates.C_L3));
    NamedCommands.registerCommand("C_L4", new SetCoralState(arm, elevator, CoralStates.C_L4));

    NamedCommands.registerCommand("C_INTAKE", coralIntake.intakeCoral());
    NamedCommands.registerCommand("C_OUTTAKE", coralIntake.outtakeCoral());
    NamedCommands.registerCommand("C_STOPINTAKE", coralIntake.stopIntake());
  }
}
