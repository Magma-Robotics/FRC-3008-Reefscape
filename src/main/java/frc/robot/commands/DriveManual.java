package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveManual extends Command {
    SwerveSubsystem drive;
    DoubleSupplier xAxis, yAxis, rotationAxis;
    BooleanSupplier slowMode, leftReef, rightReef, coralStationLeft, coralStationRight, processor, resetPoseWithLimelight;
    double slowMultiplier = 0;
    SwerveInputStream chassisSpeeds;
    boolean reefDebounce = false;

    public DriveManual(SwerveSubsystem drive, DoubleSupplier xAxis, DoubleSupplier yAxis, 
            DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef, 
            BooleanSupplier coralStationLeft, BooleanSupplier coralStationRight,
            BooleanSupplier processorBtnBooleanSupplier, BooleanSupplier resetPoseWithLimelight) {
        this.drive = drive;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.rotationAxis = rotationAxis;
        this.slowMode = slowMode;
        this.leftReef = leftReef;
        this.rightReef = rightReef;
        this.coralStationLeft = coralStationLeft;
        this.coralStationRight = coralStationRight;
        this.processor = processorBtnBooleanSupplier;
        this.resetPoseWithLimelight = resetPoseWithLimelight;

        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        
    }
  
    @Override
    public void execute() {
        // -- Multipliers --
        if (slowMode.getAsBoolean()) {
            slowMultiplier = 0.5;
        } else {
            slowMultiplier = 1;
        }

        double elevatorHeightMultiplier = 1;

        double transMultiplier = slowMultiplier * elevatorHeightMultiplier;

        chassisSpeeds = SwerveInputStream.of(drive.getSwerveDrive(),
                                                                () -> yAxis.getAsDouble() * transMultiplier,
                                                                () -> xAxis.getAsDouble() * transMultiplier)
                                                            .withControllerRotationAxis(() -> -rotationAxis.getAsDouble() * transMultiplier)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

         // -- Velocities --
        LinearVelocity xVelocity = Units.MetersPerSecond.of(chassisSpeeds.get().vxMetersPerSecond);
        LinearVelocity yVelocity = Units.MetersPerSecond.of(chassisSpeeds.get().vyMetersPerSecond);
        AngularVelocity rVelocity = Units.RadiansPerSecond
            .of(chassisSpeeds.get().omegaRadiansPerSecond);

        // -- Reef --
        if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
            //Pose2d desiredReef = drive.getDesiredReef(leftReef.getAsBoolean());
            //Distance reefDistance = Meters.of(drive.getPose().getTranslation().getDistance(desiredReef.getTranslation()));

            /*Commands.deferredProxy(() -> Commands.run(() -> drive.autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, 
                transMultiplier, 
                Constants.Drive.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE)));*/
            //SmartDashboard.putNumber("DesiredReef", desiredReef.getX());
            //Commands.deferredProxy(() -> drive.driveToPose(desiredReef)).execute();
            
            /*if (reefDebounce == false) {
                reefDebounce = true;
                drive.driveToPose(desiredReef).execute();
            }*/
            drive.driveToReef(leftReef.getAsBoolean());
        }

        // -- Coral Station --
        else if (coralStationRight.getAsBoolean()) {
            Pose2d desiredCoralStation = Constants.constField.getCoralStationPositions().get().get(0);
            Distance coralStationDistance = Units.Meters
                .of(drive.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

            drive.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
                transMultiplier,
                Constants.Drive.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE);
        }

        else if (coralStationLeft.getAsBoolean()) {
            Pose2d desiredCoralStation = Constants.constField.getCoralStationPositions().get().get(2);
            Distance coralStationDistance = Units.Meters
                .of(drive.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

            drive.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
                transMultiplier,
                Constants.Drive.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE);
        }

        // -- Processors --
        else if (processor.getAsBoolean()) {
            Pose2d desiredProcessor = drive.getDesiredProcessor();
            Distance processorDistance = Units.Meters
                .of(drive.getPose().getTranslation().getDistance(desiredProcessor.getTranslation()));
    
                drive.rotationalAutoAlign(processorDistance, desiredProcessor, xVelocity, yVelocity, rVelocity,
                transMultiplier,
                Constants.Drive.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE);
        }

        if (resetPoseWithLimelight.getAsBoolean()) {
            drive.resetPoseWithAprilTag();
        }

        else {
            reefDebounce = false;
            drive.driveFieldOriented(chassisSpeeds).execute();
        }
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
