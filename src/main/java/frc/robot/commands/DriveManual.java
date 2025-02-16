package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveManual extends Command {
    SwerveSubsystem drive;
    DoubleSupplier xAxis, yAxis, rotationAxis;
    BooleanSupplier slowMode, leftReef, rightReef, leftCoralStationNear, rightCoralStationNear, leftCoralStationFar,
    rightCoralStationFar;
    double slowMultiplier = 0;
    SwerveInputStream driveAngularVelocity;

    public DriveManual(SwerveSubsystem drive, DoubleSupplier xAxis, DoubleSupplier yAxis, 
            DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef) {
        this.drive = drive;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.rotationAxis = rotationAxis;
        this.slowMode = slowMode;
        this.leftReef = leftReef;
        this.rightReef = rightReef;

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

        driveAngularVelocity = SwerveInputStream.of(drive.getSwerveDrive(),
                                                                () -> yAxis.getAsDouble() * slowMultiplier,
                                                                () -> xAxis.getAsDouble() * slowMultiplier)
                                                            .withControllerRotationAxis(() -> -rotationAxis.getAsDouble() * slowMultiplier)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

        if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
            Pose2d desiredReef = drive.getDesiredReef(leftReef.getAsBoolean());
            Distance reefDistance = Meters.of(drive.getPose().getTranslation().getDistance(desiredReef.getTranslation()));

            if (!reefDistance.gte(Constants.Drive.MAX_AUTO_DRIVE_REEF_DISTANCE)) {
                drive.driveToPose(desiredReef);
            }
        }

        else {
            drive.driveFieldOriented(driveAngularVelocity);
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
