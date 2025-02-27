package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AddVisionMeasurement extends Command {
    SwerveSubsystem drive;
    VisionSubsystem visionSubsystem;

    PoseEstimate estimatedPose;
    double drivetrainRotation = 0;

    public AddVisionMeasurement(SwerveSubsystem drive, VisionSubsystem visionSubsystem) {
        this.drive = drive;
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Tells the limelight where we are on the field
        LimelightHelpers.SetRobotOrientation("limelight",
            drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        AngularVelocity gyroRate = Units.DegreesPerSecond.of(drive.getGyroRate());

        Optional<PoseEstimate> estimatedPose = visionSubsystem.determinePoseEstimate(gyroRate);
        if (estimatedPose.isPresent()) {
            drive.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
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
