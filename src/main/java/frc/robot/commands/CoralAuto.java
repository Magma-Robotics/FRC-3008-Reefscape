package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;



public class CoralAuto extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private boolean isInit = false;

    public CoralAuto(SwerveSubsystem botSwerve){
        swerveSubsystem = botSwerve;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.zeroGyro();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if(swerveSubsystem.b_IsPositionCameraInitalized){
            isInit = true;
            return;
        }
        swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, Constants.MAX_ANGV/2));
        
    }

    @Override
    public boolean isFinished() {
        return isInit;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
