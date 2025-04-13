package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveSubsystem;

public class PathToCoralPID extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private int stepNum = 0;
    private boolean isLastPoint = false;
    private boolean completeFlag = false;
    private double lastExecuteTime = 0.0;

    private double timeStep = 0.0;

    private List<PathPoint> pathPoints;
    private double rotationTarget = 0.0;
    private Pose2d finalGoalPose;

    private final HolonomicDriveController autoAlignController = Constants.Drive.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER;
    
    public PathToCoralPID(SwerveSubsystem swerveSubsystem, Pose2d goalPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.finalGoalPose = goalPose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Initialize the command
        pathPoints = ppPlan(finalGoalPose).getAllPathPoints();
        rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
        SmartDashboard.putNumber("NumSteps", pathPoints.size());
        isLastPoint = false;
        updateSubGoals();
        lastExecuteTime = Timer.getTimestamp();
    }
    
    @Override
    public void execute() {
        SmartDashboard.putData(swerveSubsystem);
        //Calculate time since last execute
        timeStep = Timer.getTimestamp() - lastExecuteTime;
        lastExecuteTime = Timer.getTimestamp();
        //Get variables needed for path planning
        PathPoint currentGoalPoint = pathPoints.get(stepNum);
        Pose2d currentPose = getCurrentPos();
        SmartDashboard.putNumber("Goal X", currentGoalPoint.position.getX());
        SmartDashboard.putNumber("Goal Y", currentGoalPoint.position.getY());
        SmartDashboard.putNumber("StepNumber", stepNum);
    }

    @Override
    public boolean isFinished() {
        return completeFlag;
    }

    @Override
    public void end(boolean interrupted) {
        /*Trajectory trajectory = tra
        Trajectory.State traj = 
        // Clean up after the command ends
        swerveSubsystem.drive(autoAlignController.calculate(getCurrentPos(), finalGoalPose, finalGoalPose.getRotation()));*/
    }

    private PathPlannerPath ppPlan(Pose2d finalGoal){
        //Get current position with either camera reading or odometry estimate
        PoseEstimate initPose = swerveSubsystem.visionSubsystem.GetVisionEstimate();
        Pose2d currentPose;
        if(initPose.tagCount <= 0){
             currentPose = swerveSubsystem.getPose();
        }
        else{
            currentPose = initPose.pose;
        }
        //swerveSubsystem.updateOdomWithCamera(); //Tell swerve to update its odometry with camera reading

        List<Waypoint> pathPoints = PathPlannerPath.waypointsFromPoses(currentPose, finalGoal);
        PathConstraints constraints = new PathConstraints(Constants.MAX_SPEED, Constants.MAX_ACCEL, Constants.MAX_ANGV, Constants.MAX_ANGA);
        double currentVelocity = Math.sqrt(Math.pow(swerveSubsystem.getFieldVelocity().vxMetersPerSecond, 2) + Math.pow(swerveSubsystem.getFieldVelocity().vyMetersPerSecond, 2));
        IdealStartingState currentState = new IdealStartingState(currentVelocity, swerveSubsystem.getHeading());
        PathPlannerPath pathToCoral = new PathPlannerPath(pathPoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(finalGoal.getRotation().getDegrees())));
        return pathToCoral;
    }

    private Pose2d getCurrentPos(){
        PoseEstimate vision = swerveSubsystem.visionSubsystem.GetVisionEstimate();
        if(vision.tagCount == 0){
            return swerveSubsystem.getPose();
        }
        return vision.pose;
    }

    private void updateSubGoals(){
        stepNum++;
        if(stepNum == pathPoints.size() - 1){
            isLastPoint = true;
        }
    }

    
    
}
