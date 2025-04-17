package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.*;

import java.util.Collections;
import java.util.List;
import java.util.Vector;
import java.util.ArrayList;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.Island;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

public class OAPathCoral extends Command {
    //SwerveSubsystem
    private final SwerveSubsystem swerveSubsystem;

    //State tracking variables
    private int coralPositionID = 18;
    private int stepNum = 0;
    private boolean isLastPoint = false;
    private boolean completeFlag = false;

    //Movement variables
    private double decelerationDist = 0.75;
    private double angleDecelerationDistance = 7.0;
    private double minAngleSpeed = 0.5;
    private double minSpeed = 0.25;


    //Pathplanning variables
    private List<PathPoint> pathPoints;
    private double enRouteTolarance = 0.3;
    private double finalTolerance = 0.05;
    private double rotationTarget = 0.0;
    private double rotationTolerance = 3.0;
    private Pose2d finalGoalPose;
    private boolean waitForPathPlan;
    private PathConstraints constraintsForPath;
    private GoalEndState goalStateForpath;
    private double planDecisionDistance = 0.25;

    public OAPathCoral(SwerveSubsystem swerve, Pose2d goalPose) {
        swerveSubsystem = swerve;
        finalGoalPose = goalPose;
        addRequirements(swerve);
    }

    public void setCoralGoal(int id){
        coralPositionID = id;
    }

    public int getCoralPositionGoal(){
        return coralPositionID;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        waitForPathPlan = false;
        completeFlag = false;
        goalStateForpath = null;
        constraintsForPath = null;
        stepNum = 0;
        pathPoints = null;
        PathPlannerPath plannedPath;
        if(isShortPath(finalGoalPose)){
            /* 
            plannedPath = ppPlan(finalGoalPose);
            pathPoints = plannedPath.getAllPathPoints();
            rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
            if(pathPoints.size() == 1){
                isLastPoint = true;
            }
            else{
                isLastPoint = false;
            }*/
            pathPoints = new ArrayList<PathPoint>();
            pathPoints.add(new PathPoint(finalGoalPose.getTranslation()));
            rotationTarget = finalGoalPose.getRotation().getDegrees();
            isLastPoint = true;
        }
        else{
            isLastPoint = false;
            plannedPath = oaPlan(finalGoalPose);
            if(plannedPath == null){
                waitForPathPlan = true;
            }
            else{
                pathPoints = plannedPath.getAllPathPoints();
                rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
                SmartDashboard.putNumber("NumSteps", pathPoints.size());
            }
        }
        swerveSubsystem.updateOdomForPathPlanning();
        //updateSubGoals();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putData(swerveSubsystem);
        if(waitForPathPlan){
            if(Pathfinding.isNewPathAvailable()){
                PathPlannerPath plannedPath = Pathfinding.getCurrentPath(constraintsForPath, goalStateForpath);
                if(plannedPath == null){
                    SmartDashboard.putBoolean("Waiting for path", waitForPathPlan);
                    return;
                }
                pathPoints = plannedPath.getAllPathPoints();
                rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
                SmartDashboard.putNumber("NumSteps", pathPoints.size());
                waitForPathPlan = false;
                SmartDashboard.putBoolean("Waiting for path", waitForPathPlan);
            }
            else{
                return;
            }
        }
        //Get variables needed for path planning
        PathPoint currentGoalPoint = pathPoints.get(stepNum);
        Pose2d currentPose = getCurrentPos();
        SmartDashboard.putNumber("Goal X", currentGoalPoint.position.getX());
        SmartDashboard.putNumber("Goal Y", currentGoalPoint.position.getY());
        SmartDashboard.putNumber("StepNumber", stepNum);
        //Determine if path is valid
        //--Calculate move step--//
        //Determine if this is the last point in the path (Optimize this!) 
        ChassisSpeeds moveStep = calculateMoveStep(currentGoalPoint, currentPose, finalGoalPose);
        if(moveStep == null){
            if(isLastPoint){
                completeFlag = true;
                return;
            }
            updateSubGoals();
            moveStep = calculateMoveStep(currentGoalPoint, currentPose, finalGoalPose);
        }
        //--Ececute move step--//
        if(moveStep != null){
            SmartDashboard.putNumber("CommandedMoveStepX", moveStep.vxMetersPerSecond);
            SmartDashboard.putNumber("CommandedMoveStepY", moveStep.vyMetersPerSecond);
            //swerveSubsystem.drive(moveStep);
            swerveSubsystem.driveFieldOriented(moveStep);
        }
        //swerveSubsystem.drive(moveStep);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            //swerveSubsystem.lock();
        }
        else{
            swerveSubsystem.drive(new ChassisSpeeds());
            //swerveSubsystem.getSwerveDrive().lockPose();
        }
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return completeFlag;
    }

    private ChassisSpeeds calculateMoveStep(PathPoint goalPoint, Pose2d currentPose, Pose2d finalGoal){
        //Determine if are at the end of path or at a path node
        double distToGoal = goalPoint.position.getDistance(currentPose.getTranslation());
        double distToFinal = finalGoalPose.minus(currentPose).getTranslation().getNorm();
        SmartDashboard.putNumber("DistToFinal", distToFinal);
        if(distToGoal <= enRouteTolarance){
            if(!isLastPoint){
                //If here, we are at a path node and within enroute tolerance, move to next path step
                SmartDashboard.putBoolean("PathStepFinished", true);
                return null;
            }
            //If here, we are at the final node, determine final tolerance and rotation
            if(distToGoal <= finalTolerance && 
                (Math.abs(rotationTarget - swerveSubsystem.getSwerveDrive().getYaw().getDegrees()) <= rotationTolerance) ){
                SmartDashboard.putBoolean("PathStepFinished", true);
                return null;
            }
        }
        //End condtions not met, continue moving
        SmartDashboard.putBoolean("PathStepFinished", false);

        ChassisSpeeds moveStep = new ChassisSpeeds();

        //---Translation---//
        //Calculate direction of travel
        //
        Translation2d translationError = goalPoint.position.minus(currentPose.getTranslation()); //Error calculation
        translationError = translationError.div(translationError.getNorm()); //Unit vector

        double speedFactor = MathUtil.clamp(distToFinal, 0.0, decelerationDist);
        speedFactor = speedFactor/decelerationDist;
        speedFactor = Math.max(speedFactor, minSpeed);

        Translation2d cheat = translationError.times(Constants.ALIGN_MAX_SPEED * speedFactor);

        //---Rotation---//
        double rotationError = rotationTarget - (swerveSubsystem.getSwerveDrive().getYaw().getDegrees()%360);
        SmartDashboard.putNumber("Rotation Error", rotationError);

        //Testing only
        double rotationSpeedFactor = MathUtil.clamp(rotationError, -angleDecelerationDistance, angleDecelerationDistance);
        rotationSpeedFactor = Math.abs(rotationSpeedFactor/angleDecelerationDistance);
        rotationSpeedFactor = Math.max(rotationSpeedFactor, minAngleSpeed);

        if(rotationError >= rotationTolerance){
            //moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * rotationSpeedFactor * rotationError * 0.01;
            moveStep.omegaRadiansPerSecond = (Constants.MAX_ANGV/4) * rotationSpeedFactor;
        }
        else if(rotationError <= -rotationTolerance){
            //moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * rotationSpeedFactor * rotationError * 0.01;
            moveStep.omegaRadiansPerSecond = -(Constants.MAX_ANGV/4) * rotationSpeedFactor;
        }
        else{
            moveStep.omegaRadiansPerSecond = 0.0;
        }
        moveStep.vxMetersPerSecond = cheat.getX();
        moveStep.vyMetersPerSecond = cheat.getY();
        
        return moveStep;
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
        //double currentVelocity = Math.sqrt(Math.pow(swerveSubsystem.getFieldVelocity().vxMetersPerSecond, 2) + Math.pow(swerveSubsystem.getFieldVelocity().vyMetersPerSecond, 2));
        //IdealStartingState currentState = new IdealStartingState(currentVelocity, swerveSubsystem.getHeading());
        PathPlannerPath pathToCoral = new PathPlannerPath(pathPoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(finalGoal.getRotation().getDegrees())));

        //Pathfinding.ensureInitialized();
        //Pathfinding.setGoalPosition(finalGoal.getTranslation());
        //Pathfinding.setStartPosition(currentPose.getTranslation());
        return pathToCoral;
    }

    private PathPlannerPath oaPlan(Pose2d finalGoal){
        //Get current position with either camera reading or odometry estimate
        PoseEstimate initPose = swerveSubsystem.visionSubsystem.GetVisionEstimate();
        Pose2d currentPose;
        if(initPose.tagCount <= 0){
            currentPose = swerveSubsystem.getPose();
        }
        else{
            currentPose = initPose.pose;
            swerveSubsystem.resetOdometry(currentPose);
        }
        Pathfinding.ensureInitialized();
        Pathfinding.setStartPosition(currentPose.getTranslation());
        Pathfinding.setGoalPosition(finalGoal.getTranslation());
        constraintsForPath = new PathConstraints(Constants.MAX_SPEED, Constants.MAX_ACCEL, Constants.MAX_ANGV, Constants.MAX_ANGA);
        goalStateForpath = new GoalEndState(0.0, Rotation2d.fromDegrees(finalGoal.getRotation().getDegrees()));
        PathPlannerPath plannedPath = null;
        if(Pathfinding.isNewPathAvailable()){
            plannedPath = Pathfinding.getCurrentPath(constraintsForPath, goalStateForpath);
        }
        return plannedPath;
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

    private boolean isShortPath(Pose2d finalGoalPose){
        Translation2d vectorToFinal = getCurrentPos().getTranslation().minus(finalGoalPose.getTranslation());
        if(vectorToFinal.getNorm() <= planDecisionDistance){
            return true;
        }
        return false;
    }
}
