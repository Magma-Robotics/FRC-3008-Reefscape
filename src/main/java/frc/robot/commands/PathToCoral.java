package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PathToCoral extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight");

    //State tracking variables
    private Pose2d goalPose = Pose2d.kZero;
    private int stepNum = 0;
    private boolean isLastPoint = false;
    private boolean completeFlag = false;
    private double lastExecuteTime = 0.0;

    //Movement variables
    private double timeStep = 0.0;
    private double kp = 1.0;
    private double ki = 0.3;
    private double kd = 0.0;
    private double integralErrorX = 0;
    private double integralErrorY = 0;
    private double derivativeErrorX = 0;
    private double derivativeErrorY = 0;

    //Pathplanning variables
    private List<PathPoint> pathPoints;
    private double enRouteTolarance = 0.2;
    private double finalTolerance = 0.05;
    private double rotationTarget = 0.0;
    private double rotationTolerance = 1.0;

    public PathToCoral(SwerveSubsystem swerveSubsystem, Pose2d goalPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.goalPose = goalPose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        completeFlag = false;
        pathPoints = getPath().getAllPathPoints();
        rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
        SmartDashboard.putNumber("NumSteps", pathPoints.size());
        isLastPoint = false;
        lastExecuteTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        //Calculate time since last execute
        timeStep = Timer.getTimestamp() - lastExecuteTime;
        lastExecuteTime = Timer.getTimestamp();
        //Get variables needed for path planning
        PathPoint currentGoalPoint = pathPoints.get(stepNum);
        Pose2d currentPose = getCurrentPos();
        SmartDashboard.putNumber("Goal X", currentGoalPoint.position.getX());
        SmartDashboard.putNumber("Goal Y", currentGoalPoint.position.getY());
        SmartDashboard.putNumber("StepNumber", stepNum);
        //--Calculate move step--//
        //Determine if this is the last point in the path (Optimize this!) 
        ChassisSpeeds moveStep = calculateMoveStep(currentGoalPoint, currentPose);
        if(moveStep == null){
            if(isLastPoint){
                completeFlag = true;
                return;
            }
            updateSubGoals();
            moveStep = calculateMoveStep(currentGoalPoint, currentPose);
        }
        //--Ececute move step--//
        if(moveStep != null){
            SmartDashboard.putNumber("CommandedMoveStepX", moveStep.vxMetersPerSecond);
            SmartDashboard.putNumber("CommandedMoveStepY", moveStep.vyMetersPerSecond);
            swerveSubsystem.drive(moveStep);
        }
        //swerveSubsystem.drive(moveStep);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return completeFlag;
    }

    private ChassisSpeeds calculateMoveStep(PathPoint goalPoint, Pose2d currentPose){
        //Determine if are at the end of path or at a path node
        double distToGoal = goalPoint.position.getDistance(currentPose.getTranslation());
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
        Translation2d cheat = translationError.times(Constants.MAX_SPEED);
        /* 
        if ((pathPoints.get(pathPoints.size() - 1).distanceAlongPath - 
            (goalPoint.distanceAlongPath - currentPose.getTranslation().getDistance(goalPoint.position))) < 0.1) {
            cheat = translationError.times(Constants.MAX_SPEED/4);
        }*/
        if(distToGoal < 0.25 && isLastPoint){
            cheat = translationError.times(Constants.MAX_SPEED);//translationError.div(8);
        }

        //Calculate X
        double proportionalError = kp * translationError.getX();
        integralErrorX += translationError.getX();
        double integralError = ki * integralErrorX;
        double derivativeError = kd * (translationError.getX() - derivativeErrorX);
        derivativeErrorX = translationError.getX();
        double finalX = proportionalError + integralError + derivativeError;

        //Calculate Y
        proportionalError = kp * translationError.getY();
        integralErrorY += translationError.getY();
        integralError = ki * integralErrorY;
        derivativeError = kd * (translationError.getY() - derivativeErrorY);
        derivativeErrorY = translationError.getY();
        double finalY = proportionalError + integralError + derivativeError;

        Translation2d finalMovementVector = new Translation2d(finalX, finalY);
        if(finalMovementVector.getNorm() > 1.0){
            finalMovementVector = finalMovementVector.div(finalMovementVector.getNorm());
        }
        finalMovementVector = finalMovementVector.times(Constants.MAX_SPEED);

        //---Rotation---//
        double rotationError = rotationTarget - swerveSubsystem.getSwerveDrive().getYaw().getDegrees();
        SmartDashboard.putNumber("Rotation Error", rotationError);

        //Testing only
        if(rotationError >= rotationTolerance){
            moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * 0.01 * rotationError;
        }
        else if(rotationError <= -rotationTolerance){
            moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * 0.01 * rotationError;
        }
        else{
            moveStep.omegaRadiansPerSecond = 0.0;
        }
        //moveStep.vxMetersPerSecond = finalMovementVector.getX();
        //moveStep.vyMetersPerSecond = finalMovementVector.getY();
        moveStep.vxMetersPerSecond = cheat.getX();
        moveStep.vyMetersPerSecond = cheat.getY();
        SmartDashboard.putNumber("TranslationVectorX", finalMovementVector.getX());
        SmartDashboard.putNumber("TranslationVectorY", finalMovementVector.getY());

        /*
        //Compare desired travel vector to current speeds and apply acceleration as nessessary
        double currentSpeed = Math.hypot(swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                                        swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        //If last point, determine if we can slow down in time
        if(isLastPoint){
            //-(vi^2)/2a = d
            double distToStop = Math.pow(currentSpeed, 2.0)/(2*Constants.MAX_ACCEL);
            if(distToStop <= distToGoal){
                moveStep.vxMetersPerSecond = swerveSubsystem.getFieldVelocity().vxMetersPerSecond - (Constants.MAX_ACCEL * timeStep);
                moveStep.vyMetersPerSecond = swerveSubsystem.getFieldVelocity().vyMetersPerSecond - (Constants.MAX_ACCEL * timeStep);
            }
        }*/

        

        return moveStep;
    }

    private PathPlannerPath getPath() {
        //Get current position with either camera reading or odometry estimate
        PoseEstimate initPose = visionSubsystem.GetVisionEstimate();
        Pose2d currentPose;
        if(initPose.tagCount <= 0){
             currentPose = swerveSubsystem.getPose();
        }
        else{
            currentPose = initPose.pose;
            swerveSubsystem.resetOdometry(new Pose2d(currentPose.getX(), currentPose.getY(), swerveSubsystem.getHeading()));
        }

        List<Waypoint> pathPoints = PathPlannerPath.waypointsFromPoses(currentPose, goalPose);
        PathConstraints constraints = new PathConstraints(1, Constants.MAX_ACCEL, Constants.MAX_ANGV, Constants.MAX_ANGA);
        double currentVelocity = Math.sqrt(Math.pow(swerveSubsystem.getFieldVelocity().vxMetersPerSecond, 2) + 
                                 Math.pow(swerveSubsystem.getFieldVelocity().vyMetersPerSecond, 2));
        IdealStartingState currentState = new IdealStartingState(currentVelocity, swerveSubsystem.getHeading());
        PathPlannerPath pathToCoral = new PathPlannerPath(pathPoints, constraints, currentState, 
            new GoalEndState(0.0, Rotation2d.fromDegrees(goalPose.getRotation().getDegrees())));
        return pathToCoral;
    }

    private Pose2d getCurrentPos(){
        PoseEstimate vision = visionSubsystem.GetVisionEstimate();
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
