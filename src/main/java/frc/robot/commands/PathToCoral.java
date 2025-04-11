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

import java.util.List;
import java.util.Vector;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.Island;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

public class PathToCoral extends Command {
    //SwerveSubsystem
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight");

    //State tracking variables
    private int coralPositionID = 18;
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
    private double decelerationDist = 0.75;
    private double angleDecelerationDistance = 20.0;
    private double minAngleSpeed = 0.25;
    private double minSpeed = 0.25;


    //Pathplanning variables
    private List<PathPoint> pathPoints;
    private double enRouteTolarance = 0.3;
    private double finalTolerance = 0.1;
    private double rotationTarget = 0.0;
    private double rotationTolerance = 3.0;
    private Pose2d finalGoalPose;

    public PathToCoral(SwerveSubsystem swerve, Pose2d goalPose) {
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
        completeFlag = false;
        stepNum = -1;
        /*
        integralErrorX = 0.0;
        integralErrorY = 0.0;
        derivativeErrorX = 0.0;
        derivativeErrorY = 0.0;
        */
        //finalGoalPose = getFinalGoalPose(coralPositionID);
        pathPoints = ppPlan(finalGoalPose).getAllPathPoints();
        rotationTarget = pathPoints.get(pathPoints.size() - 1).rotationTarget.rotation().getDegrees();
        SmartDashboard.putNumber("NumSteps", pathPoints.size());
        isLastPoint = false;
        updateSubGoals();
        lastExecuteTime = Timer.getTimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
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
            swerveSubsystem.drive(moveStep);
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

        /* 
        double speedFactor;
        if(distToFinal <= decelerationDist){ //Clamp function could replcace this all
            speedFactor = (1 - (decelerationDist - distToFinal));
            if(speedFactor < minSpeed){
                speedFactor = minSpeed;
            }
        }
        else{
            speedFactor = 1.0;
        }
        */

        Translation2d cheat = translationError.times(Constants.ALIGN_MAX_SPEED * speedFactor);

        /*
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
        */

        //---Rotation---//
        double rotationError = rotationTarget - swerveSubsystem.getSwerveDrive().getYaw().getDegrees();
        SmartDashboard.putNumber("Rotation Error", rotationError);

        //Testing only
        double rotationSpeedFactor = MathUtil.clamp(rotationError, -angleDecelerationDistance, angleDecelerationDistance);
        rotationSpeedFactor = Math.abs(rotationSpeedFactor/angleDecelerationDistance);
        rotationSpeedFactor = Math.max(rotationSpeedFactor, minAngleSpeed);

        if(rotationError >= rotationTolerance){
            moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * rotationSpeedFactor * rotationError * 0.01;
        }
        else if(rotationError <= -rotationTolerance){
            moveStep.omegaRadiansPerSecond = Constants.MAX_ANGV * rotationSpeedFactor * rotationError * 0.01;
        }
        else{
            moveStep.omegaRadiansPerSecond = 0.0;
        }
        //moveStep.vxMetersPerSecond = finalMovementVector.getX();
        //moveStep.vyMetersPerSecond = finalMovementVector.getY();
        moveStep.vxMetersPerSecond = cheat.getX();
        moveStep.vyMetersPerSecond = cheat.getY();

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

    ///Returns the final pose position based off of pose defined in the Constants
    /*private Pose2d getFinalGoalPose(int goalSelect){
        Pose2d goalPose;
        switch(goalSelect){
            case 18:
                goalPose = Constants.CoralAlignmentCoords.blueFarLeftL;
                break;
            default:
                goalPose = Constants.CoralAlignmentCoords.blueFarLeftR;
                break;
        }
        return goalPose;
    }*/

    private PathPlannerPath ppPlan(Pose2d finalGoal){
        //Get current position with either camera reading or odometry estimate
        PoseEstimate initPose = visionSubsystem.GetVisionEstimate();
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
