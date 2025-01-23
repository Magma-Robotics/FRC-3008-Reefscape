package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
//FIRST Library imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Limelight library imports
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

//NavX library imports
import com.studica.frc.AHRS;

//Base java imports
import java.util.concurrent.locks.ReentrantLock;

/*
 * This subsystem handles using the limelight camera to update the robot's estimated
 * position on the field
 * 
 * Systems/subsystems that want to use the information should run the "GetLLPos()" function
 * to safely access the data
 * 
 * The periodic function is what is used to upate the estimated position using the limelight
 */
public class VisionSubsystem extends SubsystemBase{

    //Field-oriented pose estimate:
    private LimelightHelpers.PoseEstimate m_FieldPose;
    double m_LLTimestamp;

    //Memory lock to prevent race conditions
    private ReentrantLock lock = new ReentrantLock();

    private final String LL_NAME;

    LimelightHelpers.PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    //Gyro
    private AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

    //Constructor
    public VisionSubsystem(String LimelightName){
        LL_NAME = LimelightName;
        LimelightHelpers.setPipelineIndex(LL_NAME, 0);
        navX.zeroYaw();
    }
    
    //On the periodic cycle (20ms), update the field pose estimate
    @Override
    public void periodic() {
        double heading = navX.getAngle();
        SmartDashboard.putNumber("Heading", heading);
        LimelightHelpers.SetRobotOrientation(LL_NAME, heading+180, 0, 0, 0, 0, 0);
        
        lock.lock();
        m_FieldPose = newEstimate;
        lock.unlock();
        if(newEstimate == null){
            SmartDashboard.putString("LL Status", "No estimate!");
            return;
        }
        else{
            SmartDashboard.putString("LL Status", "Generating estimates...");
        }
        //Code below is for data display purposes only
        SmartDashboard.putNumber("VisionPose_X", newEstimate.pose.getX());
        SmartDashboard.putNumber("VisionPose_Y", newEstimate.pose.getY());
        SmartDashboard.putNumber("Number of detected tags", newEstimate.rawFiducials.length);
        String detectedTagIDs = new String();
        for(int i = 0; i < newEstimate.rawFiducials.length; i++){
            detectedTagIDs = detectedTagIDs + ", " + String.valueOf(newEstimate.rawFiducials[i].id);
            if(newEstimate.rawFiducials[i].id == 1){
                LimelightHelpers.RawFiducial dudTag = newEstimate.rawFiducials[i];
                SmartDashboard.putNumber("Dud tag horizontal offset", dudTag.txnc);
                SmartDashboard.putNumber("Dud tag vertical offset", dudTag.tync);
            }
        }
        //SmartDashboard.putNumberArray("Detected Tag IDs", detectedTagIDs);
        SmartDashboard.putString("Detected tag IDs", detectedTagIDs);
    }

    //Use this function to fetch the current pose of the robot estimated by limelight
    public LimelightHelpers.PoseEstimate GetVisionEstimate(){
        LimelightHelpers.PoseEstimate updatedEstimate;
        lock.lock();
        if(m_FieldPose == null){
            updatedEstimate = new LimelightHelpers.PoseEstimate();
        }
        updatedEstimate = m_FieldPose;
        lock.unlock();
        return updatedEstimate;
    }
}
