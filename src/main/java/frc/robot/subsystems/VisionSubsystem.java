package frc.robot.subsystems;

//FIRST Library imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
//Limelight library imports
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

//NavX library imports
import com.studica.frc.AHRS;

import java.util.Optional;
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

    //Memory lock to prevent race conditions
    private ReentrantLock lock = new ReentrantLock();

    private final String LL_NAME;

    PoseEstimate lastEstimate = new PoseEstimate();
    boolean newEstimate = false;
    Pose2d pose = new Pose2d();

    private boolean useMegaTag2 = false;
    //Gyro
    private AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

    //Constructor
    public VisionSubsystem(String LimelightName){
        LL_NAME = LimelightName;
        LimelightHelpers.setPipelineIndex(LL_NAME, 0);
        navX.zeroYaw();
    }

    public PoseEstimate getLastPoseEstimate() {
        return lastEstimate;
    }
    
    public void setMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(Units.DegreesPerSecond.of(720)) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > 0.1) {
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

   /**
   * Updates the current pose estimates for the robot using
   * data from Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   *
   *                 This method retrieves pose estimates from the Limelight
   *                 camera and updates the
   *                 pose estimate if it is valid. The method
   *                 supports two modes of operation:
   *                 one using MegaTag2 and one without. The appropriate pose
   *                 estimate retrieval method is chosen
   *                 based on the value of the `useMegaTag2` flag.
   *
   *                 If the retrieved pose estimates are valid and not rejected
   *                 based on the current angular velocity,
   *                 the method updates the last known estimates and sets flags
   *                 indicating new estimates are available.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimate = new PoseEstimate();

    if (useMegaTag2) {
        currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_NAME);
    }
    else {
        currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME);
    }

    if (currentEstimate != null && !rejectUpdate(currentEstimate, gyroRate)) {
      lastEstimate = currentEstimate;
      pose = currentEstimate.pose;
      newEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newEstimate) {
      return Optional.empty();
    } 
    else {
      // valid pose estimate
      newEstimate = false;
      return Optional.of(lastEstimate);
    } 
  }
    
    //On the periodic cycle (20ms), update the field pose estimate
    @Override
    public void periodic() {
        /*double heading = navX.getAngle();
        SmartDashboard.putNumber("Heading", heading);
        LimelightHelpers.SetRobotOrientation(LL_NAME, heading, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_NAME);
        lock.lock();
        m_FieldPose = newEstimate;
        lock.unlock();
        if(newEstimate == null){
            //If here, camera is not ready, do nothing
            SmartDashboard.putString("LL Status", "No estimate!");
            return;
        }
        SmartDashboard.putString(LL_NAME + " status", "Generating estimates...");
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
        SmartDashboard.putString("Detected tag IDs", detectedTagIDs);*/
    }
/*
    //Use this function to fetch the current pose of the robot estimated by limelight
    public LimelightHelpers.PoseEstimate GetVisionEstimate(){
        LimelightHelpers.PoseEstimate updatedEstimate;
        lock.lock();
        updatedEstimate = m_FieldPose;
        lock.unlock();
        return updatedEstimate;
    }*/
    public LimelightHelpers.PoseEstimate GetVisionEstimate() {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME);
    }
}
