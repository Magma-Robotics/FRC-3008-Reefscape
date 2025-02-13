package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hang extends SubsystemBase {
    private SparkMaxConfig hangConfig = new SparkMaxConfig();
    private SparkMax hang = new SparkMax(Constants.CANIds.kHangID, MotorType.kBrushless);
    private SparkClosedLoopController hangController = hang.getClosedLoopController();
    
    public Hang() {
        hangConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        hangConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(5)
            .maxAcceleration(5)
            .allowedClosedLoopError(1)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        

        hang.configure(hangConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command hangUp() {
        return runOnce(() -> hang.set(0.5));
    }

    public Command hangDown() {
        return runOnce(() -> hang.set(-0.5));
    }

    public Command stopHang() {
        return runOnce(() -> hang.set(0));
    }

    public void reachHangPivotTarget(double target) {
        hangController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setHangPivotTarget(double target) {
       return run(() -> reachHangPivotTarget(target));
    }

}
