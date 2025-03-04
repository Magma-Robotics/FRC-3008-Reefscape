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
    private double hangPivotSetpoint = 0;
    
    public Hang() {
        hangConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        hangConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0, 0)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(5)
            .maxAcceleration(5)
            .allowedClosedLoopError(1)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        

        hang.configure(hangConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command hangUp() {
        return runOnce(() -> hang.set(Constants.Hang.hangSpeed));
        /*return run(() -> {
            if (hangPivotSetpoint > Constants.Hang.maxHangAngle) {
                hangPivotSetpoint -= 0.1;
            }
            reachHangPivotTarget(hangPivotSetpoint);
       });*/
    }

    public Command hangDown() {
        return runOnce(() -> hang.set(-Constants.Hang.hangSpeed));
        /*return run(() -> {
            if (hangPivotSetpoint > Constants.Hang.minHangAngle) {
                hangPivotSetpoint += 0.1;
            }
            reachHangPivotTarget(hangPivotSetpoint);
           });*/
    }

    public Command stopHang() {
        return runOnce(() -> hang.set(0));
        /*return run(() -> {
            reachHangPivotTarget(hangPivotSetpoint);
           });*/
    }

    public void reachHangPivotTarget(double target) {
        hangPivotSetpoint = target;
        hangController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setHangPivotTarget(double target) {
        hangPivotSetpoint = target;
       return run(() -> reachHangPivotTarget(target));
    }

}
