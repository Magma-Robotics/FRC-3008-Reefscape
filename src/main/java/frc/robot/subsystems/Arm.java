package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;
import frc.robot.Constants.RobotStates.CoralStates;

public class Arm extends SubsystemBase {
    private SparkFlexConfig armPivotConfig = new SparkFlexConfig();
    private SparkFlexConfig wristConfig = new SparkFlexConfig();

    //initialize arm pivot
    private SparkFlex armPivot = 
        new SparkFlex(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
    private SparkClosedLoopController armPivotController = armPivot.getClosedLoopController();
    private RelativeEncoder armPivotEncoder = armPivot.getEncoder();
    public static double kArmP = 0.1;
    public static double kArmI = 0;
    public static double kArmD = 0;
    public static double armAngleSetPoint = 0;

    //initialize intake pivot
    private SparkFlex wrist = 
        new SparkFlex(Constants.CANIds.kWristID, MotorType.kBrushless);
    private SparkClosedLoopController wristController = wrist.getClosedLoopController();
    private RelativeEncoder wristEncoder = wrist.getEncoder();
    public static double kWristP = 0.1;
    public static double kWristI = 0;
    public static double kWristD = 0;
    public static double wristAngleSetPoint = 0;

    public Arm() {
        //create configs
        armPivotConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        armPivotConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kArmP, kArmI, kArmD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(300000)
            .maxAcceleration(60000)
            .allowedClosedLoopError(1)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        armPivotConfig
            .encoder
            //degrees
            .positionConversionFactor(Constants.Arm.kArmRotationsToDeg)
            //degrees per sec
            .velocityConversionFactor(Constants.Arm.kArmRPMtoDegPerSec);

        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        wristConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kWristP, kWristI, kWristD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(11000)
            .maxAcceleration(4000)
            .allowedClosedLoopError(1)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        wristConfig
            .encoder
            //degrees
            .positionConversionFactor(Constants.Wrist.kWristRotationsToDeg)
            //degrees per sec
            .velocityConversionFactor(360/60);

        //set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("ArmAngleSetpoint", armAngleSetPoint);
        SmartDashboard.putNumber("WristAngleSetpoint", wristAngleSetPoint);
    }

    public void reachArmPivotTarget(double target) {
        armAngleSetPoint = target;
        armPivotController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setArmPivotTarget(double target) {
        armAngleSetPoint = target;
        return run(() -> reachArmPivotTarget(target));
    }

    public Command setManualArm(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            armAngleSetPoint += leftJoystick.getAsDouble();
            wristAngleSetPoint += rightJoystick.getAsDouble();
            reachArmPivotTarget(armAngleSetPoint);
            reachWristTarget(wristAngleSetPoint);
        });
    }

    public void setArmPivotState(Constants.RobotStates.CoralStates state)  {
        switch(state) {
            case C_STOW:
                reachArmPivotTarget(Constants.Arm.C_STOW_ANGLE);
                break;
            
            case C_L1:
                reachArmPivotTarget(Constants.Arm.C_L1_ANGLE);
                break;
            
            case C_L2:
                reachArmPivotTarget(Constants.Arm.C_L2_ANGLE);
                break;
            
            case C_L3:
                reachArmPivotTarget(Constants.Arm.C_L3_ANGLE);
                break;

            case C_L4:
                reachArmPivotTarget(Constants.Arm.C_L4_ANGLE);
                break;

            case C_LOAD:
                reachArmPivotTarget(Constants.Arm.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                reachArmPivotTarget(Constants.Arm.C_GROUND_ANGLE);
                break;
        }
    }

    public void setWristState(CoralStates state) {
        switch(state) {
            case C_STOW:
                reachWristTarget(Wrist.C_STOW_ANGLE);
                break;
            
            case C_L1:
                reachWristTarget(Wrist.C_L1_ANGLE);
                break;
            
            case C_L2:
                reachWristTarget(Wrist.C_L2_ANGLE);
                break;
            
            case C_L3:
                reachWristTarget(Wrist.C_L3_ANGLE);
                break;

            case C_L4:
                reachWristTarget(Wrist.C_L4_ANGLE);
                break;

            case C_LOAD:
                reachWristTarget(Wrist.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                reachWristTarget(Wrist.C_GROUND_ANGLE);
                break;
        }
    }

    public void reachWristTarget(double target) {
        wristAngleSetPoint = target;
        wristController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    /*
    public Command setWristTarget(double target) {
        wristAngleSetPoint = target;
        return run(() -> reachWristTarget(target));
    }*/

    public void resetArmPivotEncoder() {
        armPivotEncoder.setPosition(0);
    }

    public void resetWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public double getArmPivotEncoderPos() {
        return armPivotEncoder.getPosition();
    }

    public double getwristEncoderPos() {
        return wristEncoder.getPosition();
    }

    public Command wristForward() {
        return run(() -> wrist.set(0.5));
    }

    public Command wristBackwards() {
        return run(() -> wrist.set(-0.5));
    }

    public Command stopWrist() {
        return run(() -> wrist.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armPivotEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());

        //reachArmPivotTarget(SmartDashboard.getNumber("ArmAngleSetpoint", 0));
        //reachWristTarget(SmartDashboard.getNumber("WristAngleSetpoint", 0));
    }
}
