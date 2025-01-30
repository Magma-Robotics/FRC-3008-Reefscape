package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private SparkFlexConfig armPivotConfig, wristConfig = new SparkFlexConfig();
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();

    //initialize arm pivot
    private SparkFlex armPivot = 
        new SparkFlex(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
    private SparkClosedLoopController armPivotController = armPivot.getClosedLoopController();
    private RelativeEncoder armPivotEncoder = armPivot.getEncoder();

    //initialize intake pivot
    private SparkFlex wrist = 
        new SparkFlex(Constants.CANIds.kWristID, MotorType.kBrushless);
    private SparkClosedLoopController wristController = armPivot.getClosedLoopController();
    private RelativeEncoder wristEncoder = armPivot.getEncoder();

    //initialize intake motor
    private SparkMax intake = 
        new SparkMax(0, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = armPivot.getEncoder();

    public Arm() {
        //create configs
        armPivotConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        armPivotConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(0)
            .maxAcceleration(0)
            .allowedClosedLoopError(0);
        armPivotConfig
            .encoder
            .positionConversionFactor(Constants.Arm.ArmAngleConversionFactor);

        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        wristConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(0)
            .maxAcceleration(0)
            .allowedClosedLoopError(0);
        wristConfig
            .encoder
            .positionConversionFactor(Constants.Arm.WristAngleConversionFactor);

        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        //set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reachArmPivotTarget(double target) {
        armPivotController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setArmPivotTarget(double target) {
        return run(() -> reachArmPivotTarget(target));
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
        }
    }

    public void reachWristTarget(double target) {
        wristController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setWristTarget(double target) {
        return run(() -> reachWristTarget(target));
    }

    public void stopArmPivot() {
        armPivot.set(0);
    }

    public void stopWrist() {
        wrist.set(0);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void intake() {
        intake.set(0.4);
    }

    public void outtake() {
        intake.set(-0.4);
    }

    public void resetArmPivotEncoder() {
        armPivotEncoder.setPosition(0);
    }

    public void resetWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public void getArmPivotEncoderPos() {
        armPivotEncoder.getPosition();
    }

    public void getwristEncoderPos() {
        wristEncoder.getPosition();
    }
}
