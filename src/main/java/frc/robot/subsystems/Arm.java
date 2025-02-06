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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;
import frc.robot.Constants.RobotStates.CoralStates;

public class Arm extends SubsystemBase {

    private SparkFlexConfig armPivotConfig, wristConfig = new SparkFlexConfig();
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();

    //initialize arm pivot
    private SparkFlex armPivot = 
        new SparkFlex(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
    private SparkClosedLoopController armPivotController = armPivot.getClosedLoopController();
    private RelativeEncoder armPivotEncoder = armPivot.getEncoder();
    public static double kArmP = 0;
    public static double kArmI = 0;
    public static double kArmD = 0;
    public static double armAngleSetPoint = 0;

    //initialize intake pivot
    private SparkFlex wrist = 
        new SparkFlex(Constants.CANIds.kWristID, MotorType.kBrushless);
    private SparkClosedLoopController wristController = armPivot.getClosedLoopController();
    private RelativeEncoder wristEncoder = wrist.getEncoder();
    public static double kWristP = 0;
    public static double kWristI = 0;
    public static double kWristD = 0;
    public static double wristAngleSetPoint = 0;

    //initialize intake motor
    private SparkMax intake = 
        new SparkMax(Constants.CANIds.kCoralIntakeID, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = intake.getEncoder();

    public Arm() {
        //create configs
        armPivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        armPivotConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kArmP, kArmI, kArmD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(10)
            .maxAcceleration(5)
            .allowedClosedLoopError(5);
        armPivotConfig
            .encoder
            .positionConversionFactor(Constants.Arm.ArmAngleConversionFactor);

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
            .maxVelocity(10)
            .maxAcceleration(5)
            .allowedClosedLoopError(5);
        wristConfig
            .encoder
            .positionConversionFactor(Constants.Wrist.WristAngleConversionFactor);

        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        //set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intakeCoral() {
        return runOnce(() -> intake.set(1));
    }

    public Command outtakeCoral() {
        return runOnce(() -> intake.set(-1));
    }

    public Command stopIntake() {
        return runOnce(() -> intake.set(0));
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

            case C_LOAD:
                reachArmPivotTarget(Constants.Arm.C_LOADING_ANGLE);
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
                reachArmPivotTarget(Wrist.C_LOADING_ANGLE);
                break;
        }
    }

    public void reachWristTarget(double target) {
        wristController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setWristTarget(double target) {
        return run(() -> reachWristTarget(target));
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armPivotEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());

        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
        SmartDashboard.putNumber("ArmAngleSetpoint", armAngleSetPoint);

        SmartDashboard.putNumber("kWristP", kWristP);
        SmartDashboard.putNumber("kWristI", kWristI);
        SmartDashboard.putNumber("kWristD", kWristD);
        SmartDashboard.putNumber("WristAngleSetpoint", wristAngleSetPoint);

        reachArmPivotTarget(armAngleSetPoint);
        //reachWristTarget(wristAngleSetPoint);
    }
}
