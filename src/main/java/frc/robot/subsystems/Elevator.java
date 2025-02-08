package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();
    private SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();

    private SparkFlex leftElevator = new SparkFlex(Constants.CANIds.kLeftElevatorID, MotorType.kBrushless);
    private SparkClosedLoopController elevatorController = leftElevator.getClosedLoopController();
    private RelativeEncoder leftElevatorEncoder = leftElevator.getEncoder();
    public static double kElevatorP = 0;
    public static double kElevatorI = 0;
    public static double kElevatorD = 0;
    public static double elevatorSetpoint = 0;

    private SparkFlex rightElevator = new SparkFlex(Constants.CANIds.kRightElevatorID, MotorType.kBrushless);
    private RelativeEncoder rightElevatorEncoder = rightElevator.getEncoder();

    private ElevatorFeedforward elevatorFeedforward = 
        new ElevatorFeedforward(0, 0, 0);

    public Elevator() {
        //create configs
        leftElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        leftElevatorConfig
            .encoder
            .positionConversionFactor(Constants.Elevator.kElevatorRotationsToInches);
        leftElevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kElevatorP, kElevatorI, kElevatorD)
            .outputRange(-1, 1)
            .maxMotion
            .maxAcceleration(1)
            .maxVelocity(1)
            .allowedClosedLoopError(3);

        rightElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(leftElevator);
        rightElevatorConfig
            .encoder
            .positionConversionFactor(Constants.Elevator.kElevatorRotationsToInches);

        //sets configs
        leftElevator.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevator.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("ElevatorSetpoint", elevatorSetpoint);
    }

    public Command elevatorUp() {
        return runOnce(() -> leftElevator.set(0.6));
    }

    public Command elevatorDown() {
        return runOnce(() -> leftElevator.set(-0.6));
    }

    public Command stopElevator() {
        return runOnce(() -> leftElevator.set(0));
    }

    public void reachElevatorTarget(double target) {
        elevatorController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setElevatorTarget(double target) {
        return run(() -> reachElevatorTarget(target));
    }

    public void setElevatorState(Constants.RobotStates.CoralStates state)  {
        switch(state) {
            case C_STOW:
                reachElevatorTarget(Constants.Elevator.C_STOW_POS);
                break;
            
            case C_L1:
                reachElevatorTarget(Constants.Elevator.C_L1_POS);
                break;
            
            case C_L2:
                reachElevatorTarget(Constants.Elevator.C_L2_POS);
                break;
            
            case C_L3:
                reachElevatorTarget(Constants.Elevator.C_L3_POS);
                break;

            case C_L4:
                reachElevatorTarget(Constants.Elevator.C_L4_POS);
                break;
                
            case C_LOAD:
                reachElevatorTarget(Constants.Elevator.C_LOADING_POS);
                break;
        }
    }

    public double getLeftElevatorEncoderPos() {
        return leftElevatorEncoder.getPosition();
    }

    public void resetLeftElevatorEncoder() {
        leftElevatorEncoder.setPosition(0);
    }

    public Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height, getLeftElevatorEncoderPos(), tolerance));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Pos", getLeftElevatorEncoderPos());

        //reachElevatorTarget(SmartDashboard.getNumber("ElevatorSetpoint", 0));
    }
}
