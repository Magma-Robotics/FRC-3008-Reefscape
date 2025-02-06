package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkFlexConfig algaePivotConfig = new SparkFlexConfig();
    private SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
    
    private SparkFlex algaePivot = 
        new SparkFlex(Constants.CANIds.kAlgaePivotID, MotorType.kBrushless);
    private SparkClosedLoopController algaePivotController = algaePivot.getClosedLoopController();
    private RelativeEncoder algaePivotEncoder = algaePivot.getEncoder();
    public static double kAlgaePivotP = 0;
    public static double kAlgaePivotI = 0;
    public static double kAlgaePivotD = 0;
    public static double algaePivotSetpoint = 0;

     private SparkMax algaeIntake = 
        new SparkMax(Constants.CANIds.kAlgaeIntakeID, MotorType.kBrushless);
    private RelativeEncoder algaeIntakeEncoder = algaeIntake.getEncoder();

    public AlgaeSubsystem() {
        //create configs
        algaePivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        algaePivotConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kAlgaePivotP, kAlgaePivotI, kAlgaePivotD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(10)
            .maxAcceleration(5)
            .allowedClosedLoopError(5);
        algaePivotConfig
            .encoder
            .positionConversionFactor(Constants.Algae.AlgaePivotAngleConversionFactor);

        algaeIntakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        //set configs for motors
        algaePivot.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntake.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reachAlgaePivotAngle(double target) {
        algaePivotController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setAlgaePivotAngle(double target) {
        return run(() -> reachAlgaePivotAngle(target));
    }

    public void setAlgaePivotState(Constants.RobotStates.AlgaeStates state)  {
        switch(state) {
            case A_STOW:
                reachAlgaePivotAngle(Constants.Algae.A_STOW_ANGLE);
                break;
            
            case A_PROCCESSOR:
                reachAlgaePivotAngle(Constants.Algae.A_PROCCESSOR_ANGLE);
                break;

            case A_LOAD:
                reachAlgaePivotAngle(Constants.Algae.A_LOADING_ANGLE);
                break;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePivot Position", algaePivotEncoder.getPosition());

        SmartDashboard.putNumber("kAlgaePivotP", kAlgaePivotP);
        SmartDashboard.putNumber("kAlgaePivotI", kAlgaePivotI);
        SmartDashboard.putNumber("kAlgaePivotD", kAlgaePivotD);

        SmartDashboard.putNumber("AlgaePivotSetpoint", algaePivotSetpoint);
    }

}
