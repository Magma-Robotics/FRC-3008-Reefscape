package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaePivot extends SubsystemBase {
    private SparkFlexConfig algaePivotConfig = new SparkFlexConfig();

    private SparkFlex algaePivot = 
        new SparkFlex(Constants.CANIds.kAlgaePivotID, MotorType.kBrushless);
    private SparkClosedLoopController algaePivotController = algaePivot.getClosedLoopController();
    private RelativeEncoder algaePivotEncoder = algaePivot.getEncoder();
    public static double kAlgaePivotP = 0;
    public static double kAlgaePivotI = 0;
    public static double kAlgaePivotD = 0;
    public static double algaePivotSetpoint = 0;

    //Target
    public double algaePivotTarget = 0;

    public AlgaePivot() {
        //create configs
        algaePivotConfig
            .inverted(true)
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
            .positionConversionFactor(Constants.Algae.kAlgaePivotRotationsToDeg);

        algaePivot.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reachAlgaePivotAngle(double target) {
        algaePivotController.setReference(target, ControlType.kMAXMotionPositionControl);
    }

    public Command setAlgaePivotAngle(double target) {
        return run(() -> reachAlgaePivotAngle(target));
    }

    public Command algaePivotDown() {
        return run(() -> {
            algaePivot.set(0.6);
            /*algaePivotTarget -= 0.1;
            setAlgaePivotAngle(algaePivotTarget);*/
        });
    }

    public Command algaePivotUp() {
        return run(() -> {
            algaePivot.set(-0.6);
            /*algaePivotTarget += 0.1;
            setAlgaePivotAngle(algaePivotTarget);*/
        });
    }

    public Command stopAlgaePivot() {
        return run(() -> algaePivot.set(0)/*() -> setAlgaePivotAngle(algaePivotTarget)*/);
    }

    public void setAlgaePivotState(Constants.RobotStates.AlgaeStates state)  {
        switch(state) {
            case A_STOW:
                //reachAlgaePivotAngle(Constants.Algae.A_STOW_ANGLE);
                algaePivotTarget = Constants.Algae.A_STOW_ANGLE;
                break;
            
            case A_PROCCESSOR:
                //reachAlgaePivotAngle(Constants.Algae.A_PROCCESSOR_ANGLE);
                algaePivotTarget = Constants.Algae.A_PROCCESSOR_ANGLE;
                break;

            case A_LOAD:
                //reachAlgaePivotAngle(Constants.Algae.A_LOADING_ANGLE);
                algaePivotTarget = Constants.Algae.A_LOADING_ANGLE;
                break;
        }
        setAlgaePivotAngle(algaePivotTarget);
    }

    public double getAlgaePivotPos() {
        return algaePivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePivot Position", algaePivotEncoder.getPosition());
        SmartDashboard.putNumber("AlgaePivotSetpoint", algaePivotSetpoint);

        //reachAlgaePivotAngle(algaePivotSetpoint);
    }
    
}
