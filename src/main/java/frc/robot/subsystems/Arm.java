package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private SparkMax armPivot, intakePivot, intake;
    private SparkMaxConfig armPivotConfig, intakePivotConfig, intakeConfig = new SparkMaxConfig();
    private RelativeEncoder armPivotEncoder, intakePivotEncoder;
    private ProfiledPIDController armPivotPIDController =
        new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(5, 5)
        );
    
    private ProfiledPIDController intakePivotPIDController =
        new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(5, 5)
        );

    private ArmFeedforward armFeedforward = 
        new ArmFeedforward(0, 0, 0);

    private ArmFeedforward intakeFeedforward = 
        new ArmFeedforward(0, 0, 0);
    

    public Arm() {
        //set CAN IDs
        armPivot = new SparkMax(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
        intakePivot = new SparkMax(Constants.CANIds.kIntakePivotID, MotorType.kBrushless);

        //create configs
        armPivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakePivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        //set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //gets encoders
        armPivotEncoder = armPivot.getEncoder();
        intakePivotEncoder = intakePivot.getEncoder();
    }

    public void stopArmPivot() {
        armPivot.set(0);
    }

    public void stopIntakePivot() {
        intakePivot.set(0);
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

    public void resetIntakePivotEncoder() {
        intakePivotEncoder.setPosition(0);
    }

    public void getArmPivotEncoderPos() {
        armPivotEncoder.getPosition();
    }

    public void getIntakePivotEncoderPos() {
        intakePivotEncoder.getPosition();
    }

    //sets target and runs arm pivot motor
    public Command setArmPivotTarget(double target) {
        return runOnce(
            () -> {
                armPivotPIDController.setGoal(target);
            }
        )
        .andThen(
            run(
                () -> {
                    //need to program a way to calculate angle of arm
                    armPivot.setVoltage(
                        armPivotPIDController.calculate(armPivotEncoder.getPosition()) 
                        + armFeedforward.calculate(0, armPivotPIDController.getSetpoint().velocity)
                    );
                }
            )
        );
    }

    //sets target and runs intake pivot motor
    public Command setIntakePivotTarget(double target) {
        return runOnce(
            () -> {
                intakePivotPIDController.setGoal(target);
            }
        )
        .andThen(
            run(
                () -> {
                    armPivot.setVoltage(
                        intakePivotPIDController.calculate(intakePivotEncoder.getPosition()) 
                        + intakeFeedforward.calculate(0, armPivotPIDController.getSetpoint().velocity)
                    );
                }
            )
        );
    }

}
