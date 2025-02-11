package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();
    //initialize intake motor
    private SparkMax intake = 
        new SparkMax(Constants.CANIds.kCoralIntakeID, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = intake.getEncoder();

    public CoralIntake() {
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public Command intakeCoral() {
        return runOnce(() -> intake.set(0.5));
    }

    public Command outtakeCoral() {
        return runOnce(() -> intake.set(-0.5));
    }

    public Command stopIntake() {
        return runOnce(() -> intake.set(0));
    }

}
