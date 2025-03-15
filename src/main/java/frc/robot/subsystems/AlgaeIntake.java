package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

    private SparkMax algaeIntake = 
        new SparkMax(Constants.CANIds.kAlgaeIntakeID, MotorType.kBrushless);
    private RelativeEncoder algaeIntakeEncoder = algaeIntake.getEncoder();

    public AlgaeIntake() {
        algaeIntakeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        //set configs for motors
        algaeIntake.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intakeAlgae() {
        return run(() -> algaeIntake.set(Constants.Algae.intakeSpeed));
    }

    public Command outtakeAlgae() {
        return run(() -> algaeIntake.set(-Constants.Algae.intakeSpeed));
    }

    public Command stopAlgaeIntake() {
        return run(() -> algaeIntake.set(0));
    }
}
