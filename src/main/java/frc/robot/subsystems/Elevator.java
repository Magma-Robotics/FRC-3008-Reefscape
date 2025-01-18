package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax leftElevator, rightElevator;
    private SparkMaxConfig leftElevatorConfig, rightElevatorConfig = new SparkMaxConfig();

    public Elevator() {
        leftElevator = new SparkMax(Constants.CANIds.kLeftElevatorID, MotorType.kBrushless);
        rightElevator = new SparkMax(Constants.CANIds.kRightElevatorID, MotorType.kBrushless);

        leftElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        rightElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        leftElevator.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevator.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
