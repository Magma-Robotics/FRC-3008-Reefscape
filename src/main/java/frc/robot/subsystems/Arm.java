package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private SparkMax armPivot, intakePivot;
    private SparkMaxConfig armPivotConfig, intakePivotConfig = new SparkMaxConfig();

    public Arm() {
        armPivot = new SparkMax(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
        intakePivot = new SparkMax(Constants.CANIds.kIntakePivotID, MotorType.kBrushless);

        armPivotConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        intakePivotConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
