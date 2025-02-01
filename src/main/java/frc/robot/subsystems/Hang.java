package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hang extends SubsystemBase {
    private SparkFlexConfig hangConfig = new SparkFlexConfig();
    private SparkFlex hang = new SparkFlex(Constants.CANIds.kHangID, MotorType.kBrushless);
    
    public Hang() {
        hangConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        hang.configure(hangConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command hangUp() {
        return runOnce(() -> hang.set(1));
    }

    public Command hangDown() {
        return runOnce(() -> hang.set(-1));
    }

    public Command stopHang() {
        return runOnce(() -> hang.set(0));
    }

}
