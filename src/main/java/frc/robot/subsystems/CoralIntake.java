package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private SparkFlexConfig intakeConfig = new SparkFlexConfig();
    //initialize intake motor
    private SparkFlex intake = 
        new SparkFlex(Constants.CANIds.kCoralIntakeID, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = intake.getEncoder();

    public double coralIntakeSpeed = 0;

    public CoralIntake() {
        intakeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("CoralIntakeSpeed", coralIntakeSpeed);
    }
    
    public Command intakeCoral() {
        return runOnce(() -> intake.set(Constants.Wrist.coralIntakeSpeed));
    }

    public Command outtakeCoral() {
        return runOnce(() -> intake.set(-Constants.Wrist.coralIntakeSpeed));
    }

    public Command slowIntakeCoral() {
        return runOnce(() -> intake.set(Constants.Wrist.slowCoralIntakeSpeed));
    }

    public Command slowOuttakeCoral() {
        return runOnce(() -> intake.set(-Constants.Wrist.slowCoralIntakeSpeed));
    }

    public Command stopIntake() {
        return runOnce(() -> intake.set(0));
    }

    public void setIntake(double speed) {
        intake.set(speed);
    }

    public void periodic() {
        if (Constants.Testing.testingCoralIntake) {
            setIntake(SmartDashboard.getNumber("CoralIntakeSpeed", 0));
        }
    }

}
