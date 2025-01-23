package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax leftElevator, rightElevator;
    private SparkMaxConfig leftElevatorConfig, rightElevatorConfig = new SparkMaxConfig();

    private RelativeEncoder leftElevatorEncoder, rightElevatorEncoder;

    private ProfiledPIDController elevatorPIDController =
        new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(5, 5)
        );

    private ElevatorFeedforward elevatorFeedforward = 
        new ElevatorFeedforward(0, 0, 0);

    public Elevator() {
        //set CAN IDs
        leftElevator = new SparkMax(Constants.CANIds.kLeftElevatorID, MotorType.kBrushless);
        rightElevator = new SparkMax(Constants.CANIds.kRightElevatorID, MotorType.kBrushless);

        //create configs
        leftElevatorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        rightElevatorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .follow(leftElevator);

        //sets configs
        leftElevator.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevator.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //gets encoders
        leftElevatorEncoder = leftElevator.getEncoder();
        rightElevatorEncoder = rightElevator.getEncoder();
    }

    public void getLeftElevatorEncoderPos() {
        leftElevatorEncoder.getPosition();
    }

    public void resetLeftElevatorEncoder() {
        leftElevatorEncoder.setPosition(0);
    }

    public void stopElevator() {
        leftElevator.set(0);
    }

    //sets target and runs elevator motor
    public Command setElevatorTarget(double target) {
        return runOnce(
            () -> {
                elevatorPIDController.setGoal(target);
            }
        )
        .andThen(
            run(
                () -> {
                    leftElevator.setVoltage(
                        elevatorPIDController.calculate(leftElevatorEncoder.getPosition()) 
                        + elevatorFeedforward.calculate(elevatorPIDController.getSetpoint().velocity)
                    );
                }
            )
        );
    }
}
