package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.io.Console;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.RobotStates.CoralStates;

public class Wrist extends SubsystemBase {
    private SparkFlexConfig wristConfig = new SparkFlexConfig();

    public final Trigger atWristMin = new Trigger(() -> getWristAngle().lte(Constants.Wrist.kMinAngle.plus(Degrees.of(5))));
    public final Trigger atWristMax = new Trigger(() -> getWristAngle().gte(Constants.Wrist.kMaxAngle.minus(Degrees.of(5))));
    
    //initialize intake pivot
    private SparkFlex wrist = 
        new SparkFlex(Constants.CANIds.kWristID, MotorType.kBrushless);
    private SparkClosedLoopController wrist2Controller = wrist.getClosedLoopController();
    private RelativeEncoder wristEncoder = wrist.getEncoder();
    public static double kWristP = 0.3;//0.017831;
    public static double kWristI = 0;
    public static double kWristD = 0;
    public static double wristAngleSetPoint = Constants.Wrist.encoderOffset;

    private final ProfiledPIDController wristController;
    private final ArmFeedforward        wristFeedforward = new ArmFeedforward(0.80747,
                                                                              108.24,
                                                                              0.066017,
                                                                              0.012116);

    // SysId Routine and seutp
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage         m_wristAppliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle           m_wristAngle          = Degrees.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_wristVelocity       = DegreesPerSecond.mutable(0);
    // SysID Routine
    private final SysIdRoutine       m_wristSysIdRoutine   =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(6), Seconds.of(30)),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                wrist::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("wrist")
                    .voltage(
                        m_wristAppliedVoltage.mut_replace(wrist.getAppliedOutput() *
                                                        RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_wristAngle.mut_replace(wristEncoder.getPosition(), Degrees))
                    .angularVelocity(m_wristVelocity.mut_replace(wristEncoder.getVelocity(), DegreesPerSecond));
                },
                this));

    public Wrist() {
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        wristConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(kWristP, kWristI, kWristD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(Constants.Wrist.maxWristVelocity)
            .maxAcceleration(Constants.Wrist.maxWristAcceleration)
            .allowedClosedLoopError(0.5)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        wristConfig
            .encoder
            //degrees
            .positionConversionFactor(Constants.Wrist.kWristRotationsToDeg)
            //degrees per sec
            .velocityConversionFactor(360/60);

        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristController = new ProfiledPIDController(kWristP,
                                                kWristI,
                                                kWristD,
                                                new Constraints(Constants.Wrist.maxWristVelocity,
                                                                Constants.Wrist.maxWristAcceleration));
        wristController.setTolerance(0.5);
        wristEncoder.setPosition(90);
        
        SmartDashboard.putNumber("WristAngleSetpoint", wristAngleSetPoint);
    }

    public Trigger atWristAngle(double angle, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(angle, wristEncoder.getPosition(), tolerance));
    }

    public Angle getWristAngle() {
        return Degrees.of(wristEncoder.getPosition());
    }

    public AngularVelocity getWristVelocity() {
        return DegreesPerSecond.of(wristEncoder.getVelocity());
    }

    public Command runWristSysIdRoutine()
    {
        return m_wristSysIdRoutine.dynamic(Direction.kForward).until(atWristMax)
                            .andThen(m_wristSysIdRoutine.dynamic(Direction.kReverse).until(atWristMin))
                            .andThen(m_wristSysIdRoutine.quasistatic(Direction.kForward).until(atWristMax))
                            .andThen(m_wristSysIdRoutine.quasistatic(Direction.kReverse).until(atWristMin));
    }

    public Command setManualWrist(DoubleSupplier joystick) {
        return run(() -> {
            wristAngleSetPoint += joystick.getAsDouble();
            reachWristTarget(wristAngleSetPoint);
        });
    }

    public Command stopPIDWrist() {
        return run(() -> {
            reachWristTarget(wristAngleSetPoint);
        });
    }

    public void setWristState(CoralStates state) {
        switch(state) {
            case C_STOW:
                reachWristTarget(Constants.Wrist.C_STOW_ANGLE);
                break;
            
            case C_L1:
                reachWristTarget(Constants.Wrist.C_L1_ANGLE);
                break;
            
            case C_L2:
                reachWristTarget(Constants.Wrist.C_L2_ANGLE);
                break;
            
            case C_L3:
                reachWristTarget(Constants.Wrist.C_L3_ANGLE);
                break;

            case C_L4:
                reachWristTarget(Constants.Wrist.C_L4_ANGLE);
                break;

            case C_LOAD:
                reachWristTarget(Constants.Wrist.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                reachWristTarget(Constants.Wrist.C_GROUND_ANGLE);
                break;
        }
    }

    public void setWristSetpoint(CoralStates state) {
        switch(state) {
            case C_STOW:
                wristAngleSetPoint = (Constants.Wrist.C_STOW_ANGLE);
                break;
            
            case C_L1:
                wristAngleSetPoint = (Constants.Wrist.C_L1_ANGLE);
                break;
            
            case C_L2:
                wristAngleSetPoint = (Constants.Wrist.C_L2_ANGLE);
                break;
            
            case C_L3:
                wristAngleSetPoint = (Constants.Wrist.C_L3_ANGLE);
                break;

            case C_L4:
                wristAngleSetPoint = (Constants.Wrist.C_L4_ANGLE);
                break;

            case C_LOAD:
                wristAngleSetPoint = (Constants.Wrist.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                wristAngleSetPoint = (Constants.Wrist.C_GROUND_ANGLE);
                break;

            case A_GROUND:
                wristAngleSetPoint = (Constants.Wrist.A_GROUND_ANGLE);
                break;

            case A_BARGE:
                wristAngleSetPoint = (Constants.Wrist.A_BARGE_ANGLE);
                break;
        }
    }

    public Command setWristSetpointCommand(CoralStates state) {
        return run(() -> {
            switch(state) {
                case C_STOW:
                    wristAngleSetPoint = (Constants.Wrist.C_STOW_ANGLE);
                    break;
                
                case C_L1:
                    wristAngleSetPoint = (Constants.Wrist.C_L1_ANGLE);
                    break;
                
                case C_L2:
                    wristAngleSetPoint = (Constants.Wrist.C_L2_ANGLE);
                    break;
                
                case C_L3:
                    wristAngleSetPoint = (Constants.Wrist.C_L3_ANGLE);
                    break;

                case C_L4:
                    wristAngleSetPoint = (Constants.Wrist.C_L4_ANGLE);
                    break;

                case C_LOAD:
                    wristAngleSetPoint = (Constants.Wrist.C_LOADING_ANGLE);
                    break;

                case C_GROUND:
                    wristAngleSetPoint = (Constants.Wrist.C_GROUND_ANGLE);
                    break;

                case A_GROUND:
                    wristAngleSetPoint = (Constants.Wrist.A_GROUND_ANGLE);
                    break;

                case A_BARGE:
                    wristAngleSetPoint = (Constants.Wrist.A_BARGE_ANGLE);
                    break;
            }
        });
    }

    public void reachWristTarget(double target) {
        wristAngleSetPoint = target;
        //wrist2Controller.setReference(target, ControlType.kMAXMotionPositionControl);
        wrist.setVoltage(wristController.calculate(wristEncoder.getPosition(), target) //+
                         /*wristFeedforward.calculate(target, wristController.getSetpoint().velocity)*/);
    }

    
    public Command setWristTarget(double target) {
        wristAngleSetPoint = target;
        return run(() -> reachWristTarget(target));
    }

    public void resetWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public double getWristEncoderPos() {
        return wristEncoder.getPosition();
    }

    public Command wristForward() {
        return run(() -> wrist.set(0.5));
    }

    public Command wristBackwards() {
        return run(() -> wrist.set(-0.5));
    }

    public Command stopWrist() {
        return run(() -> wrist.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());

        //SmartDashboard.putNumber("WristAngleSetpoint", wristAngleSetPoint);

        /*if (Constants.Testing.testingWrist) {
            reachWristTarget(SmartDashboard.getNumber("WristAngleSetpoint", Constants.Wrist.encoderOffset));
        }*/

        
        //reachWristTarget(SmartDashboard.getNumber("WristAngleSetpoint", Constants.Wrist.encoderOffset));

        
        reachWristTarget(wristAngleSetPoint);
    }
}
