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
import frc.robot.Constants.Wrist;
import frc.robot.Constants.RobotStates.CoralStates;

public class Arm extends SubsystemBase {
    private SparkFlexConfig armPivotConfig = new SparkFlexConfig();

    public final Trigger atArmMin = new Trigger(() -> getArmAngle().lte(Constants.Arm.kMinAngle.plus(Degrees.of(5))));
    public final Trigger atArmMax = new Trigger(() -> getArmAngle().gte(Constants.Arm.kMaxAngle.minus(Degrees.of(5))));


    //initialize arm pivot
    private SparkFlex armPivot = 
        new SparkFlex(Constants.CANIds.kArmPivotID, MotorType.kBrushless);
    private SparkClosedLoopController armPivotController = armPivot.getClosedLoopController();
    private RelativeEncoder armPivotEncoder = armPivot.getEncoder();
    public static double kArmP = 1.1;
    public static double kArmI = 0;
    public static double kArmD = 0;
    public static double armAngleSetPoint = Constants.Arm.encoderOffset;

    private final ProfiledPIDController armController;
    private final ArmFeedforward        armFeedforward = new ArmFeedforward(0.772,
                                                                            94.827,
                                                                            13.597,
                                                                            1.6165);



    // SysId Routine and seutp
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage         m_armAppliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle           m_armAngle          = Degrees.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_armVelocity       = DegreesPerSecond.mutable(0);
    // SysID Routine
    private final SysIdRoutine       m_armSysIdRoutine   =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(6), Seconds.of(30)),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                armPivot::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("arm")
                    .voltage(
                        m_armAppliedVoltage.mut_replace(armPivot.getAppliedOutput() *
                                                        RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_armAngle.mut_replace(armPivotEncoder.getPosition(), Degrees))
                    .angularVelocity(m_armVelocity.mut_replace(armPivotEncoder.getVelocity(), DegreesPerSecond));
                },
                this));


    public Arm() {
        //create configs
        armPivotConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        armPivotConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kArmP, kArmI, kArmD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(Constants.Arm.maxArmVelocity)
            .maxAcceleration(Constants.Arm.maxArmAcceleration)
            .allowedClosedLoopError(0.5)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        armPivotConfig
            .encoder
            //degrees
            .positionConversionFactor(Constants.Arm.kArmRotationsToDeg)
            //degrees per sec
            .velocityConversionFactor(Constants.Arm.kArmRPMtoDegPerSec);

        //set configs for motors
        armPivot.configure(armPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = new ProfiledPIDController(kArmP,
                                                  kArmI,
                                                  kArmD,
                                                  new Constraints(Constants.Arm.maxArmVelocity,
                                                                  Constants.Arm.maxArmAcceleration));
        armController.setTolerance(0.5);

        


        armPivotEncoder.setPosition(90);

        SmartDashboard.putNumber("ArmAngleSetpoint", armAngleSetPoint);

    }

    public Trigger atArmAngle(double angle, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(angle, armPivotEncoder.getPosition(), tolerance));
    }

    public Angle getArmAngle() {
        return Degrees.of(armPivotEncoder.getPosition());
    }

    public AngularVelocity getArmVelocity() {
        return DegreesPerSecond.of(armPivotEncoder.getVelocity());
    }


    public Command runArmSysIdRoutine()
  {
    return m_armSysIdRoutine.dynamic(Direction.kForward).until(atArmMax)
                         .andThen(m_armSysIdRoutine.dynamic(Direction.kReverse).until(atArmMin))
                         .andThen(m_armSysIdRoutine.quasistatic(Direction.kForward).until(atArmMax))
                         .andThen(m_armSysIdRoutine.quasistatic(Direction.kReverse).until(atArmMin));
  }

  
    public void reachArmPivotTarget(double target) {
        armAngleSetPoint = target;
        //armPivotController.setReference(target, ControlType.kMAXMotionPositionControl);
        armPivot.setVoltage(armController.calculate(armPivotEncoder.getPosition(), target) //+
                            /*armFeedforward.calculate(target, armController.getSetpoint().velocity)*/);
    }

    public Command setArmPivotTarget(double target) {
        armAngleSetPoint = target;
        return run(() -> reachArmPivotTarget(target));
    }

    /*public Command setManualArm(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            wristAngleSetPoint += rightJoystick.getAsDouble();
            reachWristTarget(wristAngleSetPoint);
            armAngleSetPoint += leftJoystick.getAsDouble();
            reachArmPivotTarget(armAngleSetPoint);
        });
    }

    public Command stopWholeArm() {
        return run(() -> {
            armPivot.set(0);
            wrist.set(0);
        });
    }

    public Command stopWholePIDArm() {
        return run(() -> {
            reachArmPivotTarget(armAngleSetPoint);
            reachWristTarget(wristAngleSetPoint);
        });
    }*/

    public Command setManualArm(DoubleSupplier joystick) {
        return run(() -> {
            armAngleSetPoint += joystick.getAsDouble();
            reachArmPivotTarget(armAngleSetPoint);
        });
    }

    public Command stopPIDArm() {
        return run(() -> {
            reachArmPivotTarget(armAngleSetPoint);
        });
    }


    /*public Command setManualArmVoltageWithLimiter(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            double armSpeed = leftJoystick.getAsDouble() * 0.6;
            double wristSpeed = rightJoystick.getAsDouble() * 0.6;
            if (armAngleSetPoint <= 45 || wristAngleSetPoint >= 59) {
                wrist.set(wristSpeed);
                armPivot.set(armSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
                armAngleSetPoint = armPivotEncoder.getPosition();
            }
            else if (wristAngleSetPoint < 59 && armAngleSetPoint > 45 && armSpeed <= 0) {
                wrist.set(wristSpeed);
                armPivot.set(armSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
                armAngleSetPoint = armPivotEncoder.getPosition();
            }
            else if (wristAngleSetPoint < 59 && armAngleSetPoint > 45 && armSpeed >= 0) {
                wrist.set(wristSpeed);
                wristAngleSetPoint = wristEncoder.getPosition();
            }
        });
    }*/
/* 
    public Command setManualArmVoltage(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        return run(() -> {
            double armSpeed = leftJoystick.getAsDouble() * 0.6;
            double wristSpeed = rightJoystick.getAsDouble() * 0.6;
            wrist.set(wristSpeed);
            armPivot.set(armSpeed);
            wristAngleSetPoint = wristEncoder.getPosition();
            armAngleSetPoint = armPivotEncoder.getPosition();
        });
    }*/

    public void setArmPivotState(Constants.RobotStates.CoralStates state)  {
        switch(state) {
            case C_STOW:
                reachArmPivotTarget(Constants.Arm.C_STOW_ANGLE);
                break;
            
            case C_L1:
                reachArmPivotTarget(Constants.Arm.C_L1_ANGLE);
                break;
            
            case C_L2:
                reachArmPivotTarget(Constants.Arm.C_L2_ANGLE);
                break;
            
            case C_L3:
                reachArmPivotTarget(Constants.Arm.C_L3_ANGLE);
                break;

            case C_L4:
                reachArmPivotTarget(Constants.Arm.C_L4_ANGLE);
                break;

            case C_LOAD:
                reachArmPivotTarget(Constants.Arm.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                reachArmPivotTarget(Constants.Arm.C_GROUND_ANGLE);
                break;
        }
    }

    public void setArmSetpoint(CoralStates state) {
        switch(state) {
            case C_STOW:
                armAngleSetPoint = (Constants.Arm.C_STOW_ANGLE);
                break;
            
            case C_L1:
                armAngleSetPoint = (Constants.Arm.C_L1_ANGLE);
                break;
            
            case C_L2:
                armAngleSetPoint = (Constants.Arm.C_L2_ANGLE);
                break;
            
            case C_L3:
                armAngleSetPoint = (Constants.Arm.C_L3_ANGLE);
                break;

            case C_L4:
                armAngleSetPoint = (Constants.Arm.C_L4_ANGLE);
                break;

            case C_LOAD:
                armAngleSetPoint = (Constants.Arm.C_LOADING_ANGLE);
                break;

            case C_GROUND:
                armAngleSetPoint = (Constants.Arm.C_GROUND_ANGLE);
                break;

            case A_GROUND:
                armAngleSetPoint = (Constants.Arm.A_GROUND_ANGLE);
                break;

            case A_BARGE:
                armAngleSetPoint = (Constants.Arm.A_BARGE_ANGLE);
                break;
        }
    }

    public Command setArmSetpointCommand(CoralStates state) {
        return run(() -> {
            switch(state) {
                case C_STOW:
                    armAngleSetPoint = (Constants.Arm.C_STOW_ANGLE);
                    break;
                
                case C_L1:
                    armAngleSetPoint = (Constants.Arm.C_L1_ANGLE);
                    break;
                
                case C_L2:
                    armAngleSetPoint = (Constants.Arm.C_L2_ANGLE);
                    break;
                
                case C_L3:
                    armAngleSetPoint = (Constants.Arm.C_L3_ANGLE);
                    break;

                case C_L4:
                    armAngleSetPoint = (Constants.Arm.C_L4_ANGLE);
                    break;

                case C_LOAD:
                    armAngleSetPoint = (Constants.Arm.C_LOADING_ANGLE);
                    break;

                case C_GROUND:
                    armAngleSetPoint = (Constants.Arm.C_GROUND_ANGLE);
                    break;

                case A_GROUND:
                    armAngleSetPoint = (Constants.Arm.A_GROUND_ANGLE);
                    break;

                case A_BARGE:
                    armAngleSetPoint = (Constants.Arm.A_BARGE_ANGLE);
                    break;
            }
        });
    }

    public Command setArmPivotStateCommand(Constants.RobotStates.CoralStates state) {
        return runOnce(() -> {
            switch(state) {
                case C_STOW:
                    reachArmPivotTarget(Constants.Arm.C_STOW_ANGLE);
                    break;
                
                case C_L1:
                    reachArmPivotTarget(Constants.Arm.C_L1_ANGLE);
                    break;
                
                case C_L2:
                    reachArmPivotTarget(Constants.Arm.C_L2_ANGLE);
                    break;
                
                case C_L3:
                    reachArmPivotTarget(Constants.Arm.C_L3_ANGLE);
                    break;
    
                case C_L4:
                    reachArmPivotTarget(Constants.Arm.C_L4_ANGLE);
                    break;
    
                case C_LOAD:
                    reachArmPivotTarget(Constants.Arm.C_LOADING_ANGLE);
                    break;
    
                case C_GROUND:
                    reachArmPivotTarget(Constants.Arm.C_GROUND_ANGLE);
                    break;
            }
        });
    }

    public void resetArmPivotEncoder() {
        armPivotEncoder.setPosition(0);
    }

 

    public double getArmPivotEncoderPos() {
        return armPivotEncoder.getPosition();
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armPivotEncoder.getPosition());

        //SmartDashboard.putNumber("ArmAngleSetpoint", armAngleSetPoint);

        /*if (Constants.Testing.testingArm) {
            reachArmPivotTarget(SmartDashboard.getNumber("ArmAngleSetpoint", Constants.Arm.encoderOffset));
        }*/

        //reachArmPivotTarget(SmartDashboard.getNumber("ArmAngleSetpoint", Constants.Arm.encoderOffset));
        reachArmPivotTarget(armAngleSetPoint);
    }
}
