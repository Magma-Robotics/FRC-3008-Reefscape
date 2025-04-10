package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
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

public class Elevator extends SubsystemBase {
    public final Trigger atMin = new Trigger(() -> getLinearPosition().isNear(Constants.Elevator.kMinElevatorHeight, 
    Inches.of(3)));
    public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(Constants.Elevator.kMaxElevatorHeight,
    Inches.of(3)));

    private SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();
    private SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();

    private SparkFlex leftElevator = new SparkFlex(Constants.CANIds.kLeftElevatorID, MotorType.kBrushless);
    private SparkClosedLoopController elevatorController = leftElevator.getClosedLoopController();
    private RelativeEncoder leftElevatorEncoder = leftElevator.getEncoder();
    public static double kElevatorP = 0.6052;
    public static double kElevatorI = 0;
    public static double kElevatorD = 0;

    public static double elevatorSetpoint = 0;

    private SparkFlex rightElevator = new SparkFlex(Constants.CANIds.kRightElevatorID, MotorType.kBrushless);
    private RelativeEncoder rightElevatorEncoder = rightElevator.getEncoder();

    private ElevatorFeedforward elevatorFeedforward = 
        new ElevatorFeedforward(0.30021, 0.27706, 3.7525, 0.36076);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
        kElevatorP,
        kElevatorI,
        kElevatorD,
        new Constraints(Constants.Elevator.maxElevatorVelocity, Constants.Elevator.maxElevatorAcceleration));

    // SysId Routine and setup
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance       m_distance       = Inches.mutable(0);
    private final MutAngle          m_rotations      = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity       = InchesPerSecond.mutable(0);
    // SysID Routine
    private final SysIdRoutine      m_sysIdRoutine   =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.per(Second).of(0.7),
                                    Volts.of(5),
                                    Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                leftElevator::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the elevator motor.
                    log.motor("elevator")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftElevator.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getLeftElevatorEncoderPos(),
                                                            Inches)) // Records Height in Inches via SysIdRoutineLog.linearPosition
                    .linearVelocity(m_velocity.mut_replace(leftElevatorEncoder.getVelocity(),
                                                            InchesPerSecond)); // Records velocity in InchesPerSecond via SysIdRoutineLog.linearVelocity
                },
                this));

    public Elevator() {
        //create configs
        leftElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.1)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);
        /* 
        leftElevatorConfig
            .encoder
            .positionConversionFactor(Constants.Elevator.kElevatorRotationsToInches)
            .velocityConversionFactor(Constants.Elevator.kElevatorRPMToInPerSec);
        leftElevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kElevatorP, kElevatorI, kElevatorD)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(Constants.Elevator.maxElevatorVelocity)
            .maxAcceleration(Constants.Elevator.maxElevatorAcceleration)
            .allowedClosedLoopError(0.5);*/

        

        rightElevatorConfig
            .idleMode(IdleMode.kBrake)
            .follow(leftElevator, false)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.1)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);
        rightElevatorConfig
            .encoder
            .positionConversionFactor(Constants.Elevator.kElevatorRotationsToInches)
            .velocityConversionFactor(Constants.Elevator.kElevatorRPMToInPerSec);

        //sets configs
        leftElevator.configure(leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightElevator.configure(rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_controller.setTolerance(0.5);

        SmartDashboard.putNumber("ElevatorSetpoint", elevatorSetpoint);
    }

    public Command elevatorUp() {
        return run(() -> {
            elevatorSetpoint += 0.13;
            reachElevatorTarget(elevatorSetpoint);
        });
    }

    public Command elevatorDown() {
        return run(() -> {
            elevatorSetpoint -= 0.13;
            reachElevatorTarget(elevatorSetpoint);
        });
    }

    public Command stopElevator() {
        return run(() -> {
            reachElevatorTarget(elevatorSetpoint);
        });
    }

    public void reachElevatorTarget(double target) {
        elevatorSetpoint = target;
        /*elevatorController.setReference(target, 
                                        ControlType.kMAXMotionPositionControl);*/
        double voltsOut = MathUtil.clamp(
            m_controller.calculate(leftElevatorEncoder.getPosition(), target) /*+
            elevatorFeedforward.calculateWithVelocities(getElevatorVelocity().in(InchesPerSecond),
                                                        m_controller.getSetpoint().velocity)*/, -7, 7);
        leftElevator.setVoltage(voltsOut);
    }

    public Command setElevatorTarget(double target) {
        elevatorSetpoint = target;
        return run(() -> reachElevatorTarget(target));
    }

    public void setElevatorState(Constants.RobotStates.CoralStates state)  {
        switch(state) {
            case C_STOW:
                reachElevatorTarget(Constants.Elevator.C_STOW_POS);
                break;
            
            case C_L1:
                reachElevatorTarget(Constants.Elevator.C_L1_POS);
                break;
            
            case C_L2:
                reachElevatorTarget(Constants.Elevator.C_L2_POS);
                break;
            
            case C_L3:
                reachElevatorTarget(Constants.Elevator.C_L3_POS);
                break;

            case C_L4:
                reachElevatorTarget(Constants.Elevator.C_L4_POS);
                break;
                
            case C_LOAD:
                reachElevatorTarget(Constants.Elevator.C_LOADING_POS);
                break;

            case C_GROUND:
                reachElevatorTarget(Constants.Elevator.C_GROUND_POS);
                break;
        }

            
    }

    public void setElevatorSetpoint(Constants.RobotStates.CoralStates state) {
        switch(state) {
            case C_STOW:
                elevatorSetpoint = Constants.Elevator.C_STOW_POS;
                break;
            
            case C_L1:
                elevatorSetpoint = Constants.Elevator.C_L1_POS;
                break;
            
            case C_L2:
                elevatorSetpoint = Constants.Elevator.C_L2_POS;
                break;
            
            case C_L3:
                elevatorSetpoint = Constants.Elevator.C_L3_POS;
                break;

            case C_L4:
                elevatorSetpoint = Constants.Elevator.C_L4_POS;
                break;
                
            case C_LOAD:
                elevatorSetpoint = Constants.Elevator.C_LOADING_POS;
                break;

            case C_GROUND:
                elevatorSetpoint = Constants.Elevator.C_GROUND_POS;
                break;
        }
    }

    public Command setElevatorStateCommand(Constants.RobotStates.CoralStates state) {
        return run(() -> {
            switch(state) {
                case C_STOW:
                    reachElevatorTarget(Constants.Elevator.C_STOW_POS);
                    break;
                
                case C_L1:
                    reachElevatorTarget(Constants.Elevator.C_L1_POS);
                    break;
                
                case C_L2:
                    reachElevatorTarget(Constants.Elevator.C_L2_POS);
                    break;
                
                case C_L3:
                    reachElevatorTarget(Constants.Elevator.C_L3_POS);
                    break;
    
                case C_L4:
                    reachElevatorTarget(Constants.Elevator.C_L4_POS);
                    break;
                    
                case C_LOAD:
                    reachElevatorTarget(Constants.Elevator.C_LOADING_POS);
                    break;
    
                case C_GROUND:
                    reachElevatorTarget(Constants.Elevator.C_GROUND_POS);
                    break;
            }
        });
    }

    /**
   * Runs the SysId routine to tune the Arm
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine() {
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }

    public double getLeftElevatorEncoderPos() {
        return leftElevatorEncoder.getPosition();
    }

    public void resetLeftElevatorEncoder() {
        leftElevatorEncoder.setPosition(0);
    }

    public LinearVelocity getElevatorVelocity() {
        return InchesPerSecond.of(leftElevatorEncoder.getVelocity());
    }

    public Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height, getLeftElevatorEncoderPos(), tolerance));
    }

    public Distance getLinearPosition() {
        return Inches.of(getLeftElevatorEncoderPos());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Pos", leftElevatorEncoder.getPosition());
        //SmartDashboard.putNumber("ElevatorSetpoint", elevatorSetpoint);

        /*if (Constants.Testing.testingElevator) {
            reachElevatorTarget(SmartDashboard.getNumber("ElevatorSetpoint", 0));
        }*/
        //reachElevatorTarget(SmartDashboard.getNumber("ElevatorSetpoint", 0));
        reachElevatorTarget(elevatorSetpoint);
    }
}
