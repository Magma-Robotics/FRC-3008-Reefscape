package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetCoralState extends Command {
    private final Arm arm;
    private final Wrist wrist;
    private final Elevator elevator;
    private final CoralStates state;

    public SetCoralState(Arm arm, Wrist wrist, Elevator elevator, CoralStates state) {
        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
        this.state = state;
        addRequirements(arm, elevator, wrist);
    }

    public void initialize() {

    }

    public void execute() {
        switch(state) {
            case C_STOW:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;

            case C_L1:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;

            case C_L2:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;

            case C_L3:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;

            case C_L4:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break; 
                
            case C_LOAD:
                arm.setArmSetpoint(state);
                elevator.setElevatorSetpoint(state);
                wrist.setWristSetpoint(state);
                break;

            case C_GROUND:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);

            case A_GROUND:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;

            case A_BARGE:
                arm.setArmSetpoint(state);
                wrist.setWristSetpoint(state);
                elevator.setElevatorSetpoint(state);
                break;
        }
    }

    public void end() {
        
    }

    public boolean isFinished() {
        return true;
    }
    
}
