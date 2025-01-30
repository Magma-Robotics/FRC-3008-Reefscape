package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class SetCoralState extends Command {
    private final Arm arm;
    private final Elevator elevator;
    private final CoralStates state;

    public SetCoralState(Arm arm, Elevator elevator, CoralStates state) {
        this.arm = arm;
        this.elevator = elevator;
        this.state = state;
        addRequirements(arm, elevator);
    }

    public void initialize() {

    }

    public void execute() {
        switch(state) {
            case C_STOW:
                arm.setArmPivotState(state);
                elevator.setElevatorState(state);
                break;
            case C_L1:
                break;
            case C_L2:
                break;
            case C_L3:
                break;
            case C_L4:
                break; 
        }
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
    
}
