package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.AlgaeStates;
import frc.robot.subsystems.AlgaeSubsystem;

public class SetAlgaeState extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final AlgaeStates state;

    public SetAlgaeState(AlgaeSubsystem algaeSubsystem, AlgaeStates state) {
        this.algaeSubsystem = algaeSubsystem;
        this.state = state;
        addRequirements(algaeSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        switch(state) {
            case A_STOW:
                algaeSubsystem.setAlgaePivotState(state);
                break;

            case A_PROCCESSOR:
                algaeSubsystem.setAlgaePivotState(state);
                break;

            case A_LOAD:
                algaeSubsystem.setAlgaePivotState(state);
                break;
        }
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}
