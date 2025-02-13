package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.AlgaeStates;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;

public class SetAlgaeState extends Command {
    private final AlgaePivot algaePivot;
    private final AlgaeStates state;

    public SetAlgaeState(AlgaePivot algaePivot, AlgaeStates state) {
        this.algaePivot = algaePivot;
        this.state = state;
        addRequirements(algaePivot);
    }

    public void initialize() {

    }

    public void execute() {
        switch(state) {
            case A_STOW:
                algaePivot.setAlgaePivotState(state);
                break;

            case A_PROCCESSOR:
                algaePivot.setAlgaePivotState(state);
                break;

            case A_LOAD:
                algaePivot.setAlgaePivotState(state);
                break;
        }
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}
