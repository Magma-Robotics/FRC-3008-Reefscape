package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStates.CoralStates;
import frc.robot.subsystems.Arm;

public class SetArmSetpointCommand extends Command {
    private Arm arm;
    private CoralStates state;

    public SetArmSetpointCommand(Arm arm, CoralStates state) {
        this.arm = arm;
        this.state = state;
        addRequirements();
    }

    @Override  
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setArmSetpoint(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
