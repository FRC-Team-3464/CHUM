package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

public class ElevatorFactory {
    private final ElevatorSubsystem m_subsystem;

    /** Creates a new ElevatorFactory. */
    public ElevatorFactory(ElevatorSubsystem subsystem) {
        m_subsystem = subsystem;
    }

    private Command setState(ElevatorState state) {
        return new InstantCommand(
           () -> m_subsystem.setState(state),
           m_subsystem
        ).withName(state.toString());
    }

    public Command moveToZero() {
        return setState(ElevatorState.ZERO);
    }

    public Command moveToIntake() {
        return setState(ElevatorState.INTAKE);
    }

    public Command moveToL2() {
        return setState(ElevatorState.L2);
    }

    public Command moveToL3() {
        return setState(ElevatorState.L3);
    }

    public Command moveToL4() {
        return setState(ElevatorState.L4);
    }

    public Command stop() {
        return setState(ElevatorState.STOPPED);
    }
}
