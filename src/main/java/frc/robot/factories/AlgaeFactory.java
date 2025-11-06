package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeState;

/**
 * Factory class for creating commands related to the algae subsystem.
 */
public class AlgaeFactory {
    private final AlgaeSubsystem m_subsystem;

    /** Creates a new AlgaeFactory. */
    public AlgaeFactory(AlgaeSubsystem subsystem) {
        m_subsystem = subsystem;
    }

    /** Sets AlgaeSubsystem state */
    private Command setState(AlgaeState state) {
        return new InstantCommand(
            () -> m_subsystem.setState(state), m_subsystem
        ).withName(state.toString());
    }

    public Command deployIntake() {
        return setState(AlgaeState.DEPLOYING);
    }

    public Command retractIntake() {
        return setState(AlgaeState.RETRACTING);
    }

    public Command intakeAlgae() {
        return setState(AlgaeState.INTAKING);
    }

    public Command spitAlgae() {
        return setState(AlgaeState.SPITTING);
    }

    public Command stop() {
        return setState(AlgaeState.STOPPED);
    }
}
