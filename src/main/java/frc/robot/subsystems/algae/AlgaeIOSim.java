package frc.robot.subsystems.algae;

/**
 * The {@code AlgaeIOSim} class simulates the Algae Subsystem.
 * It is used to test algae subsystem behavior without requiring
 * physical hardware.
 */
public class AlgaeIOSim implements AlgaeIO {
    private double m_pivotPosition = 0.0;
    private double m_rollerVelocity = 0.0;

    private boolean m_extendedLimit = false;
    private boolean m_stowLimit = true;
    private boolean m_algaeDetected = false;

    public AlgaeIOSim() {}

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.pivotPositionRot = m_pivotPosition;
        inputs.rollerVelocityRPM = m_rollerVelocity;
        inputs.extendedLimit = m_extendedLimit;
        inputs.stowLimit = m_stowLimit;
        inputs.algaeDetected = m_algaeDetected;
    }

    /* Simulates limits */
    public void setExtendedLimit(boolean extended) { m_extendedLimit = extended; }
    public void setStowLimit(boolean stow) { m_stowLimit = stow; }
    public void setAlgaeDetected(boolean detected) { m_algaeDetected = detected; }
}
