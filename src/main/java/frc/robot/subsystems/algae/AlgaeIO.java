package frc.robot.subsystems.algae;

/**
 * The {@code AlgaeIO} interface defines methods and attributes for the algae intake.
 * It provides methods for changing the motor speeds.
 */
public interface AlgaeIO {
  /** Updates the AlgaeIOInputs class. */
  default void updateInputs(AlgaeIOInputs inputs) {}

  /** Values for the AlgaeSubsystem. */
  public class AlgaeIOInputs {

    /** Pivot motor encoder position (rotations). */
    public double pivotPositionRot = 0.0;

    /** Roller velocity (RPM). */
    public double rollerVelocityRPM = 0.0;

    /** True if the intake is fully extended. */
    public boolean extendedLimit = false;

    /** True if the intake is fully retracted. */
    public boolean stowLimit = false;

    /** True if an algae piece is detected. */
    public boolean algaeDetected = false;
  }

  /** 
   * Sets the pivot motor speed (-1.0 to 1.0).
   * Positive values extend the intake; negative values retract it.
  */
  default void setPivotSpeed(double speed) {}

  /** 
   * Sets the roller motor speed (-1.0 to 1.0).
   * Positive values intake; negative values spit out.
  */
  default void setRollerSpeed(double speed) {}

}
