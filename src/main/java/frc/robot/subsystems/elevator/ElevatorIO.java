package frc.robot.subsystems.elevator;

/**
 * The {@code ElevatorIO} interface defines methods and attributes for the elevator.
 * It provides methods for changing the motor speeds.
 */
public interface ElevatorIO {
  /** Updates the ElevatorIOInputs class. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Values for the ElevatorSubsystem. */
  public class ElevatorIOInputs {

    /** True if the elevator is at its lowest position. */
    public boolean minLimit = false;

    /** True if the elevator is at its highest position. */
    public boolean maxLimit = false;

    /** Elevator Position (Rotations). */
    public double positionRot = 0.0;

    /** Elevator motor current draw. */
    public double currentAmps = 0.0;

    /** Elevator  velocity (RPM). */
    public double velocityRPM = 0.0;
  }

  /** Sets motor output (volts). */
  default void setElevatorVoltage(double voltage) {}

  /** Sets motor speed (% output). */
  default void setElevatorSpeed(double speed) {}

  /** Set encoder position. Mostly used to zero. */
  default void setPosition(double position) {}

}
