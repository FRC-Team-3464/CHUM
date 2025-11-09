package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    default void updateInputs(ModuleIOInputs inputs) {}

    public class ModuleIOInputs {
        /** Velocity of the drive motor (meters per second).  */
        public double driveVelocityMetersPerSec = 0.0;

        /** Position of the drive motor (meters). */
        public double drivePositionMeters = 0.0;

        /** Angle of the turn motor (radians).*/
        public double turnAngleRad = 0.0;

        /** Velocity of the turn motor (RPM). */
        public double turnVelocityRPM = 0.0;

        /** Current of the drive motor (amps). */
        public double driveCurrentAmps = 0.0;

        /** Current of the turn motor (amps). */
        public double turnCurrentAmps = 0.0;
    }

    /** Sets the drive motor speed (open- or closed-loop). */
    default void setSpeed(double speed, boolean isOpenLoop) {}

     /** Sets the turn motor angle (in radians). */
    default void setAngle(double angle) {}

    default SwerveModuleState getState() { return new SwerveModuleState(); }

     /** Returns the current module position (distance + angle). */
    default SwerveModuleState getPosition() { return new SwerveModuleState(); }

     /** Resets the turning encoder to match absolute CANcoder. */
    default void resetToAbsolute() {}
}
