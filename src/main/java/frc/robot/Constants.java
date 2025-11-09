// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.swerve.module.ModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ArmevatorConstants {
    public static double kElevatorGearRatio = 20;

    // table with index, elevator, and arm positions
    public static double[][] kPositions = {
        { 0, 0, 0 }, // stowed position
        { 1, 3.5, -15.14 }, // intake position 3.714
        { 2, 33.666, 4.4 }, // L2 position
        { 3, 0, 60.62 }, // L3 position
        { 4, 67.1415, 65.144 } // L4 position
    };

    public static double kElevatorDangerPosition = 25;
    public static double kArmDangerPosition = 3.714;
  }

  public static class AlgaeConstants {
    public static final double kIntakeSpeed = 0.3;
    public static final double kSpitSpeed = 0.3;

    public static final double kDeploySpeed = 0.3;
    public static final double kRetractSpeed = 0.3;
  }

  public static class SwerveConstants {

    public static final class ModuleConstants {

      /* Front Left Module - Module 0 */
      public static final class FrontLeft {
        public static final int kDriveMotorID = 1;
        public static final int kAngleMotorID = 2;
        public static final int kCanCoderID = 19;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(304.36523 - 180);
      }

      /* Front Right Module - Module 1 */
      public static final class FrontRight {
        public static final int kDriveMotorID = 3;
        public static final int kAngleMotorID = 4;
        public static final int kCanCoderID = 20;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(206.455);
      }

      /* Back Left Module - Module 2 */
      public static final class BackLeft {
        public static final int kDriveMotorID = 5;
        public static final int kAngleMotorID = 6;
        public static final int kCanCoderID = 21;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(35.419922 + 180);
      }

      /* Back Right Module - Module 3 */
      public static final class BackRight {
        public static final int kDriveMotorID = 7;
        public static final int kAngleMotorID = 8;
        public static final int kCanCoderID = 22;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(116.89453);
      }
    }

    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kAngleIdleMode = IdleMode.kBrake;
    public static final double kDrivePower = 1;
    public static final double kAnglePower = .9;

    public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

    // drivetrain constants
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    // Swerve kinematics, don't change
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // back left
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)); // back right

    // gear ratios
    public static final double kDriveGearRatio = (6.12 / 1.0);
    public static final double kAngleGearRatio = ((150.0 / 7.0) / 1.0);

    // encoder stuff
    // meters per rotation
    public static final double kDriveRevToMeters = kWheelCircumference / (kDriveGearRatio);
    public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60;

    /** The number of degrees that a single rotation of the turn motor turns the
    // wheel. */
    public static final double kDegreesPerTurnRotation = 360 / kAngleGearRatio;

    // motor inverts, check these
    public static final boolean kAngleMotorInvert = true;
    public static final InvertedValue kDriveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    public static final boolean kCanCoderInvert = false;

    /* Swerve Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 20;
    public static final int kAnglePeakCurrentLimit = 40;
    public static final double kAnglePeakCurrentDuration = 0.1;
    public static final boolean kAngleEnableCurrentLimit = true;

    public static final int kDriveSupplyCurrentLimit = 60;
    public static final boolean kDriveSupplyCurrentLimitEnable = true;
    public static final int kDriveSupplyCurrentThreshold = 60;
    public static final double kDriveSupplyTimeThreshold = 0.1;

    public static final boolean kDriveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double kAngleKP = 0.015;
    public static final double kAngleKI = 0;
    public static final double kAngleKD = 0;
    public static final double kAngleKF = 0;

    /* Drive Motor PID Values */

    public static final double kDriveKP = 0.01;
    public static final double kDriveKI = 0.0;
    public static final double kDriveKD = 0.0;

    public static final double kDriveKS = (0.32 / 12);
    public static final double kDriveKV = (1.988 / 12);
    public static final double kDriveKA = (1.0449 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double kPhysicalMaxSpeed = 5.0;
    public static final double kMaxTeleDriveSpeed = 4.5;
    // radians per sec
    public static final double kPhysicalMaxAngularSpeed = 2 * 2 * Math.PI;
    public static final double kMaxTeleAngularSpeed = kPhysicalMaxAngularSpeed / 2;

    public static final double kMaxAngularAccelerationSpeed = 4 / Math.PI;
    public static final double kMaxTeleAngularAccelerationSpeed = kMaxAngularAccelerationSpeed / 2;
    /** Radians per Second */

    public static final double kDeadband = 0.08;

    public static final Map<Integer, Double> kDistances = Map.of(
        0, 0.0,
        1, 1.0,
        2, 2.0,
        3, 3.0,
        4, 4.0);

    public static int targetPosition = 0;
  }

  public static final class AutoConstants {
    public static final double kPXController = 1.5;
    public static final double kPYController = 1;
    public static final double kPThetaController = 4;

    public static final double kThetaTolerance = 0;
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            SwerveConstants.kMaxTeleAngularSpeed,
            SwerveConstants.kMaxTeleAngularAccelerationSpeed);

    public static final PIDController xController = new PIDController(kPXController, 0, 0);
    public static final PIDController yController = new PIDController(kPYController, 0, 0);
    public static final ProfiledPIDController rotationController = new ProfiledPIDController(kPThetaController, 0, 0,
        thetaControllerConstraints);
  }

  /** Contains the constants for the controller ports, controllers, and buttons. <i>Buttons and controllers are not named with "k".</i> */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxStickPort = 1;

    public static final XboxController xbox = new XboxController(kDriverControllerPort);
    public static final Joystick auxStick = new Joystick(kAuxStickPort);

    public static final JoystickButton button1 = new JoystickButton(auxStick, 1);
    public static final JoystickButton button2 = new JoystickButton(auxStick, 2);
    public static final JoystickButton button3 = new JoystickButton(auxStick, 3);
    public static final JoystickButton button4 = new JoystickButton(auxStick, 4);
    public static final JoystickButton button5 = new JoystickButton(auxStick, 5);
    public static final JoystickButton button6 = new JoystickButton(auxStick, 6);
    public static final JoystickButton button7 = new JoystickButton(auxStick, 7);
    public static final JoystickButton button8 = new JoystickButton(auxStick, 8);
    public static final JoystickButton button9 = new JoystickButton(auxStick, 9);
    public static final JoystickButton button10 = new JoystickButton(auxStick, 10);
    public static final JoystickButton button11 = new JoystickButton(auxStick, 11);
    public static final JoystickButton button12 = new JoystickButton(auxStick, 12);

    public static final POVButton pancakeUp = new POVButton(auxStick, 0);
    public static final POVButton pancakeDown = new POVButton(auxStick, 180);
    public static final POVButton pancakeRight = new POVButton(auxStick, 90);
    public static final POVButton pancakeLeft = new POVButton(auxStick, 270);

    public static final POVButton dPadUp = new POVButton(xbox, 0);
    public static final POVButton dPadDown = new POVButton(xbox, 180);
    public static final POVButton dPadRight = new POVButton(xbox, 90);
    public static final POVButton dPadLeft = new POVButton(xbox, 270);

    public static final JoystickButton buttonA = new JoystickButton(xbox, 1);
    public static final JoystickButton buttonB = new JoystickButton(xbox, 2);
    public static final JoystickButton buttonX = new JoystickButton(xbox, 3);
    public static final JoystickButton buttonY = new JoystickButton(xbox, 4);
    public static final JoystickButton buttonLB = new JoystickButton(xbox, 5);
    public static final JoystickButton buttonRB = new JoystickButton(xbox, 6);
  }
}
