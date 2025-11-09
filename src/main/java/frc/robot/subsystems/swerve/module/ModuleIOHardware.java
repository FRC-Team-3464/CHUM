package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ModuleIOHardware implements ModuleIO {
    private final TalonFX m_driveMotor;
    private final SparkMax m_turnMotor;

    private final CANcoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;

    private final Rotation2d m_angleOffset;

    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.kDriveKS,
            SwerveConstants.kDriveKV, SwerveConstants.kDriveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public ModuleIOHardware(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        m_driveMotor = new TalonFX(driveMotorID);
        m_turnMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        m_driveEncoder = new CANcoder(canCoderID);
        m_turnEncoder = m_turnMotor.getEncoder();
        m_angleOffset = angleOffset;
    }

    @Override
    public void setSpeed(double speed, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeed;
            m_driveMotor.set(percentOutput);
        } else {

        }
    }

    @Override
    public void setAngle(double angle) {
        m_driveEncoder.setPosition(angle);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVelocityMetersPerSec = m_driveMotor.getVelocity().getValueAsDouble()
                * SwerveConstants.kDriveRevToMeters;
        inputs.drivePositionMeters = m_driveMotor.getPosition().getValueAsDouble() * SwerveConstants.kDriveRevToMeters;
        inputs.turnAngleRad = Rotation2d.fromRotations(m_turnEncoder.getPosition() / SwerveConstants.kAngleGearRatio)
                .getRadians();
        inputs.turnVelocityRPM = m_turnEncoder.getVelocity();
        inputs.driveCurrentAmps = m_driveMotor.getSupplyCurrent().getValueAsDouble();
        inputs.turnCurrentAmps = m_turnMotor.getOutputCurrent();
    }
}
