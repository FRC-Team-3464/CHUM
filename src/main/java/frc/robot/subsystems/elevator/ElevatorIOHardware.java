package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The {@code ElevatorIOHardware} class implements the {@link ElevatorIO} interface to
 * provide hardware-level access to elevator sensors, motors, and limit
 * switches.
 */
public class ElevatorIOHardware implements ElevatorIO {
    private final SparkMax m_leftMotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(10, MotorType.kBrushless);

    private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();

    private final DigitalInput m_minLimit = new DigitalInput(8);
    private final DigitalInput m_maxLimit = new DigitalInput(1);

    private final SparkMaxConfig m_rightConfig = new SparkMaxConfig();

    public ElevatorIOHardware() {
        m_rightConfig.follow(m_leftMotor, true);
        m_leftEncoder.setPosition(0);

        m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setElevatorVoltage(double volts) {
        m_leftMotor.setVoltage(volts);
    }

    @Override
    public void setElevatorSpeed(double speed) {
        m_leftMotor.set(speed);
    }

    @Override
    public void setPosition(double position) {
        m_leftEncoder.setPosition(position);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.maxLimit = m_maxLimit.get();
        inputs.minLimit = m_minLimit.get();
        inputs.currentAmps = m_leftMotor.getOutputCurrent();
        inputs.positionRot = m_leftEncoder.getPosition();
        inputs.velocityRPM = m_leftEncoder.getVelocity();
    }
}
