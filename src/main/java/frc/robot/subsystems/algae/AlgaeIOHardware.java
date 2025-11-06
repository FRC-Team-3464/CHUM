package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/** 
 * The {@code AlgaeIOHardware} class implements the {@link AlgaeIO} interface to
 * provide hardware-level access to algae intake sensors, motors, and limit switches.
*/
public class AlgaeIOHardware implements AlgaeIO {

    private final SparkMax m_pivotMotor = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax m_rollerMotor = new SparkMax(15, MotorType.kBrushless);

    private final RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();
    private final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();

    private final DigitalInput m_algaeSensor = new DigitalInput(9);
    private final DigitalInput m_extendedLimit = new DigitalInput(0);
    private final DigitalInput m_stowLimit = new DigitalInput(7);
    
    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.pivotPositionRot = m_pivotEncoder.getPosition();
        inputs.rollerVelocityRPM = m_rollerEncoder.getVelocity();
        inputs.extendedLimit = m_extendedLimit.get();
        inputs.stowLimit = m_stowLimit.get();
        inputs.algaeDetected = m_algaeSensor.get();
    }

    @Override
    public void setPivotSpeed(double speed) {
        /* -speed because motor faces opposite direction */
        m_pivotMotor.set(-speed);
    }

    @Override
    public void setRollerSpeed(double speed) {
        m_rollerMotor.set(speed);
    }
    
}
