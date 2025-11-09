// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

/**
 * The {@code AlgaeSubsystem} controls the robot's algae intake mechanisms.
 * It manages the both the position and movement of the of the pivot and roller
 * motors.
 */
public class AlgaeSubsystem extends SubsystemBase {
	private final AlgaeIO m_io;
	private final AlgaeIOInputs m_inputs = new AlgaeIOInputs();

	private AlgaeState m_state = AlgaeState.IDLE;

	public enum AlgaeState {
		RETRACTING,
		DEPLOYING,
		INTAKING,
		SPITTING,
		IDLE
	}

	/** Creates a new AlgaeSubsystem. */
	public AlgaeSubsystem(AlgaeIO io) {
		m_io = io;
	}

	public void setState(AlgaeState state) {
		m_state = state;
	}

	public AlgaeState getState() {
		return m_state;
	}

	public AlgaeIOInputs getInputs() {
		return m_inputs;
	}

	@Override
	public void periodic() {
		m_io.updateInputs(m_inputs);

		switch (m_state) {
			case RETRACTING:
				m_io.setPivotSpeed(m_inputs.stowLimit ? 0.0 : AlgaeConstants.kRetractSpeed);
				break;
			case DEPLOYING:
				m_io.setPivotSpeed(m_inputs.extendedLimit ? 0.0 : AlgaeConstants.kDeploySpeed);
				break;
			case INTAKING:
				m_io.setRollerSpeed(AlgaeConstants.kIntakeSpeed);
				break;
			case SPITTING:
				m_io.setRollerSpeed(AlgaeConstants.kSpitSpeed);
				break;
			case IDLE:
				m_io.setPivotSpeed(0);
				m_io.setPivotSpeed(0);
		}
	}

}
