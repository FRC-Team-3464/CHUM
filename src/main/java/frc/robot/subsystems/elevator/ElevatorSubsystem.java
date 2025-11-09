// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmevatorConstants;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeState;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO m_io;
  private final ElevatorIOInputs m_inputs = new ElevatorIOInputs();
  private ElevatorState m_state;

  private final ProfiledPIDController m_elevatorController = new ProfiledPIDController(
    0.8, 0, 0, 
    new TrapezoidProfile.Constraints(140, 800)
  );

  public enum ElevatorState {
    MANUAL,
    ZERO,
    INTAKE,
    L2,
    L3,
    L4,
    STOPPED
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    m_io = io;
    m_elevatorController.setTolerance(0.1);
  }

  public void setState(ElevatorState state) {
    m_state = state;
  }

  public ElevatorState getState() {
		return m_state;
	}

  public ElevatorIOInputs getInputs() {
		return m_inputs;
	}

  public void resetPosition() {
    m_io.setPosition(0);
  }

  public void moveToGoal() {
    m_io.setElevatorVoltage(m_elevatorController.calculate(m_inputs.positionRot) + 0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.updateInputs(m_inputs);

    switch (m_state) {
      case ZERO:
        m_elevatorController.setGoal(ArmevatorConstants.kPositions[0][1]);
        moveToGoal();
        break;
      case INTAKE:
        m_elevatorController.setGoal(ArmevatorConstants.kPositions[1][1]);
        moveToGoal();
        break;
      case L2:
        m_elevatorController.setGoal(ArmevatorConstants.kPositions[2][1]);
        moveToGoal();
        break;
      case L3:
        m_elevatorController.setGoal(ArmevatorConstants.kPositions[3][1]);
        moveToGoal();
        break;
      case L4:
        m_elevatorController.setGoal(ArmevatorConstants.kPositions[4][1]);
        moveToGoal();
        break;
      case STOPPED:
        m_io.setElevatorSpeed(0);
        break;
      default:
        break;
    }
  }
}
