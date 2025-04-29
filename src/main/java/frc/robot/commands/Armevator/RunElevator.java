// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Armevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  private final ElevatorSubsystem elevatorSub;
  public boolean direction;
  /** Creates a new RunElevator. */
  public RunElevator(boolean direction) {
    elevatorSub = ElevatorSubsystem.getInstance();
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == true) {
      elevatorSub.runElevator(0.3);
    }
    else if (direction == false) {
      elevatorSub.runElevator(-0.3);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.runElevator(0.02); //0.028

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
