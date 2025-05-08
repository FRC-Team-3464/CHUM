// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseClimber extends Command {
  /** Creates a new RaiseClimber. */
  private ClimbSubsystem climberSub;

  public RaiseClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
    climberSub = ClimbSubsystem.getInstance();

    addRequirements(climberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSub.StopClimber();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSub.RaiseClimber(.5); //increase speed later, if needed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSub.StopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return climberSub.GetUpperLimit();
    return false;
  }
}
