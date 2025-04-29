// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L1Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotDown extends Command {
  /** Creates a new PivotUp. */

  private L1Subsystem L1Sub; 

  public PivotDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    L1Sub = L1Subsystem.getInstance();

    addRequirements(L1Sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    L1Sub.StopPivot(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    L1Sub.PivotDown(.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    L1Sub.StopPivot(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
