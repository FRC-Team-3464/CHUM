// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRotation extends Command {
  /** Creates a new AutoRotation. */
  private final SwerveSubsystem swerveSub;
  private double target;

  public AutoRotation() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSub = SwerveSubsystem.getInstance();
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double target = swerveSub.getYaw().getRadians() - Math.PI;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.drive(new Translation2d(0, 0), target, isFinished(), isScheduled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveSub.getYaw().getRadians() >= target;
  }
}
