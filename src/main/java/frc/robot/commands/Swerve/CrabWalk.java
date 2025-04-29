// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CrabWalk extends Command {
  /** Creates a new DampenSwerve. */
  private double direction;
  private final SwerveSubsystem swerveSub;
  public CrabWalk(double direction) {
    this.direction = direction;
    swerveSub = SwerveSubsystem.getInstance();
    addRequirements(swerveSub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(direction);
    if (direction == 0) {
      swerveSub.drive(new Translation2d(0.5, 0), 0, false, true);
    }
    if (direction == 90) {
      swerveSub.drive(new Translation2d(0, -0.4), 0, false, true);
    }
    if (direction == 180) {
      swerveSub.drive(new Translation2d(-0.3, 0), 0, false, true);
    }
    if (direction == 270) {
      swerveSub.drive(new Translation2d(0, 0.4), 0, false, true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
