// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Armevator.ArmevatorToPosition;
import frc.robot.commands.Armevator.RunElevator;
import frc.robot.commands.Coral.ReverseCoralIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardAuto extends SequentialCommandGroup {
  /** Creates a new DriveForwardAuto. */
  public final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();
  public DriveForwardAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPosition(0)),
      new InstantCommand(() -> ArmSubsystem.getInstance().setArmEncoderPosition(0)),
    new ParallelDeadlineGroup(
      new WaitCommand(1),  
      new InstantCommand(() -> swerveSub.drive(new Translation2d(0.3, 0), 0, false, true)),
      new ArmevatorToPosition(4)),
    new ParallelDeadlineGroup(
      new WaitCommand(1),
      new ReverseCoralIntake(),
      new RunElevator(true)
      ),
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new InstantCommand(() -> swerveSub.drive(new Translation2d(0.3, 0), 0, false, true))
    ),
    new ArmevatorToPosition(0)
    );
    
  }
}
