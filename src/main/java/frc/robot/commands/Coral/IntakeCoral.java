// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  private CoralSubsystem coralSub;
  private Timer timer;

  /** Creates a new IntakeCoral. */
  public IntakeCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    coralSub = CoralSubsystem.getInstance();
    armSub = ArmSubsystem.getInstance();

    addRequirements(coralSub);
    addRequirements(armSub);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tune speed later, through trial & error

    if(SwerveConstants.targetPosition == 2){
      coralSub.runIntake(-.9);
    } else if(SwerveConstants.targetPosition == 3) {
      coralSub.runIntake(0.5);
    } else if(SwerveConstants.targetPosition == 4) {
      coralSub.runIntake(0.5);
      elevatorSub.runElevator(0.3);
    } else {
      coralSub.runIntake(-0.5);
    }

    if (coralSub.getPhotoElectric()) {
      timer.start();
    }
    System.out.println(timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return coralSub.getPhotoElectric();
    return timer.hasElapsed(.09);
  }
}