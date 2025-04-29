// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseAlgaeIntake extends Command {
  /** Creates a new ReverseAlgaeIntake. */
  private AlgaeSubsystem algaeSub;

  private boolean targetReached;
  private double rotations;
  private boolean rotationsSet;

  public ReverseAlgaeIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    algaeSub = AlgaeSubsystem.getInstance();
    addRequirements(algaeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetReached = false;
    rotationsSet = false;
    rotations = 0;
    algaeSub.setAlgaeIntakePosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeSub.runAlgaeMotor(-0.3);
    // if (algaeSub.getAlgaeSpeed() <= -1600) {
    //   targetReached = true;
    // }
    // if (targetReached) {
    //   if (algaeSub.getAlgaeSpeed() <= 1800) {
    //     if (!rotationsSet) {
    //       rotations = algaeSub.getAlgaeIntakePosition();
    //       rotationsSet = true;
    //       System.out.println("set");
    //     }
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeSub.runAlgaeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (rotationsSet && (algaeSub.getAlgaeIntakePosition() - rotations) < -15);
    return false;
  }
}
