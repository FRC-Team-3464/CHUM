// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  private CoralSubsystem coralSub;
  private ElevatorSubsystem elevatorSub;
  private Timer timer;
  private double speed;
  // private final Map<Integer, Double> speedsMap = Map.ofEntries(
  //   Map.entry(0, -0.5),
  //   Map.entry(1, -0.5),
  //   Map.entry(2, -0.9),
  //   Map.entry(3, 0.5),
  //   Map.entry(4, 0.5)
  // );
  

  /** Creates a new IntakeCoral. */
  public IntakeCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    coralSub = CoralSubsystem.getInstance();

    addRequirements(coralSub);

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

    // speed = speedsMap.get(SwerveConstants.targetPosition);
    // System.out.println(SwerveConstants.targetPosition);
    coralSub.runIntake(-0.5);
    if (coralSub.getPhotoElectric()) {
      timer.start();
    }
    
    

    // if(SwerveConstants.targetPosition == 4){
    //   coralSub.runIntake(speed);
    //   elevatorSub.runElevator(0.3);
    // } else {
    //   coralSub.runIntake(speed);
    // }
    // if (coralSub.getPhotoElectric() && SwerveConstants.targetPosition == 1) {
    //   timer.start();
    // }
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
    return timer.hasElapsed(.1);
  }
}
