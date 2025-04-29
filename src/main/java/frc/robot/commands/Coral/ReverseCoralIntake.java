// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseCoralIntake extends Command {
 
  private CoralSubsystem coralSub;
  private ElevatorSubsystem elevatorSub;
  private LEDSubsystem ledSub;

  private double speed;


  /** Creates a new IntakeCoral. */
  public ReverseCoralIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    coralSub = CoralSubsystem.getInstance();
    elevatorSub = ElevatorSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();

    addRequirements(elevatorSub);
    addRequirements(coralSub);
    addRequirements(ledSub); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Tune speed later, through trial & error

    speed = SwerveConstants.targetPosition == 2 ? -0.9 : 0.5;

    if (SwerveConstants.targetPosition == 4) {
      elevatorSub.runElevator(0.3);
    }

    coralSub.runIntake(speed);
    ledSub.redLED();
    System.out.println("Red outtake LEDs should be running");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // elevatorSub.runElevator(0);
    coralSub.runIntake(0);
    ledSub.setOff(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !(coralSub.getPhotoElectric());
    return false;
  }
}
