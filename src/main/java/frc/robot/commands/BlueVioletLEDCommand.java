// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlueVioletLEDCommand extends Command {
  private LEDSubsystem LEDSub;
  /** Creates a new RedLEDCommand. */
  public BlueVioletLEDCommand() {
    LEDSub = LEDSubsystem.getInstance();
    addRequirements(LEDSub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // LEDSub.redLED();
    LEDSub.bluevioletLED();
    System.out.println("bluvio LEDs should be running");
  }

  // Called once the command ends or is interrupted which will then turn the LED lights off.
  @Override
  public void end(boolean interrupted) {
    LEDSub.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}