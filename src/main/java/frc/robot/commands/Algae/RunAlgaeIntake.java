// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Algae;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.algae.AlgaeSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class RunAlgaeIntake extends Command {
//   /** Creates a new RunAlgaeIntake. */
//   private AlgaeSubsystem algaeSub;

//   private boolean targetReached;
//   private double rotations;

//   public RunAlgaeIntake() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     algaeSub = AlgaeSubsystem.getInstance();
//     addRequirements(algaeSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     targetReached = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     algaeSub.runAlgaeMotor(0.3);
//     // if (algaeSub.getAlgaeSpeed() >= 2000) {
//     //   targetReached = true;
//     // }
//     // if (targetReached) {
//     //   if (algaeSub.getAlgaeSpeed() <= 1800) {
//     //     rotations = algaeSub.getAlgaeIntakePosition();
//     //   }
//     // }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     algaeSub.runAlgaeMotor(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return algaeSub.getAlgae();
//     // return false;
//   }
// }
