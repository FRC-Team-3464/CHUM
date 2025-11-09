// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.OldSwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPosition extends Command {
  /** Creates a new AutoPosition. */
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);

  private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, THETA_CONSTRAINTS);
 

  public final VisionSubsystem visionSub = VisionSubsystem.getInstance();
  public final OldSwerveSubsystem swerveSub = OldSwerveSubsystem.getInstance();

  public AutoPosition() {
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(visionSub);
    addRequirements(swerveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var robotPose = swerveSub.getPose();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());
    System.out.println("robiot pose is at the beginning: " + robotPose);

    xController.setGoal(robotPose.getX());
    yController.setGoal(robotPose.getY() + 1);
    thetaController.setGoal(0);   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var robotPose = swerveSub.getPose();
    var xSpeed = xController.atGoal() ? 0 : xController.calculate(robotPose.getX());
    var ySpeed = yController.atGoal() ? 0 : yController.calculate(robotPose.getY());
    var thetaSpeed = thetaController.atGoal() ? 0 : thetaController.calculate(robotPose.getRotation().getRadians());


    // System.out.println("x speed: " + xSpeed);
    // System.out.println("y speed: " + ySpeed);
    // System.out.println("theta speed: " + thetaSpeed);

    // System.out.println("x pose: " + robotPose.getX());
    // System.out.println("y pose: " + robotPose.getY());
    // System.out.println("theta pose: " + robotPose.getRotation().getRadians());


    swerveSub.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // swerveSub.drive(new Translation2d(0, 0), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}