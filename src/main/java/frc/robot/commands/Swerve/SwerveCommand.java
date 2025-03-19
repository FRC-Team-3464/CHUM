// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {
  private SwerveSubsystem swerveSub;
  private PIDController rotationController;
  private DoubleSupplier ySpeedSup;
  private DoubleSupplier xSpeedSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  // private BooleanSupplier dampenSup;
  // private DoubleSupplier dynamicHeadingSup;

  /** Creates a new SwerveCommand. */
  public SwerveCommand(DoubleSupplier ySpeedSup, DoubleSupplier xSpeedSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup /*, DoubleSupplier dynamicHeadingSup */) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSub = SwerveSubsystem.getInstance();
    addRequirements(swerveSub);

    rotationController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Constants.AutoConstants.kThetaTolerance);

    this.ySpeedSup = ySpeedSup;
    this.xSpeedSup = xSpeedSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    // this.dampenSup = dampen;
    // this.dynamicHeadingSup = dynamicHeadingSup;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -ySpeedSup.getAsDouble()*0.8;
    double ySpeed = -xSpeedSup.getAsDouble()*0.8;
    double rotation = -rotationSup.getAsDouble();
    
    xSpeed = Math.abs(xSpeed) > Constants.SwerveConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > SwerveConstants.kDeadband ? ySpeed : 0.0;
    rotation = Math.abs(rotation) > SwerveConstants.kDeadband ? rotation : 0.0;

    rotation *= Constants.SwerveConstants.kMaxTeleAngularSpeed;
    swerveSub.drive(new Translation2d(xSpeed, ySpeed).times(SwerveConstants.kPhysicalMaxSpeed), rotation, robotCentricSup.getAsBoolean(), true);
    // translation strafe
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.drive(new Translation2d(0, 0), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

