// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.OldSwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignReef extends Command {
  /** Creates a new AutoAlignReef. */
  public final int[] coralIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  private final Map<Integer, Integer> gyroOffsets = Map.ofEntries(
    Map.entry(6, -60),
    Map.entry(7,0),
    Map.entry(8, 60),
    Map.entry(9, 120),
    Map.entry(10, 180),
    Map.entry(11, -120),
    Map.entry(17,60),
    Map.entry(18, 0),
    Map.entry(19, -60),
    Map.entry(20, -120),
    Map.entry(21, 180),
    Map.entry(22, 120),
    Map.entry(15, 90)
  );

  private double xspeed = 0;
  private double yspeed = 0;
  private double rotationSpeed = 0;


  private final PIDController xController = new PIDController(3, 0, 0);
  private final PIDController yController = new PIDController(3, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);
 
  private Transform2d TAG_TO_GOAL = new Transform2d();

  private double yOffset;

  public final VisionSubsystem visionSub = VisionSubsystem.getInstance();
  public final OldSwerveSubsystem swerveSub = OldSwerveSubsystem.getInstance();

  public final Transform3d ROBOT_TO_CAMERA_3D = visionSub.getFrontRobotToCamera();
  public final Transform2d ROBOT_TO_CAMERA = new Transform2d(ROBOT_TO_CAMERA_3D.getX(), ROBOT_TO_CAMERA_3D.getY(), ROBOT_TO_CAMERA_3D.getRotation().toRotation2d());

  private PhotonTrackedTarget lastTarget;

  private boolean right;

  private final Timer timer = new Timer();

  private final PhotonCamera photonCamera;

  public AutoAlignReef(boolean side) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonCamera = visionSub.getFrontCamera();

    yOffset = side ? Units.inchesToMeters(7) : Units.inchesToMeters(-4.5);
    right = side;

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(1));

    TAG_TO_GOAL = 
      new Transform2d(
        new Translation2d(0.4, side ? Units.inchesToMeters(5) : Units.inchesToMeters(-10)), 
        new Rotation3d(0, 0,  Math.PI).toRotation2d()
      );

    addRequirements(swerveSub);
    addRequirements(visionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = swerveSub.getPose();

    xController.setSetpoint(0.5);
    yController.setSetpoint(yOffset);
    thetaController.setSetpoint(0);
    // xController.reset(robotPose.getX());
    // yController.reset(robotPose.getY());
    // thetaController.reset(robotPose.getRotation().getRadians());

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose = swerveSub.getPose();
    var cameraResults = photonCamera.getAllUnreadResults();
    if (!cameraResults.isEmpty()) {
      var result = cameraResults.get(cameraResults.size() - 1);
      if (result.hasTargets()) {
        // var targetOpt = result.getTargets().stream()
        //   .filter(t -> Arrays.asList(coralIDs).contains(t.getFiducialId()))
        //   .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() >= 0.2 && t.getPoseAmbiguity() != -1)
        //   .findFirst();

        var targetOpt = result.getTargets().stream()
        .filter(t -> t.getPoseAmbiguity() != -1).findFirst();

        if (targetOpt.isPresent()) {
          // swerveSub.offsetGyro(gyroOffsets.get(targetOpt.get().getFiducialId()));
          // System.out.println(gyroOffsets.get(targetOpt.get().getFiducialId()));

          var target = targetOpt.get();
          lastTarget = target;
  
          var camToTarget = target.getBestCameraToTarget();
          var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

          var targetPose = cameraPose.transformBy(new Transform2d(camToTarget.getX(), camToTarget.getY(), camToTarget.getRotation().toRotation2d()));

          var goalPose = targetPose.transformBy(TAG_TO_GOAL);

          // var goalPoseNew = new Pose2d(camToTarget.getX() + robotPose.getX() - TAG_TO_GOAL.getX(), camToTarget.getY() + robotPose.getY() - yOffset, new Rotation2d(0));

          var goalPoseNew = visionSub.getRobotToTagTransform(right, targetOpt.get().fiducialId);

          // System.out.println(target.getBestCameraToTarget());
          System.out.println("goal: " + goalPose);

          var xSpeed = xController.calculate(robotPose.getX(), goalPoseNew.getX());
          var ySpeed = yController.calculate(robotPose.getY(), goalPoseNew.getY());
          var thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians(), goalPoseNew.getRotation().getRadians());

          swerveSub.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, false);

        }
      }
    }
    
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.drive(new Translation2d(0, 0), 0, false, true);
    System.out.println("goal reached");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(5)) {
      System.out.println("absolutely nothing in sight, look somewhere else");
      return true;
    }
    if (xController.atSetpoint() && yController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
