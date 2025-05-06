// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignAprilTags extends Command {
    
  /** Creates a new AutoAlignAprilTags. */
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

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 5);
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);

  private final ProfiledPIDController xController = new ProfiledPIDController(7, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(7, 0, 0.4, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, THETA_CONSTRAINTS);

  private Transform2d TAG_TO_GOAL = new Transform2d();

  private double yOffset;

  public  Transform3d ROBOT_TO_CAMERA_3D; 
  public  Transform2d ROBOT_TO_CAMERA;

  public  Transform3d ROBOT_TO_CAMERA_3D_BACK; 
  public  Transform2d ROBOT_TO_CAMERA_BACK;

  private final Timer timer = new Timer();

  private final PhotonCamera photonCamera;

  private PhotonTrackedTarget lastTarget;

  public final VisionSubsystem visionSub = VisionSubsystem.getInstance();
  public final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();

  private boolean bFCamera;

  public AutoAlignAprilTags(boolean bFrontCamera, boolean side) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (bFrontCamera==true) {
        ROBOT_TO_CAMERA_3D = visionSub.getFrontRobotToCamera();
        ROBOT_TO_CAMERA = new Transform2d(ROBOT_TO_CAMERA_3D.getX(), ROBOT_TO_CAMERA_3D.getY(), ROBOT_TO_CAMERA_3D.getRotation().toRotation2d());

        photonCamera = visionSub.getFrontCamera();
        yOffset = side ? Units.inchesToMeters(7) : Units.inchesToMeters(-4.5);      
    }
    else {
        ROBOT_TO_CAMERA_3D_BACK = visionSub.getBackRobotToCamera();
        ROBOT_TO_CAMERA = new Transform2d(ROBOT_TO_CAMERA_3D_BACK.getX(), ROBOT_TO_CAMERA_3D_BACK.getY(), ROBOT_TO_CAMERA_3D_BACK.getRotation().toRotation2d());
    
        photonCamera = visionSub.getBackCamera();
        yOffset = Units.inchesToMeters(0);
   
    }
   
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // ???
    if (bFrontCamera==true)
    {
        TAG_TO_GOAL = 
            new Transform2d(
                new Translation2d(0.4, side ? Units.inchesToMeters(5) : Units.inchesToMeters(-10)), 
                new Rotation3d(0, 0,  Math.PI).toRotation2d()
            );       
    }
    else
    {
        TAG_TO_GOAL = 
            new Transform2d(
                new Translation2d(0.4, 0), 
                new Rotation3d(0, 0,  Math.PI).toRotation2d()
            );  
    }
    
    this.bFCamera = bFrontCamera;

    addRequirements(swerveSub);
    addRequirements(visionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = swerveSub.getPose();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());

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
         var targetOpt = result.getTargets().stream()
        .filter(t -> t.getPoseAmbiguity() != -1).findFirst();

        if (targetOpt.isPresent()) {
          // swerveSub.offsetGyro(gyroOffsets.get(targetOpt.get().getFiducialId()));
          System.out.println(gyroOffsets.get(targetOpt.get().getFiducialId()));

          var target = targetOpt.get();
          lastTarget = target;

          var camToTarget = target.getBestCameraToTarget();
          var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

          var targetPose = cameraPose.transformBy(new Transform2d(camToTarget.getX(), camToTarget.getY(), camToTarget.getRotation().toRotation2d()));

          var goalPose = targetPose.transformBy(TAG_TO_GOAL);

          var goalPoseNew = new Pose2d(camToTarget.getX() + robotPose.getX() - TAG_TO_GOAL.getX(), camToTarget.getY() + robotPose.getY() - yOffset, new Rotation2d(0));


          // System.out.println(target.getBestCameraToTarget());
          System.out.println("goal: " + goalPose);
          // System.out.println("current pose: " + swerveSub.getPose());
          xController.setGoal(goalPoseNew.getX());
          yController.setGoal(goalPoseNew.getY());
          thetaController.setGoal(goalPose.getRotation().getRadians());   

          // if (right) {
          //   yspeed = target.getBestCameraToTarget().getY() < (Units.inchesToMeters(4.5)) ? 0.3 : 0;
          // }
          // else {
          //   yspeed = target.getBestCameraToTarget().getY() > (Units.inchesToMeters(-4)) ? -0.3 : 0;
          // }
          // if (target.getBestCameraToTarget().getRotation().getAngle() < 0) {
          //   rotationSpeed = 0.2;
          // }
          // if (target.getBestCameraToTarget().getRotation().getAngle() > 0) {
          //   rotationSpeed = -0.2;
          // }
          // else {
          //   rotationSpeed = 0;
          // }
          // swerveSub.drive(new Translation2d(0, yspeed), rotationSpeed, false, true);
        }
      }
    }
    if (lastTarget == null) {
      swerveSub.drive(new Translation2d(0, 0), 0, false, true);
    }
    else {
      if (this.bFCamera == true) {

      var xSpeed = xController.atGoal() ? 0 : xController.calculate(robotPose.getX());
      var ySpeed = yController.atGoal() ? 0 : yController.calculate(robotPose.getY());
      var thetaSpeed = thetaController.atGoal() ? 0 : thetaController.calculate(robotPose.getRotation().getRadians());

      swerveSub.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, false);
      }
      else {
        var xSpeed = xController.atGoal() ? 0 : xController.calculate(robotPose.getX()*(-1));
        var ySpeed = yController.atGoal() ? 0 : yController.calculate(robotPose.getY()*(-1));
        var thetaSpeed = thetaController.atGoal() ? 0 : thetaController.calculate(robotPose.getRotation().getRadians());
          
        swerveSub.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, false);
      }
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.drive(new Translation2d(0, 0), 0, false, true);
    // swerveSub.offsetGyro(0);
    System.out.println("goal reached");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(5)) {
      System.out.println("absolutely nothing in sight, look somewhere else");
      return true;
    }
    if (xController.atGoal() && yController.atGoal()) {
      return true;
    }
    return false;
  }
}