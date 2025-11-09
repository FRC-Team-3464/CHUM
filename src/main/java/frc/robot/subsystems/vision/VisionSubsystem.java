// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.OldSwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public static VisionSubsystem instance;

  public final PhotonCamera frontAprilCamera;
  // public final PhotonCamera backAprilCamera;

  private final PhotonPoseEstimator photonPoseEstimatorFront;
  // private final PhotonPoseEstimator photonPoseEstimatorBack;

  private final Transform3d frontCameraTransform = new Transform3d(
    new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(2), Units.inchesToMeters(7)),
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

  private final Transform3d backCameraTransform = new Transform3d(
    new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

  public final AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    frontAprilCamera = new PhotonCamera("Front April Camera");
    // backAprilCamera = new PhotonCamera("Back April Camera");

    photonPoseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCameraTransform);
    // photonPoseEstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCameraTransform);

    photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  public static VisionSubsystem getInstance() {
    if (instance == null) {
      instance = new VisionSubsystem();
    }
    return instance;
  }


  public PhotonCamera getFrontCamera() {
    return frontAprilCamera;
  }

  public Transform3d getFrontRobotToCamera() {
    return frontCameraTransform;
  }

  public Pose2d getRobotToTagTransform(boolean right, int Id) { 
    Pose2d tagPose = fieldLayout.getTagPose(Id).get().toPose2d();
    Pose2d robotPose = OldSwerveSubsystem.getInstance().getPose();
    return robotPose.relativeTo(tagPose);

  }



  public void addVisionMeasurement(PhotonPoseEstimator photonPoseEstimator, PhotonCamera photonCamera) {

    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var change : photonCamera.getAllUnreadResults()) {
      visionEst = photonPoseEstimator.update(change);
    }
    double timestampSeconds = visionEst.get().timestampSeconds;

    OldSwerveSubsystem.getInstance().addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), timestampSeconds);
  }
  


  @Override
  public void periodic() {
    // addVisionMeasurement(photonPoseEstimatorFront, frontAprilCamera);
    // addVisionMeasurement(photonPoseEstimatorBack, backAprilCamera);
    // This method will be called once per scheduler run
  }
}
