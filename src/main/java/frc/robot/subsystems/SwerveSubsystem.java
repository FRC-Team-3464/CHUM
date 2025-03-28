// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public static SwerveSubsystem instance;
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;

  public AHRS gyro;
  public SwerveDrivePoseEstimator poseEstimator;
  public RobotConfig config;


  private final Field2d field;
  
  public SwerveSubsystem() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.zeroYaw();

    swerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.ModConstants.Mod0.constants),
      new SwerveModule(1, Constants.ModConstants.Mod1.constants),
      new SwerveModule(2, Constants.ModConstants.Mod2.constants),
      new SwerveModule(3, Constants.ModConstants.Mod3.constants)
    };

    Timer.delay(1);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());

    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

    field = new Field2d();

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

     AutoBuilder.configure(
            this::getSwervePose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }
 
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.kMaxTeleDriveSpeed);

    for(SwerveModule mod : swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        // System.out.println(mod.moduleNumber + "    "+ mod.getCanCoder().getDegrees());
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : swerveMods) {
        positions[mod.getModuleNumber()] = mod.getPosition();
    } 
    return positions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : swerveMods){
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void offsetGyro(double angle) {
    gyro.setAngleAdjustment(angle);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getSwervePose() {
    return swerveOdometry.getPoseMeters();
  }

  public void zeroPose() {
    swerveOdometry.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void addVisionMeasurement(Pose2d estimatedRobotPose2d, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(estimatedRobotPose2d, timestampSeconds);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.kMaxTeleDriveSpeed);
    
    for(SwerveModule mod : swerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule mod : swerveMods){
        mod.resetToAbsolute();
    }
  }

  public Rotation2d getYaw() {
    return gyro.getRotation2d();
  }


  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getModulePositions());

    SmartDashboard.putNumber("Gyro Heading", gyro.getAngle());
    // field.setRobotPose(getPose());

    SmartDashboard.putData("Field", field);

    for(SwerveModule mod : swerveMods){
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + "Turn Motor Current", mod.getTurnCurrent());
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + "Drive Motor Current", mod.getDriveCurrent());
    // This method will be called once per scheduler run
    }
  }
}

