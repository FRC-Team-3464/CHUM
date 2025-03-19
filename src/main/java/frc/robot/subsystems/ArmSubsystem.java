// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private final SparkMax leftMotor = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(12, MotorType.kBrushless);

  private final DutyCycleEncoder absArmEncoder = new DutyCycleEncoder(3);
  private static ArmSubsystem instance = null;

  private final ProfiledPIDController armController = new ProfiledPIDController(.9, 0, 00, new TrapezoidProfile.Constraints(100, 300));
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final DigitalInput minArmSwitch = new DigitalInput(4);
  private final DigitalInput maxArmSwitch = new DigitalInput(5);
  
  private SparkMaxConfig rightMotorConfig;

  public ArmSubsystem() {
    armController.setTolerance(.5);
    rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.follow(11, true);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftEncoder.setPosition(0);
    
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void runArm(double speed) {
    // if (getMaxArmLimit()) {
    //   leftMotor.set(0);
    // }
    // else if (getMinArmLimit()) {
    //   leftMotor.set(0);
    // }
    // else {
    //   leftMotor.set(speed);
    // }
    leftMotor.set(speed);
  }

  public void moveToPosition(double target) {
    if (getMaxArmLimit()) {
      leftMotor.set(0);
    }
    else if (getMinArmLimit()) {
      leftMotor.set(0);
    }
    else if (Math.abs(target - getArmDegrees()) < target && target < getArmDegrees()) {
      leftMotor.set(0.5);
    }
    else if (Math.abs(target - getArmDegrees()) > target && target > getArmDegrees()) {
      leftMotor.set(-0.5);
    }
    else {
      leftMotor.set(0);
    }
  }

  public void setArmTarget(double target) {
    armController.setGoal(target);
    leftMotor.setVoltage(armController.calculate(getRelativeArmPosition()));
    // System.out.println("arm voltage: " + armController.calculate(getRelativeArmPosition()) + "     arm position: "  + getRelativeArmPosition() + "    target: " + target);

    // leftMotor.setVoltage(armController.calculate(leftEncoder.getPosition()) + armFeedforward.calculate(Units.degreesToRadians(leftEncoder.getPosition()-90), armController.getSetpoint().velocity));
  }

  public double getRelativeArmPosition() {
    return leftEncoder.getPosition();
  }

  public void setArmEncoderPosition(double position) {
    leftEncoder.setPosition(position);
  }

  public boolean armAtPosition() {
    return armController.atGoal();
  }

  public double getAbsArmPosition() {
    return absArmEncoder.get();
  }

  public double getArmDegrees() {
    double rotations = getAbsArmPosition();
    return rotations * (360/277);
  }


  public boolean getMaxArmLimit() {
    return maxArmSwitch.get();
  }

  public boolean getMinArmLimit() {
    return minArmSwitch.get();
  }

  public void setEncoderPosition(double position) {
    leftEncoder.setPosition(position); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Degrees", getRelativeArmPosition());
    SmartDashboard.putBoolean("Arm Max Limit", getMaxArmLimit());
    SmartDashboard.putBoolean("Arm Min Limit", getMinArmLimit());
    // SmartDashboard.putNumber("Arm Setpoint", moveToPosition());
  }
}
