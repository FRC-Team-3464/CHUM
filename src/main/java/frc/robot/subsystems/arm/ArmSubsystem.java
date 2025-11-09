// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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

  private final AbsoluteEncoder absArmEncoder = leftMotor.getAbsoluteEncoder();
  private static ArmSubsystem instance = null;

  private boolean manual;

  private final ProfiledPIDController armController = new ProfiledPIDController(0.9, 0, 0, new TrapezoidProfile.Constraints(140, 300));
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final DigitalInput minArmSwitch = new DigitalInput(4);
  private final DigitalInput maxArmSwitch = new DigitalInput(5);
  
  private SparkMaxConfig rightMotorConfig;
  private SparkMaxConfig leftMotorConfig;

  public ArmSubsystem() {
    armController.setTolerance(.1);

    leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.absoluteEncoder.zeroOffset(0);
    leftMotorConfig.absoluteEncoder.positionConversionFactor(160);
    leftMotorConfig.absoluteEncoder.inverted(true);
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    // if (getAbsArmPosition() > 90 && speed > 0) {
    //   leftMotor.set(0);
    // }
    // else if (getAbsArmPosition() < 8 && speed < 0) {
    //   leftMotor.set(0);
    // }
    // else {
    manual = true;
    leftMotor.set(speed);
    // }

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
    manual = false;
    armController.setGoal(target);
    // double voltage = armController.calculate(getRelativeArmPosition());
    // if (getAbsArmPosition() > 90 && voltage > 0) {
    //   leftMotor.set(0);
    // }

    // System.out.println("arm voltage: " + armController.calculate(getRelativeArmPosition()) + "     arm position: "  + getRelativeArmPosition() + "    target: " + target);
  }

  public double getRelativeArmPosition() {
    return leftEncoder.getPosition();
  }

  public void setArmEncoderPosition(double position) {
    leftEncoder.setPosition(position);
  }

  public boolean armAtPosition() {
    return armController.atSetpoint();
  }

  public double getAbsArmPosition() {
    return absArmEncoder.getPosition();
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

    SmartDashboard.putNumber("Arm Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Arm Degrees", getRelativeArmPosition());
    SmartDashboard.putBoolean("Arm Max Limit", getMaxArmLimit());
    SmartDashboard.putBoolean("Arm Min Limit", getMinArmLimit());
    SmartDashboard.putNumber("Abs Encoder Degrees", getAbsArmPosition());
    // SmartDashboard.putNumber("Arm Setpoint", moveToPosition());
    if (!manual) {
      leftMotor.setVoltage(armController.calculate(getRelativeArmPosition()));
    }
  }
}
