// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Assigning variables to the motors and other objects 
    private final SparkMax leftElevator = new SparkMax(9, MotorType.kBrushless); 
    private final SparkMax rightElevator = new SparkMax(10, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftElevator.getEncoder();
    private final RelativeEncoder rightEncoder = rightElevator.getEncoder();

    private final ProfiledPIDController elevatorController = new ProfiledPIDController(0.8, 0, 0, new TrapezoidProfile.Constraints(120, 800));
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(.081, .254, 1, .01); // to be used in the future hopefully


    // assigning the limit switches
    private final DigitalInput minElevatorLimit = new DigitalInput(8);
    private final DigitalInput maxElevatorLimit = new DigitalInput(1);
    
    // configures the right motor to be newly configured 
    private SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();
    private SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();


    Boolean LeftResistance;
    Boolean RightResistance;


  public ElevatorSubsystem() {
    // setting resistance/directions both motors will run in  
    leftEncoder.setPosition(0);
    elevatorController.setGoal(0);
    elevatorController.setTolerance(1);
    
    LeftResistance = false;
    RightResistance = false; 

    rightSparkMaxConfig.follow(leftElevator, true);
    // sets the right elevator motor to actually be configured to the code that sets the configuration
    rightElevator.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // CREATING GET INSTANCE
  private static ElevatorSubsystem instance = null;
  
  public static ElevatorSubsystem getInstance(){
    if (instance == null) {
      instance= new ElevatorSubsystem();  
    }
    return instance;
  }
  
  public void setElevateTarget(double target) {
    // checking which direction the elevator has to travel in to reach the target parameter after accounting for limit switches
        System.out.println("setElevateTarget should be running");
        System.out.println(Math.abs(Constants.ArmevatorConstants.kElevatorGearRatio * (target) - getElevatorPosition()));

    if (getMinElevatorLimit()) {
      leftElevator.set(0);
      System.out.println("speed set to 0 because bottom limit switch");
    }
    else if (getMaxElevatorLimit()) {
      leftElevator.set(0);
      System.out.println("speed set to 0 because top limit switch");
    }
    else if (Math.abs(Constants.ArmevatorConstants.kElevatorGearRatio * (target) - getElevatorPosition()) < 5) {
      leftElevator.set(0);
    }
    else if (Math.abs(Constants.ArmevatorConstants.kElevatorGearRatio * (target) - getElevatorPosition()) > 5 && getElevatorPosition() < target) {
      leftElevator.set(0.1);
      System.out.println("speed set to 0.6");
    }
    else if (Math.abs(Constants.ArmevatorConstants.kElevatorGearRatio * (target) - getElevatorPosition()) > 5 && getElevatorPosition() > target) {
      leftElevator.set(-0.1);
      System.out.println("speed set to -0.6");
    }
  }

  // setting the default speed of the left elevator at zero while also accounting for limit switches
  public void runElevator(double speed) {
    if (getMinElevatorLimit() && speed < 0) {
      leftElevator.set(0);
    }
    else if (getMaxElevatorLimit() && speed > 0) {
      leftElevator.set(0);
    }
    else {
      leftElevator.set(speed);
    }
    System.out.println(speed);
  }

  public void setElevatorPosition(double position) {
    leftEncoder.setPosition(position);
  }

  public void setElevatorTarget(double target) {
    elevatorController.setGoal(target);
    // leftElevator.setVoltage(elevatorFeedforward.calculate(elevatorController.getSetpoint().velocity));
    leftElevator.setVoltage(elevatorController.calculate(leftEncoder.getPosition()));
  }

  public boolean elevatorAtPosition() {
    return elevatorController.atGoal();
  }


  public double getElevatorPosition() {
    // gets elevator position based on number of rotations of duty cycle encoder
    return leftEncoder.getPosition();
  }



  public boolean getMaxElevatorLimit() {
    // checks if maximum limit switch is hit
    return !maxElevatorLimit.get();
  }
  public boolean getMinElevatorLimit() {
    // checks if minimum limit switch is hit
    return !minElevatorLimit.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Min Elevator Limit", getMinElevatorLimit());
    SmartDashboard.putBoolean("Max Elevator Limit", getMaxElevatorLimit());
    SmartDashboard.putNumber("Elevator Position",getElevatorPosition());
    if (getMinElevatorLimit()) {
      leftEncoder.setPosition(0);
    };
    if (getMaxElevatorLimit()) {
      leftEncoder.setPosition(81);
    }
    if (getElevatorPosition() >= 81) {
      runElevator(0.02);
    }
    SmartDashboard.putNumber("Elevator Draw", leftElevator.getOutputCurrent());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Speed", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Voltage", leftElevator.getAppliedOutput()*RobotController.getBatteryVoltage());
  }

  
}
