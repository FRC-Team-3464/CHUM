// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Subsystem extends SubsystemBase {
  /** Creates a new L1Subsystem. */
  public static L1Subsystem instance;

  private final SparkMax intakeMotor = new SparkMax(30, MotorType.kBrushless);
  private final SparkMax pivotingMotor = new SparkMax(31, MotorType.kBrushless);

  private final DigitalInput SuperHighTechStopper = new DigitalInput(7);
  
  public L1Subsystem() {}

  public void PivotUp(double Variable){

    pivotingMotor.set(Variable);

  }

  public static L1Subsystem getInstance(){
    if (instance == null){
      instance = new L1Subsystem();
    }
    return instance;
  }

  public void PivotDown(double AnotherVariable){

    pivotingMotor.set(AnotherVariable);

  }

  public void StopPivot(double AnotherAnotherVarible){

    AnotherAnotherVarible = 0;
    intakeMotor.set(AnotherAnotherVarible);

  }

  public void Intake(double AnotherAnotherAnotherVariable){

    pivotingMotor.set(AnotherAnotherAnotherVariable);

  }

  public void Outtake(double AnotherAnotherAnotherAnotherVariable){

    intakeMotor.set(-(AnotherAnotherAnotherAnotherVariable));

  }

  public void StopIntake(double AnotherAnotherAnotherAnotherAnotherVariable){

    AnotherAnotherAnotherAnotherAnotherVariable = 0;
    intakeMotor.set(AnotherAnotherAnotherAnotherAnotherVariable);

  }

  public boolean GetIntakeLimit(){

    return SuperHighTechStopper.get();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
