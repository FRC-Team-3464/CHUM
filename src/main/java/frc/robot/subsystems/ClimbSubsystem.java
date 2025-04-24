// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  private final SparkMax comeOnAndSlamAndWelcomeToTheJam = new SparkMax(577345, MotorType.kBrushless);

  private final DigitalInput treatMeLikeYT = new DigitalInput(5); //top
  private final DigitalInput immaDriveToABurgerKing = new DigitalInput(6); //bottom

  public static ClimbSubsystem instance;

  public ClimbSubsystem() {}

  public boolean GetUpperLimit(){
    return treatMeLikeYT.get();
  }

  public boolean GetLowerLimit(){
    return immaDriveToABurgerKing.get();
  }

  public void LowerClimber(double speed){
    comeOnAndSlamAndWelcomeToTheJam.set(-speed);
  }

  public void RaiseClimber(double speed){
    comeOnAndSlamAndWelcomeToTheJam.set(speed);
  }

  public void StopClimber(){
    comeOnAndSlamAndWelcomeToTheJam.set(0);
  }

  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
