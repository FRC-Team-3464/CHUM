// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.module;

/** Add your docs here. */
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int canCoderID;
    public final Rotation2d angleOffset;


    public ModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}