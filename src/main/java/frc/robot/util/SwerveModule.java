// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.time.chrono.IsoChronology;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX driveMotor;
    private SparkMax turnMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder turnEncoder;

    private CANcoderConfigurator angleEncoderConfigurator;
    private CANcoderConfiguration angleEncoderConfiguration;
    private EncoderConfig turnEncoderConfig;
    private SparkMaxConfig turnSparkMaxConfig;
    private TalonFXConfiguration driveMotorConfiguration;
    private TalonFXConfigurator driveMotorConfigurator;
    private CurrentLimitsConfigs driveSupplyLimit;

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int moduleNumber, ModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // turnMotor configuration
        turnMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configureTurnMotor();

        // driveMotor configuration
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configureDriveMotor();

        // ecoders configuration
        angleEncoder = new CANcoder(moduleConstants.canCoderID);
        configureEncoders();
    }



    public void setDesiredState(SwerveModuleState desiredState, Boolean isOpenLoop) {
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        // cos compensation for reducing skew, reduces the speed when it's not pointing in the right direction
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos(); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        SmartDashboard.putNumber("Desired angle", desiredState.angle.getDegrees());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeed;
            driveMotor.set(percentOutput);
        }

        else {
          driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
        //   driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
          driveMotor.setControl(driveVelocity);
        }
      }


    public void setAngle(SwerveModuleState desiredState) {
        // if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.kMaxTeleDriveSpeed * 0.001))
        // {
        //  turnMotor.stopMotor();
        //  return;
        // }
        Rotation2d angle = desiredState.angle;
         SparkClosedLoopController controller = turnMotor.getClosedLoopController();
         controller.setReference(angle.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getTurnCurrent() {
        return turnMotor.getAppliedOutput();
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        turnEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            (driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.driveRevToMeters), 
            getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            (driveMotor.getPosition().getValueAsDouble() * SwerveConstants.driveRevToMeters),
            getAngle());
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public double getMotorSpeed() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    private void configureEncoders() {
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfiguration = new CANcoderConfiguration();
        angleEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoderConfigurator.apply(angleEncoderConfiguration);

        turnEncoder = turnMotor.getEncoder();        
    }

    private void configureTurnMotor() {
        turnSparkMaxConfig = new SparkMaxConfig();
        turnEncoderConfig = new EncoderConfig();
        turnSparkMaxConfig
            .smartCurrentLimit(SwerveConstants.anglePeakCurrentLimit)
            .idleMode(SwerveConstants.angleIdleMode)
            .inverted(SwerveConstants.angleMotorInvert)
            .closedLoop
                .pidf(
                    SwerveConstants.angleKP, 
                    SwerveConstants.angleKI, 
                    SwerveConstants.angleKD, 
                    SwerveConstants.angleKF, 
                    ClosedLoopSlot.kSlot0)
                .outputRange(-SwerveConstants.anglePower, SwerveConstants.anglePower);
        turnEncoderConfig
            .positionConversionFactor(SwerveConstants.DegreesPerTurnRotation)
            .velocityConversionFactor(SwerveConstants.DegreesPerTurnRotation / 60); // this is degrees per sec
        turnSparkMaxConfig.encoder.apply(turnEncoderConfig);
        turnMotor.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureDriveMotor(){     
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfigurator = driveMotor.getConfigurator();
        driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.driveSupplyCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentLowerLimit = SwerveConstants.driveSupplyCurrentThreshold;
        driveSupplyLimit.SupplyCurrentLowerTime = SwerveConstants.driveSupplyTimeThreshold;

        driveMotorConfiguration.Slot0.kP = SwerveConstants.driveKP;
        driveMotorConfiguration.Slot0.kI = SwerveConstants.driveKI;
        driveMotorConfiguration.Slot0.kD = SwerveConstants.driveKD;
        driveMotorConfiguration.Slot0.kS = SwerveConstants.driveKS; 
        driveMotorConfiguration.Slot0.kV = SwerveConstants.driveKV;
        driveMotorConfiguration.Slot0.kA = SwerveConstants.driveKA;
        driveMotorConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        driveMotorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        driveMotorConfiguration.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotorConfigurator.apply(driveMotorConfiguration);
        driveMotorConfigurator.apply(driveSupplyLimit);
    }

}

