// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.module;

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
import frc.robot.util.ModuleUtils;

/** Add your docs here. */
public class OldSwerveModule {
    
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

    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.kDriveKS, SwerveConstants.kDriveKV, SwerveConstants.kDriveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public OldSwerveModule(int moduleNumber, ModuleConstants moduleConstants) {
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
        desiredState = ModuleUtils.optimize(desiredState, getState().angle);
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
          driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.kWheelCircumference;
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
            (driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.kDriveRevToMeters), 
            getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            (driveMotor.getPosition().getValueAsDouble() * SwerveConstants.kDriveRevToMeters),
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
            .smartCurrentLimit(SwerveConstants.kAnglePeakCurrentLimit)
            .idleMode(SwerveConstants.kAngleIdleMode)
            .inverted(SwerveConstants.kAngleMotorInvert)
            .closedLoop
                .pidf(
                    SwerveConstants.kAngleKP, 
                    SwerveConstants.kAngleKI, 
                    SwerveConstants.kAngleKD, 
                    SwerveConstants.kAngleKF, 
                    ClosedLoopSlot.kSlot0)
                .outputRange(-SwerveConstants.kAnglePower, SwerveConstants.kAnglePower);
        turnEncoderConfig
            .positionConversionFactor(SwerveConstants.kDegreesPerTurnRotation)
            .velocityConversionFactor(SwerveConstants.kDegreesPerTurnRotation / 60); // this is degrees per sec
        turnSparkMaxConfig.encoder.apply(turnEncoderConfig);
        turnMotor.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureDriveMotor(){     
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfigurator = driveMotor.getConfigurator();
        driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.kDriveSupplyCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.kDriveSupplyCurrentLimitEnable;
        driveSupplyLimit.SupplyCurrentLowerLimit = SwerveConstants.kDriveSupplyCurrentThreshold;
        driveSupplyLimit.SupplyCurrentLowerTime = SwerveConstants.kDriveSupplyTimeThreshold;

        driveMotorConfiguration.Slot0.kP = SwerveConstants.kDriveKP;
        driveMotorConfiguration.Slot0.kI = SwerveConstants.kDriveKI;
        driveMotorConfiguration.Slot0.kD = SwerveConstants.kDriveKD;
        driveMotorConfiguration.Slot0.kS = SwerveConstants.kDriveKS; 
        driveMotorConfiguration.Slot0.kV = SwerveConstants.kDriveKV;
        driveMotorConfiguration.Slot0.kA = SwerveConstants.kDriveKA;
        driveMotorConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.kOpenLoopRamp;
        driveMotorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.kClosedLoopRamp;
        driveMotorConfiguration.MotorOutput.Inverted = SwerveConstants.kDriveMotorInvert;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotorConfigurator.apply(driveMotorConfiguration);
        driveMotorConfigurator.apply(driveSupplyLimit);
    }

}

