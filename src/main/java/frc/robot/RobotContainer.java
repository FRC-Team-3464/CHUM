// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.BenPeiLEDCommand;
import frc.robot.commands.BluGrenConGradientLEDCommand;
import frc.robot.commands.BlueLEDCommand;
import frc.robot.commands.BlueVioletLEDCommand;
import frc.robot.commands.ClimbersLEDCommand;
import frc.robot.commands.DenimLEDCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FunEffectLEDCommand;
import frc.robot.commands.GhostWhiteLEDCommand;
import frc.robot.commands.GoldLEDCommand;
import frc.robot.commands.GreenLEDCommand;
import frc.robot.commands.IntakeLEDCommand;
import frc.robot.commands.OrangeLEDCommand;
import frc.robot.commands.RainbowLEDCommand;
import frc.robot.commands.RedBluNonContGradLEDCommand;
import frc.robot.commands.RedLEDCommand;
import frc.robot.commands.WarningPulseLEDCommand;
import frc.robot.commands.ClimbersLEDCommand;
import frc.robot.commands.YelRedConGradientLEDCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final SendableChooser<Command> autoChooser;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSub.setDefaultCommand(
      new SwerveCommand(
        () -> Constants.OperatorConstants.xbox.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> Constants.OperatorConstants.xbox.getRawAxis(XboxController.Axis.kLeftX.value), 
        () -> OperatorConstants.xbox.getRawAxis(XboxController.Axis.kRightX.value), 
        () -> false)
    );

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  
    Constants.OperatorConstants.button1.whileTrue(new RainbowLEDCommand());
    Constants.OperatorConstants.button2.whileTrue(new GhostWhiteLEDCommand());
    Constants.OperatorConstants.button3.whileTrue(new IntakeLEDCommand());
    Constants.OperatorConstants.button4.whileTrue(new WarningPulseLEDCommand());
    Constants.OperatorConstants.button5.whileTrue(new ClimbersLEDCommand());
    Constants.OperatorConstants.button6.whileTrue(new FunEffectLEDCommand());
    Constants.OperatorConstants.button7.whileTrue(new BenPeiLEDCommand());
    Constants.OperatorConstants.button8.whileTrue(new RedBluNonContGradLEDCommand());
    Constants.OperatorConstants.button9.whileTrue(new DenimLEDCommand());
    Constants.OperatorConstants.button10.whileTrue(new BluGrenConGradientLEDCommand());
    Constants.OperatorConstants.button11.whileTrue(new GoldLEDCommand());
    Constants.OperatorConstants.button12.whileTrue(new YelRedConGradientLEDCommand());

    // Constants.OperatorConstants.button2.whileTrue(new InstantCommand(LEDSubsystem.getInstance() :: greenLED, LEDSubsystem.getInstance()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return autoChooser.getSelected();
    // return Autos.exampleAuto(exampleSubsystem);
    // return null;
  }
}
