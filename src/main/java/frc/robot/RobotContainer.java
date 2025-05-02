// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.OperatingSystemMXBean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagAuto;
import frc.robot.commands.DriveForwardAuto;
import frc.robot.commands.Algae.DeployAlgaeIntake;
import frc.robot.commands.Algae.RetractAlgaeIntake;
//import frc.robot.commands.Algae.ReverseAlgaeIntake;
import frc.robot.commands.Algae.RunAlgaeIntake;
import frc.robot.commands.Armevator.ArmToPosition;
import frc.robot.commands.Armevator.ArmevatorToPosition;
import frc.robot.commands.Armevator.ElevatorToPosition;
import frc.robot.commands.Armevator.RunArm;
import frc.robot.commands.Armevator.RunElevator;
import frc.robot.commands.Coral.IntakeCoral;
import frc.robot.commands.Coral.ReverseCoralIntake;
import frc.robot.commands.Swerve.AutoAlignReef;
import frc.robot.commands.Swerve.AutoPosition;
import frc.robot.commands.Swerve.CrabWalk;
import frc.robot.commands.Swerve.SwerveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AprilTagAuto;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();

  
    private final SendableChooser<Command> autoChooser;

    //private final AprilTagAuto autoAprilTag = ;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSub.setDefaultCommand(
      new SwerveCommand(
        () -> Constants.OperatorConstants.xbox.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> Constants.OperatorConstants.xbox.getRawAxis(XboxController.Axis.kLeftX.value), 
        () -> OperatorConstants.xbox.getRawAxis(XboxController.Axis.kRightX.value), 
        () -> true) //!OperatorConstants.xbox.getYButton()
    );

    NamedCommands.registerCommand("Armevator0", new ArmevatorToPosition(0));
    NamedCommands.registerCommand("ArmevatorIntake", new ArmevatorToPosition(1));
    NamedCommands.registerCommand("ArmevatorL2", new ArmevatorToPosition(2));
    NamedCommands.registerCommand("ArmevatorL3", new ArmevatorToPosition(3));
    NamedCommands.registerCommand("ArmevatorL4", new ArmevatorToPosition(4));

    NamedCommands.registerCommand("ElevatorUp", new RunElevator(true));

    NamedCommands.registerCommand("CoralEject", new ReverseCoralIntake());
    NamedCommands.registerCommand("CoralPickup", new IntakeCoral());

    NamedCommands.registerCommand(
      "DriveBack", 
      new ParallelRaceGroup(
        new WaitCommand(1.0),
        new SwerveCommand(
          () -> 0.15,
          () -> 0, 
          () -> 0,  
          () -> false)
      ));
  


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
    Constants.OperatorConstants.pancakeUp.whileTrue(new RunElevator(true));
    Constants.OperatorConstants.pancakeDown.whileTrue(new RunElevator(false));

    Constants.OperatorConstants.buttonRB.whileTrue(new AutoAlignReef(true));
    Constants.OperatorConstants.buttonLB.whileTrue(new AutoAlignReef(false));

    Constants.OperatorConstants.buttonX.onTrue(new InstantCommand(() -> swerveSub.resetGyro()));
    // Constants.OperatorConstants.buttonA.whileTrue(new AutoPosition());

    

    Constants.OperatorConstants.pancakeLeft.whileTrue(new RunArm(false));
    Constants.OperatorConstants.pancakeRight.whileTrue(new RunArm(true));

    Constants.OperatorConstants.button1.whileTrue(new IntakeCoral());
    Constants.OperatorConstants.button2.whileTrue(new ReverseCoralIntake());

    Constants.OperatorConstants.dPadUp.whileTrue(new CrabWalk(0));
    Constants.OperatorConstants.dPadRight.whileTrue(new CrabWalk(90));
    Constants.OperatorConstants.dPadDown.whileTrue(new CrabWalk(180));
    Constants.OperatorConstants.dPadLeft.whileTrue(new CrabWalk(270));

    OperatorConstants.button7.onTrue(new ArmevatorToPosition(0));
    OperatorConstants.button8.onTrue(new ArmevatorToPosition(1));
    OperatorConstants.button9.onTrue(new ArmevatorToPosition(2));
    OperatorConstants.button10.onTrue(new ArmevatorToPosition(3));
    OperatorConstants.button11.onTrue(new ArmevatorToPosition(4));

    OperatorConstants.button12.onTrue(new InstantCommand(() -> ArmSubsystem.getInstance().setArmEncoderPosition(0)));
    
    OperatorConstants.button4.whileTrue(new DeployAlgaeIntake());
    OperatorConstants.button3.whileTrue(new RetractAlgaeIntake());
    
    OperatorConstants.button6.whileTrue(new RunAlgaeIntake());
    //OperatorConstants.button5.whileTrue(new ReverseAlgaeIntake());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

      }
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return null;

   //return autoChooser.getSelected();
// drive back auto
    // return new ParallelRaceGroup(
    //   new WaitCommand(2),
    //   new SwerveCommand(
    //     () -> 0.15, 
    //     () -> 0, 
    //     () -> 0, 
    //     () -> true)
    // );

    // center L4 auto without auto align:
    
    // return new SequentialCommandGroup (
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(1.5), 
    //     new ArmevatorToPosition(4)
    //   ),
    //   new ParallelRaceGroup(
    //     new WaitCommand(2.2),
    //     new SwerveCommand(
    //       () -> -0.15,
    //       () -> 0, 
    //       () -> 0, 
    //       () -> true)
    //   ),
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(1), 
    //     new ReverseCoralIntake()
    //   ),
    //   new ParallelRaceGroup(
    //     new WaitCommand(1.0),
    //     new SwerveCommand(
    //       () -> 0.15,
    //       () -> 0, 
    //       () -> 0,  
    //       () -> true)
    //   ),
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(1.5), 
    //     new ArmevatorToPosition(0)
    //   )
    // );

    // center L4 auto with auto align:

    return new SequentialCommandGroup (
      // new ParallelDeadlineGroup(
      //   new WaitCommand(1.5), 
      //   new ArmevatorToPosition(4)
      // ),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new SwerveCommand(
          () -> -0.15,
          () -> 0, 
          () -> 0, 
          () -> true)
      ),
      new AutoAlignReef(true),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(1), 
      //   new ReverseCoralIntake()
      // ),
        new ParallelRaceGroup(
        new WaitCommand(1.0),
        new SwerveCommand(
          () -> 0.15,
          () -> 0, 
          () -> 0,  
          () -> true)
      )
      // new ParallelDeadlineGroup(
      //   new WaitCommand(1.5), 
      //   new ArmevatorToPosition(0)
      // )
    );
    
    // return autoChooser.getSelected();

    // return Autos.exampleAuto(exampleSubsystem);
  }
}

