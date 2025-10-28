// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.commands.TestElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

//add PathPlanner stuff later
public class RobotContainer {
  private DriveSubsystem m_driveSub;
  private ElevatorSubsystem m_elevatorSub;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OI.Constants.k_driverControllerPort);
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initSubystems();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    //Add paths here
    autoChooser.addOption("Forward Right", new PathPlannerAuto("Forward Right"));
    autoChooser.addOption("Test", new PathPlannerAuto("Test"));
    autoChooser.addOption("Spin", new PathPlannerAuto("Spin"));
    autoChooser.addOption("Circle", new PathPlannerAuto("Circle"));
    SmartDashboard.putData("Auto Mode", autoChooser);
    
    configureBindings();

    FollowPathCommand.warmupCommand().schedule();
  }

  public void initSubystems() {
    if(Operating.Constants.k_usingDrive) {
      m_driveSub = new DriveSubsystem();
      m_driveSub.setDefaultCommand(new RunCommand(
        () -> m_driveSub.drive(
                  OI.Constants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OI.Constants.k_driverAxisY), OI.Constants.k_driveDeadband),
                  OI.Constants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OI.Constants.k_driverAxisX), OI.Constants.k_driveDeadband),
                  OI.Constants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OI.Constants.k_driverAxisRot), OI.Constants.k_driveDeadband), 
                  true,
                  "Default / Field Oriented"
        ),
        m_driveSub));
    } 
    if(Operating.Constants.k_usingElevator) {
      m_elevatorSub = new ElevatorSubsystem();
    } 
    // extend if-else chain for other subsystems
  }

  
  private void configureBindings() {
    int preset = 0;
    switch (preset){ //Add other controller schemes later
      case 1: //Elevator debugging
        m_driverController.y().onTrue(new TestElevator(m_elevatorSub, .1));
        m_driverController.x().onTrue(new TestElevator(m_elevatorSub, -.1));
        break;
      default: //Main controller scheme
        if(Operating.Constants.k_usingElevator){
          m_driverController.y().onTrue(m_elevatorSub.setSetpointCommand(Setpoint.kFeederStation));
          m_driverController.x().onTrue(m_elevatorSub.setSetpointCommand(Setpoint.kLevel1));
          m_driverController.b().onTrue(m_elevatorSub.setSetpointCommand(Setpoint.kLevel2));
          m_driverController.a().onTrue(m_elevatorSub.setSetpointCommand(Setpoint.kLevel3));
          m_driverController.rightBumper().onTrue(m_elevatorSub.setSetpointCommand(Setpoint.kLevel4));
        }
    }    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
