// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
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

  private final CommandXboxController m_driverController =
      new CommandXboxController(OI.Constants.k_driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initSubystems();
    configureBindings();
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
    } // extend if-else chain for other subsystems
  }

  
  private void configureBindings() {
    //apply later
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
