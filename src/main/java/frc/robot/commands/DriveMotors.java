package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotorsSubsystem;

public class DriveMotors extends Command {
    
    private final TestMotorsSubsystem motorsSub;
    private final double speed;

    public DriveMotors(TestMotorsSubsystem p_motorsSubsystem, double p_speed) {
        motorsSub = p_motorsSubsystem;
        speed = p_speed;
        addRequirements(motorsSub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        motorsSub.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) { 
        motorsSub.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
