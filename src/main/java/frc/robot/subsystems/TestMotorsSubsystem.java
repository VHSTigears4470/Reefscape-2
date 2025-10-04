package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorsSubsystem extends SubsystemBase{

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final DigitalInput topSwitch;
    private final DigitalInput botSwitch;

    public TestMotorsSubsystem(int id1, int id2) {
        motor1 = new SparkMax(id1, MotorType.kBrushless);
        motor2 = new SparkMax(id2, MotorType.kBrushless);
        topSwitch = new DigitalInput(0);
        botSwitch = new DigitalInput(1);
    }

    public void setSpeed(double speed) {
        speed = Math.signum(speed) * Math.min(Math.max(Math.abs(speed), 0), 1);
        motor1.set(speed);
        motor2.set(speed);
    }

    public void stopMotor() {
        motor1.stopMotor();
        motor2.stopMotor();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run;
        SmartDashboard.putBoolean("Top Switch", topSwitch.get());
        SmartDashboard.putBoolean("Bot Switch", botSwitch.get());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
