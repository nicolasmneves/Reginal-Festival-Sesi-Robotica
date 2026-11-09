package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GarraIntakeSubsystem02 extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(5);

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void in() {
        setSpeed(0.8);
    }

    public void out() {
        setSpeed(-0.8);
    }

    public void stop() {
        setSpeed(0.0);
    }
}