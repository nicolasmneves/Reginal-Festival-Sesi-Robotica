package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;


public class GarraIntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(25);
  private final TalonFX intakeFollowerMotor = new TalonFX(51);


  private final Mechanism2d m_mech = new Mechanism2d(40, 40);
  private final MechanismRoot2d m_root = m_mech.getRoot("CentroIntake", 20, 20);
  private final MechanismLigament2d m_roloLigament = m_root.append(
      new MechanismLigament2d("RoloGiro", 10, 0, 8, new Color8Bit(Color.kCrimson))
  );
  

  private double m_roloAnguloSimulado = 0.0;

  String state = "START";

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.3, 0))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withOpenLoopRampRate(Seconds.of(0.1))
      .withFollowers(new Pair<Object, Boolean>(intakeFollowerMotor, true));

  private final SmartMotorController motor = new TalonFXWrapper(intakeMotor, DCMotor.getKrakenX44(2), motorConfig);
  private final FlyWheelConfig intakeConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RPM.of(6000));

  private final FlyWheel intake = new FlyWheel(intakeConfig, motor);

  public GarraIntakeSubsystem() {

    SmartDashboard.putData("IntakeMech/mechanism", m_mech);
  }

  public AngularVelocity getVelocity() {
    return intake.getSpeed();
  }

  public Command setDutyCycle(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return intake.set(dutyCycle);
  }

  public Command in() {
    return setDutyCycle(0.8);
  }

  public Command out() {
    return setDutyCycle(-0.8);
  }

  public Command stop() {
    return setDutyCycle(0.0).alongWith(new InstantCommand(() -> state = "STOPED"));
  }

  public Command setRPM(double rpm) {
    return intake.run(RPM.of(rpm));
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();

    double rpmAtual = intake.getSpeed().in(RPM);

    m_roloAnguloSimulado += (rpmAtual * 6.0) * 0.020;

    m_roloAnguloSimulado = m_roloAnguloSimulado % 360.0;

    m_roloLigament.setAngle(m_roloAnguloSimulado);
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
} 
