package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // ========== MEDIDAS DO ELEVADOR ==========
    private static final double REDUCTION = 10.0;            
    private static final double SPOOL_DIAMETER_CM = 4.0;     // diametro da polia/roda dentada (cm)
    private static final double STAGE_MULTIPLIER = 1.0;     

    private static final double CM_PER_MOTOR_ROTATION =
        (Math.PI * SPOOL_DIAMETER_CM / REDUCTION) * STAGE_MULTIPLIER;

    // ========== CURSO (em cm) ==========
    private static final double MIN_HEIGHT_CM = 0.0;   
    private static final double MAX_HEIGHT_CM = 35.0;  
    private static final double TOLERANCE_CM = 1.0;

    private static final int MOTOR_ID = 13;
    private static final double MANUAL_SPEED = 0.3;

    // ========== FINS DE CURSO ==========
    private static final int BOTTOM_LIMIT_DIO = 0;  
    private static final int TOP_LIMIT_DIO = 1;   


    private static final boolean BOTTOM_LIMIT_INVERTED = false;
    private static final boolean TOP_LIMIT_INVERTED = false;

    private static final double DEFAULT_kP = 0.02;
    private static final double DEFAULT_kI = 0.0;
    private static final double DEFAULT_kD = 0.0;
    private static final double DEFAULT_kG = 0.0;                   
    private static final double DEFAULT_MAX_VELOCITY = 50.0;         
    private static final double DEFAULT_MAX_ACCELERATION = 100.0;   
    private static final double DEFAULT_TEST_TARGET_CM = 10.0;


    private static final String KEY_KP = "Elevator/Tuning/kP";
    private static final String KEY_KI = "Elevator/Tuning/kI";
    private static final String KEY_KD = "Elevator/Tuning/kD";
    private static final String KEY_KG = "Elevator/Tuning/kG";
    private static final String KEY_VEL = "Elevator/Tuning/MaxVelocityCmS";
    private static final String KEY_ACC = "Elevator/Tuning/MaxAccelerationCmS2";
    private static final String KEY_TARGET = "Elevator/Tuning/TestTargetCm";

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final DigitalInput bottomLimit = new DigitalInput(BOTTOM_LIMIT_DIO);
    private final DigitalInput topLimit = new DigitalInput(TOP_LIMIT_DIO);


    private double kP = DEFAULT_kP;
    private double kI = DEFAULT_kI;
    private double kD = DEFAULT_kD;
    private double kG = DEFAULT_kG;
    private double maxVelocity = DEFAULT_MAX_VELOCITY;
    private double maxAcceleration = DEFAULT_MAX_ACCELERATION;

    private double targetPositionCm = MIN_HEIGHT_CM;

    public ElevatorSubsystem() {
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        config.smartCurrentLimit(40);

        config.encoder
            .positionConversionFactor(CM_PER_MOTOR_ROTATION)
            .velocityConversionFactor(CM_PER_MOTOR_ROTATION / 60.0);

        config.softLimit
            .reverseSoftLimit(MIN_HEIGHT_CM)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(MAX_HEIGHT_CM)
            .forwardSoftLimitEnabled(true);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

        config.closedLoop.maxMotion
            .cruiseVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
            .allowedProfileError(TOLERANCE_CM, ClosedLoopSlot.kSlot0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        encoder.setPosition(MIN_HEIGHT_CM);
        targetPositionCm = MIN_HEIGHT_CM;

        SmartDashboard.setDefaultNumber(KEY_KP, kP);
        SmartDashboard.setDefaultNumber(KEY_KI, kI);
        SmartDashboard.setDefaultNumber(KEY_KD, kD);
        SmartDashboard.setDefaultNumber(KEY_KG, kG);
        SmartDashboard.setDefaultNumber(KEY_VEL, maxVelocity);
        SmartDashboard.setDefaultNumber(KEY_ACC, maxAcceleration);
        SmartDashboard.setDefaultNumber(KEY_TARGET, DEFAULT_TEST_TARGET_CM);
    }

    @Override
    public void periodic() {
        updateTunableGains();


        if (isAtBottom()) {
            encoder.setPosition(MIN_HEIGHT_CM);
        }

        SmartDashboard.putNumber("Elevator/PositionCm", encoder.getPosition());
        SmartDashboard.putNumber("Elevator/TargetCm", targetPositionCm);
        SmartDashboard.putNumber("Elevator/VelocityCmS", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator/Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/AppliedOutput", motor.getAppliedOutput());
        SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
        SmartDashboard.putBoolean("Elevator/BottomLimit", isAtBottom());
        SmartDashboard.putBoolean("Elevator/TopLimit", isAtTop());
    }

    // ---------- Fins de curso ----------

    public boolean isAtBottom() {
        return bottomLimit.get() != BOTTOM_LIMIT_INVERTED;
    }

    public boolean isAtTop() {
        return topLimit.get() != TOP_LIMIT_INVERTED;
    }



    private void updateTunableGains() {
        double newP = SmartDashboard.getNumber(KEY_KP, kP);
        double newI = SmartDashboard.getNumber(KEY_KI, kI);
        double newD = SmartDashboard.getNumber(KEY_KD, kD);
        double newVel = SmartDashboard.getNumber(KEY_VEL, maxVelocity);
        double newAcc = SmartDashboard.getNumber(KEY_ACC, maxAcceleration);

        kG = SmartDashboard.getNumber(KEY_KG, kG);

        boolean changed = newP != kP || newI != kI || newD != kD
                || newVel != maxVelocity || newAcc != maxAcceleration;

        if (changed) {
            kP = newP;
            kI = newI;
            kD = newD;
            maxVelocity = newVel;
            maxAcceleration = newAcc;

            SparkMaxConfig cfg = new SparkMaxConfig();
            cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
            cfg.closedLoop.maxMotion
                .cruiseVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
                .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0);

            motor.configure(
                cfg,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );
        }
    }



    public void setHeightCm(double targetCm) {
        targetPositionCm = MathUtil.clamp(targetCm, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
        double current = encoder.getPosition();


        if (isAtBottom() && targetPositionCm <= current) {
            motor.stopMotor();
            return;
        }


        if (isAtTop() && targetPositionCm >= current) {
            motor.stopMotor();
            return;
        }

        controller.setSetpoint(
            targetPositionCm,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            kG,
            ArbFFUnits.kVoltage
        );
    }



    public void elevatorUp() {
        if (isAtTop()) {
            motor.stopMotor();
        } else {
            motor.set(MANUAL_SPEED);
        }
    }

    public void elevatorDown() {
        if (isAtBottom()) {
            motor.stopMotor();
        } else {
            motor.set(-MANUAL_SPEED);
        }
    }

    public void elevatorStop() {
        motor.stopMotor();
        targetPositionCm = encoder.getPosition();
    }

  
    public void zeroEncoder() {
        encoder.setPosition(MIN_HEIGHT_CM);
        targetPositionCm = MIN_HEIGHT_CM;
    }

    // ---------- Comandos ----------

    public Command goToHeightCommand(double heightCm) {
        return this.run(() -> setHeightCm(heightCm));
    }

    public Command goToHeightAndWaitCommand(double heightCm) {
        return goToHeightCommand(heightCm).until(this::atSetpoint);
    }


    public Command goToTestTargetCommand() {
        return this.run(() ->
            setHeightCm(SmartDashboard.getNumber(KEY_TARGET, DEFAULT_TEST_TARGET_CM)));
    }


    public Command upCommand() {
        return this.runEnd(this::elevatorUp, this::elevatorStop);
    }

    public Command downCommand() {
        return this.runEnd(this::elevatorDown, this::elevatorStop);
    }

    // ---------- Estado ----------

    /** Chegou no alvo, ou parou no fim de curso na direcao do alvo. */
    public boolean atSetpoint() {
        double current = encoder.getPosition();
        boolean inTolerance = Math.abs(current - targetPositionCm) < TOLERANCE_CM;
        boolean stoppedAtTop = isAtTop() && targetPositionCm >= current;
        boolean stoppedAtBottom = isAtBottom() && targetPositionCm <= current;
        return inTolerance || stoppedAtTop || stoppedAtBottom;
    }

    public double getHeightCm() {
        return encoder.getPosition();
    }
}