package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;


public class DriveTrain {

    private static final int ESQUERDA_FRENTE_ID = 1;
    private static final int ESQUERDA_TRAS_ID = 2;
    private static final int DIREITA_FRENTE_ID = 3;
    private static final int DIREITA_TRAS_ID = 4;

    private static final boolean INVERTER_ESQUERDA = false; 
    private static final boolean INVERTER_DIREITA = true;

    private static final double ZONA_MORTA = 0.08;
    private static final double VELOCIDADE_MAX = 0.65; 
    private static final double GIRO_MAX = 0.6;       

    private final VictorSPX esquerdaFrente = new VictorSPX(ESQUERDA_FRENTE_ID);
    private final VictorSPX esquerdaTras = new VictorSPX(ESQUERDA_TRAS_ID);
    private final VictorSPX direitaFrente = new VictorSPX(DIREITA_FRENTE_ID);
    private final VictorSPX direitaTras = new VictorSPX(DIREITA_TRAS_ID);

    public DriveTrain() {
        configurar(esquerdaFrente, INVERTER_ESQUERDA);
        configurar(esquerdaTras, INVERTER_ESQUERDA);
        configurar(direitaFrente, INVERTER_DIREITA);
        configurar(direitaTras, INVERTER_DIREITA);
    }

    private void configurar(VictorSPX motor, boolean inverter) {
        motor.configFactoryDefault();
        motor.setInverted(inverter);
        motor.setNeutralMode(NeutralMode.Brake);
    }


    public void mover(double frente, double giro) {
        double esq = frente + giro;
        double dir = frente - giro;
        double maior = Math.max(Math.abs(esq), Math.abs(dir));
        if (maior > 1) {
            esq /= maior;
            dir /= maior;
        }
        esquerdaFrente.set(ControlMode.PercentOutput, esq);
        esquerdaTras.set(ControlMode.PercentOutput, esq);
        direitaFrente.set(ControlMode.PercentOutput, dir);
        direitaTras.set(ControlMode.PercentOutput, dir);
    }


    public void periodic(XboxController controle) {
        double frente = MathUtil.applyDeadband(-controle.getLeftY(), ZONA_MORTA) * VELOCIDADE_MAX;
        double giro = MathUtil.applyDeadband(-controle.getRightX(), ZONA_MORTA) * GIRO_MAX;
        mover(frente, -giro);
    }
}
