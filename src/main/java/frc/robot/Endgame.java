package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Endgame implements IEndgame {

    private CANSparkMax leftEndgame;
    private CANSparkMax rightEndgame;

    private static final double ENDGAME_HIGH = 0.1;

    private enum EndgameMode {
        IDLE,
        RAISE,
        LOWER,
    }

    private EndgameMode endgameMode = EndgameMode.IDLE;

    private double endgameSpeed = 0.0;

    public Endgame() {
        leftEndgame = new CANSparkMax(PortMap.CAN.ENDGAME_LEFT_MOTOR, MotorType.kBrushless);
        rightEndgame = new CANSparkMax(PortMap.CAN.ENDGAME_RIGHT_MOTOR, MotorType.kBrushless);

        leftEndgame.restoreFactoryDefaults();
        rightEndgame.restoreFactoryDefaults();

        leftEndgame.setInverted(false);
        rightEndgame.setInverted(false);

    }

    public void init() {
        stop();
        leftEndgame.restoreFactoryDefaults();
        rightEndgame.restoreFactoryDefaults();
        endgameMode = EndgameMode.IDLE;
    }

    public void stop() {
        endgameSpeed = 0.0;
    }
        
    public void raise() {
        endgameMode = EndgameMode.RAISE;
    }

    public void lower() {
        endgameMode= EndgameMode.LOWER;
    }

    public void periodic() {
        stop();

        if(endgameMode == EndgameMode.RAISE) {
            endgameSpeed = ENDGAME_HIGH;
            endgameMode = EndgameMode.IDLE;
        }

        if(endgameMode == EndgameMode.LOWER) {
            endgameSpeed = -ENDGAME_HIGH;
            endgameMode = EndgameMode.IDLE;
        }

        leftEndgame.set(endgameSpeed);
        rightEndgame.set(endgameSpeed);
    }
  
}
