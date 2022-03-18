package main.java.frc.robot;

   // x lowers and y raises, reset when we starts
    import edu.wpi.first.wpilibj.XboxController;
    import edu.wpi.first.wpilibj.DriverStation;
    import com.revrobotics.SparkMaxPIDController;
    import com.revrobotics.CANSparkMax;
    import com.revrobotics.CANSparkMax.ControlType;
    import com.revrobotics.CANSparkMaxLowLevel.MotorType;
    import java.util.Map;

    public class Endgame {

    private CANSparkMax leftEndgame;
    private CANSparkMax rightEndgame;

    private static final double ENDGAME_HIGH = 0.5;
    
    private enum EndgameMode {
        IDLE,
        RAISE,
        LOWER,
    }

    private EndgameMode endgameMode = EndgameMode.IDLE;

    private double endgameSpeed = 0.0;

    public Endgame() {
        leftEndgame = new CANSparkMax(PortMap.CAN.LEFT_ENDGAME, MotorType.kBrushless);
        rightEndgame = new CANSparkMax(PortMap.Can.RIGHT_ENDGAME, MotorType.kBrushless);

        leftEndgame.restoreFactoryDefaults();
        rightEndgame.restoreFactoryDefaults();

        leftEndgame.setInverted(false);
        rightEndgame.setInverted(false);

    }

    public void init() {
        stop();
        leftEndgame.restoreFactoryDefaults();
        rightEndgame.restoreFactoryDefaults();
        EndgameMode = EndgameMode.IDLE;
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

        if(EndgameMode == EndgameMode.RAISE) {
            endgameSpeed = ENDGAME_HIGH;
            endgameMode = EndgameMode.IDLE;
    }

        if(EndgameMode == EndgameMode.LOWER) {
            endgameSpeed = -ENDGAME_HIGH;
            endgameMode = EndgameMode.IDLE;
        }
    }
  
}
