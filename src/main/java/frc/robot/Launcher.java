package frc.robot;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Launcher implements ILauncher {
  
    private Mode mode;
    private Runnable currentCompletionRoutine;

    private CANSparkMax intakeMotor;
    private CANSparkMax shooterMotor;
    private CANSparkMax storageMotor;

    private SparkMaxPIDController shooterPIDController;

    private enum ShooterMode {
        IDLE,
        SHOOT,
        REVERSE,
    }

    private ShooterMode shooterMode = ShooterMode.IDLE;

    private double intakeSpeed = 0.0;
    private double storageSpeed = 0.0;

    private static final double INTAKE_HIGH = 0.45;
    private static final double STORAGE_HIGH = 0.5;
    private static final double STORAGE_REVERSE_SPEED_HIGH = 0.2;
    private static final double SHOOTER_SPEED_BACKWARDS = 0.5;

    private static final double AUTO_SHOOTER_ROTATIONS = 200;
    private static final double AUTO_STORAGE_ROTATIONS = 25;

    private double maxOutput = 1.0;
    private double minOutput = 0.0;
    private double shooterSetPoint = 0.0;
    private double maxRPM = 2000;

    private enum StorageMode {
        IDLE,
        STORAGE_INTAKE,
        STORAGE_INTAKE_REVERSE,
        ADVANCE,
        REVERSE,
    }

    private StorageMode storageMode = StorageMode.IDLE;

    public Launcher() {

        intakeMotor = new CANSparkMax(PortMap.CAN.INTAKE_MOTOR_CONTROLLER, MotorType.kBrushed);
        storageMotor = new CANSparkMax(PortMap.CAN.STORAGE_MOTOR_CONTROLLER, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(PortMap.CAN.SHOOTER_MOTOR_CONTROLLER, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        storageMotor.restoreFactoryDefaults();
        shooterMotor.restoreFactoryDefaults();
        
        shooterMotor.setInverted(false);
        shooterPIDController = shooterMotor.getPIDController();
        shooterPIDController.setOutputRange(minOutput, maxOutput);
        shooterPIDController.setP(.001);
        shooterPIDController.setD(0);
        shooterPIDController.setI(0);
        shooterPIDController.setFF(.000185);
    }

    @Override
    public void init() {
        stop();
        storageMotor.restoreFactoryDefaults();
        storageMode = StorageMode.IDLE;
        shooterMode = ShooterMode.IDLE;
    }

    @Override
    public void stop() {
        intakeSpeed = 0.0;
        storageSpeed = 0.0;
        shooterSetPoint = 0.0;
    }

    @Override
    public void intake() {
        mode = Mode.MANUAL;
        storageMode = StorageMode.STORAGE_INTAKE;
    }

    @Override
    public void intakeReverse() {
        mode = Mode.MANUAL;
        storageMode = StorageMode.STORAGE_INTAKE_REVERSE;
    }

    @Override
    public void advance() {
        mode = Mode.MANUAL;
        storageMode = StorageMode.ADVANCE;
    }

    @Override
    public void reverse() {
        mode = Mode.MANUAL;
        storageMode = StorageMode.REVERSE;
    }

    @Override
    public void shoot() {
        mode = Mode.MANUAL;
        shooterMode = ShooterMode.SHOOT;
    }

    @Override
    public void shooterReverse() {
        mode = Mode.MANUAL;
        shooterMode = ShooterMode.REVERSE;
    }

    @Override
    public void autoShoot(Runnable completionRoutine) {
        mode = Mode.AUTO;
        storageMotor.getEncoder().setPosition(0.0);
        shooterMotor.getEncoder().setPosition(0.0);
        currentCompletionRoutine = completionRoutine;
    }

    private void handleActionEnd() {
        stop();
        
        if (currentCompletionRoutine != null) {
            Runnable oldCompletionRoutine = currentCompletionRoutine;
            currentCompletionRoutine = null;
            oldCompletionRoutine.run();
        }
    }

    @Override
    public void periodic() {
        stop();
        if (mode == Mode.MANUAL) {

            if (storageMode == StorageMode.STORAGE_INTAKE) {
                intakeSpeed = INTAKE_HIGH;
                storageMode = StorageMode.IDLE;
            } else if (storageMode == StorageMode.STORAGE_INTAKE_REVERSE) {
                intakeSpeed = -INTAKE_HIGH;
                storageMode = StorageMode.IDLE;
            } else if (storageMode == StorageMode.ADVANCE) {
                storageSpeed = STORAGE_HIGH;
                storageMode = StorageMode.IDLE;
            } else if (storageMode == StorageMode.REVERSE) {
                storageSpeed = -STORAGE_REVERSE_SPEED_HIGH;
                storageMode = StorageMode.IDLE;
            }

            if(shooterMode == ShooterMode.SHOOT) {
                shooterSetPoint = maxRPM;
                shooterMode = ShooterMode.IDLE;
                //Debug.logPeriodic("shooter is spinning");
            } else if (shooterMode == ShooterMode.REVERSE) {
                Debug.logPeriodic("Shooter is spinning in reverse");
                shooterMode = ShooterMode.IDLE;
                shooterMotor.set(-SHOOTER_SPEED_BACKWARDS);
            }
            
            //Debug.logPeriodic("shooter motor RPM " + shooterMotor.getEncoder().getVelocity());

        } else if (mode == Mode.AUTO) {
            
            //Debug.log("AUTO shooter");
            if (shooterMotor.getEncoder().getPosition() > AUTO_SHOOTER_ROTATIONS) {
            } else {
                shooterSetPoint = maxRPM;
                //Debug.log("shooter is spinning");
            }

            if (shooterMotor.getEncoder().getVelocity() >= maxRPM - 100) {
                if (storageMotor.getEncoder().getPosition() > AUTO_STORAGE_ROTATIONS) {
                    handleActionEnd();
                } else {
                    storageSpeed = STORAGE_HIGH;
                }
    
            }

            //shooterSetPoint = maxRPM;
            //storageSpeed = STORAGE_HIGH;

            Debug.logPeriodic("Shooter motor rotations" + shooterMotor.getEncoder().getPosition());
            Debug.logPeriodic("Shooter motor speed" + shooterMotor.getEncoder().getVelocity());
            Debug.logPeriodic("Storage motor rotations" + storageMotor.getEncoder().getPosition());
        } 
        intakeMotor.set(intakeSpeed);
        storageMotor.set(storageSpeed);
        shooterPIDController.setReference(shooterSetPoint, ControlType.kVelocity);
    }   
}