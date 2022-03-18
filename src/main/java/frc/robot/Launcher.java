package frc.robot;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Launcher implements ILauncher {

    private CANSparkMax intakeMotor;
    private CANSparkMax shooterMotor;
    private CANSparkMax storageMotor;

    private SparkMaxPIDController shooterPIDController;

    private enum ShooterMode {
        IDLE,
        SHOOT,
    }

    private ShooterMode shooterMode = ShooterMode.IDLE;

    private double intakeSpeed = 0.0;
    private double storageSpeed = 0.0;

    private static final double INTAKE_HIGH = 0.45;
    private static final double STORAGE_HIGH = 0.5;

    private double maxOutput = 1.0;
    private double minOutput = 0.0;
    private double shooterSetPoint = 0.0;
    private double maxRPM = 3000;

    private enum StorageMode {
        IDLE,
        STORAGE_INTAKE,
        ADVANCE,
        REVERSE,
    }

    private StorageMode storageMode = StorageMode.IDLE;

    public Launcher() {

        intakeMotor = new CANSparkMax(PortMap.CAN.INTAKE_MOTOR_CONTROLLER, MotorType.kBrushed);
        storageMotor = new CANSparkMax(PortMap.CAN.STORAGE_MOTOR_CONTROLLER, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(PortMap.CAN.SHOOTER_MOTOR_CONTROLLER, MotorType.kBrushless);
        //the stuff below is old from last year
        // storageSwitch = new DigitalInput(PortMap.DIO.BOTTOM_STORAGE);
        
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
        storageMode = StorageMode.STORAGE_INTAKE;
    }

    @Override
    public void advance() {
        storageMode = StorageMode.ADVANCE;
    }

    @Override
    public void reverse() {
        storageMode = StorageMode.REVERSE;
    }

    @Override
    public void shoot() {
        shooterMode = ShooterMode.SHOOT;
    }

@Override
    public void periodic() {
        stop();

        if(storageMode == StorageMode.STORAGE_INTAKE) {

            /*if(storageMotor.getEncoder().getPosition() > INITIAL_INTAKE_ROTATIONS) {
                storageMode = StorageMode.IDLE;
                storageMotor.restoreFactoryDefaults();
            } else {
                intakeSpeed = INTAKE_HIGH;
                storageSpeed = STORAGE_HIGH;
            }*/
            intakeSpeed = INTAKE_HIGH;
            storageMode = StorageMode.IDLE;

        } else if(storageMode == StorageMode.ADVANCE) {

            /*if(storageMotor.getEncoder().getPosition() > ADVANCE_ONE_BALL_ROTATIONS) {
                storageMode = StorageMode.IDLE;
                storageMotor.restoreFactoryDefaults();
            } else {
                storageSpeed = STORAGE_HIGH;
            }*/
            storageSpeed = STORAGE_HIGH;
            storageMode = StorageMode.IDLE;

        } else if(storageMode == StorageMode.REVERSE) {

            /*if(Math.abs(storageMotor.getEncoder().getPosition()) < ADVANCE_ONE_BALL_ROTATIONS) {
                storageSpeed = -STORAGE_HIGH;
            } else {
                storageMode = StorageMode.IDLE;
                storageMotor.restoreFactoryDefaults();
            }*/
            storageSpeed = -STORAGE_HIGH;
            storageMode = StorageMode.IDLE;
        }
        
        if(shooterMode == ShooterMode.SHOOT) {

            shooterSetPoint = maxRPM;
            shooterMode = ShooterMode.IDLE;

        }

        intakeMotor.set(intakeSpeed);
        storageMotor.set(storageSpeed);
        shooterPIDController.setReference(shooterSetPoint, ControlType.kVelocity);
    }
}