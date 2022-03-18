package frc.robot;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import java.util.Map;

public class Launcher implements ILauncher {

    private VictorSPX intakeMotor;
    private Mode mode;

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
    private double shooterSpeed = 0.0;

    private static final double INTAKE_HIGH = 0.5;
    private static final double STORAGE_HIGH = 0.5;
    private static final double SHOOTER_HIGH = 0.5;
    
    private static final double INITIAL_INTAKE_ROTATIONS = 30000;
    private static final double ADVANCE_ONE_BALL_ROTATIONS = 20000;
   
    private static final double AUTO_SHOOTER_ROTATIONS = 10000;
    private static final double AUTO_STORAGE_ROTATIONS = 50;

    // TODO: find actual values 

    private double maxOutput = 1.0;
    private double minOutput = 0.0;
    private double shooterSetPoint = 0.0;
    private double maxRPM = 0.3;

    private enum StorageMode {
        IDLE,
        STORAGE_INTAKE,
        ADVANCE,
        REVERSE,
    }

    private StorageMode storageMode = StorageMode.IDLE;

    public Launcher() {

        intakeMotor = new VictorSPX(PortMap.CAN.INTAKE_MOTOR_CONTROLLER);
        storageMotor = new CANSparkMax(PortMap.CAN.STORAGE_MOTOR_CONTROLLER, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(PortMap.CAN.SHOOTER_MOTOR_CONTROLLER, MotorType.kBrushless);
        //the stuff below is old from last year
        // storageSwitch = new DigitalInput(PortMap.DIO.BOTTOM_STORAGE);

        shooterMotor.setInverted(true);
        shooterPIDController = shooterMotor.getPIDController();
        shooterPIDController.setOutputRange(minOutput, maxOutput);
        // setDistance(228.0);
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
    public void autoShoot() {
        mode = Mode.AUTO;

        shooterMode = ShooterMode.SHOOT;
        storageMode = StorageMode.ADVANCE;
    }


@Override
    public void periodic() {
        stop();
        if(mode == Mode.MANUAL) {
            if(storageMode == StorageMode.STORAGE_INTAKE) {

                if(storageMotor.getEncoder().getPosition() > INITIAL_INTAKE_ROTATIONS) {
                    storageMode = StorageMode.IDLE;
                    storageMotor.restoreFactoryDefaults();
                } else {
                    intakeSpeed = INTAKE_HIGH;
                    storageSpeed = STORAGE_HIGH;
                }

            } else if(storageMode == StorageMode.ADVANCE) {

                if(storageMotor.getEncoder().getPosition() > ADVANCE_ONE_BALL_ROTATIONS) {
                    storageMode = StorageMode.IDLE;
                    storageMotor.restoreFactoryDefaults();
                } else {
                    storageSpeed = STORAGE_HIGH;
                }

            } else if(storageMode == StorageMode.REVERSE) {

                if(Math.abs(storageMotor.getEncoder().getPosition()) > ADVANCE_ONE_BALL_ROTATIONS) {
                    storageMode = StorageMode.IDLE;
                    storageMotor.restoreFactoryDefaults();
                } else {
                    storageSpeed = -STORAGE_HIGH;
                }

        if(storageMode == StorageMode.STORAGE_INTAKE) {

            if(storageMotor.getEncoder().getPosition() > INITIAL_INTAKE_ROTATIONS) {
                storageMode = StorageMode.IDLE;
                storageMotor.restoreFactoryDefaults();
            } else {
                intakeSpeed = INTAKE_HIGH;
                storageSpeed = STORAGE_HIGH;
            }
            
            if(shooterMode == ShooterMode.SHOOT) {

                shooterSetPoint = maxRPM;
                shooterMode = ShooterMode.IDLE;

            }

            intakeMotor.set(intakeSpeed);
            storageMotor.set(storageSpeed);
            shooterPIDController.setReference(shooterSetPoint, ControlType.kVelocity);

        } else if (mode == Mode.AUTO) {
    
            if (storageMotor.getEncoder().getPosition() > AUTO_STORAGE_ROTATIONS) {
                storageMode = StorageMode.IDLE;
                storageMotor.restoreFactoryDefaults();
            } else {
                storageSpeed = STORAGE_HIGH;
            }

            if(shooterMotor.getEncoder().getPosition() > AUTO_SHOOTER_ROTATIONS) {
                shooterMode = ShooterMode.IDLE;
                shooterMotor.restoreFactoryDefaults();
            } else shooterSpeed = SHOOTER_HIGH; 

            intakeMotor.set(intakeSpeed);
            storageMotor.set(storageSpeed);
            shooterPIDController.setReference(shooterSetPoint, ControlType.kVelocity);

        } else {
            throw new IllegalArgumentException("The launcher controllers are in an invalid launcher mode.");
        }

        intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
        storageMotor.set(storageSpeed);
        shooterPIDController.setReference(shooterSetPoint, ControlType.kVelocity);
    }
    }   
}