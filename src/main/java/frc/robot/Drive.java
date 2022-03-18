package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive implements IDrive {

    private MecanumDrive driveBase;
    
    private Mode mode;
    private IGyroscopeSensor gyroscope;
    private Runnable currentCompletionRoutine;


    // Motors
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax rearLeftMotor;
    private CANSparkMax rearRightMotor;


    // PID (Proportional gain may need to be changed, add other gains if needed)
    private PIDController rotationController;
    private double setP = 0.6; // << same gain from 2020
    private double setI = 0.0;
    private double setD = 0.0;

    // Teleoperated
    private double forwardSpeed;
    private double strafeSpeed;
    private double angularSpeed;
    private double desiredAngle;

    // auto
    private double autoSpeed;
    private double autoAngleDegrees;
    private double desiredDistance;

    private static final double WHEEL_DIAMETER = 8.0;
    private static final double ENCODER_RESOLUTION = 2048.0;
    private static final double ROTATION_TOLERANCE_DEGREES = 2.0; // inches

    public Drive(IGyroscopeSensor gyroscope) {
        this.gyroscope = gyroscope;

        frontLeftMotor = new CANSparkMax(PortMap.CAN.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(PortMap.CAN.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(PortMap.CAN.REAR_LEFT_MOTOR, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(PortMap.CAN.REAR_RIGHT_MOTOR, MotorType.kBrushless);

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        rearLeftMotor.restoreFactoryDefaults();
        rearRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        rearLeftMotor.setInverted(false);
        rearRightMotor.setInverted(true);

        driveBase = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

        rotationController = new PIDController(setP, setI, setD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE_DEGREES));
    }

    @Override
    public Mode getCurrentDriveMode() {
        return mode;
    }

    @Override
    public void resetGyro() {
        gyroscope.resetYaw();
    }

    @Override
    public void rotateRelative(double angle) {
        mode = Mode.MANUAL;
        
        desiredAngle = gyroscope.getYaw() + angle; 
    }

    @Override
    public void rotateAbsolute(double angle) {
        mode = Mode.MANUAL;

        desiredAngle = angle;
    }

    @Override
    public void driveDistance(double distanceInches, double speed, double angle) {
        driveDistance(distanceInches, speed, angle, null);
    }

    @Override
    public void driveDistance(double distanceInches, double speed, double angle, Runnable completionRoutine) {
        
        mode = Mode.AUTO;
        
        setCompletionRoutine(completionRoutine);
        desiredDistance = distanceInches;
        autoSpeed = speed;
        autoAngleDegrees = -angle;
    }

    @Override
    public void rotateRelative(double angle, Runnable completionRoutine) {
        mode = Mode.AUTO;
        
        setCompletionRoutine(completionRoutine);
        desiredAngle = gyroscope.getYaw() + Math.toRadians(angle);
    }

    @Override
    public void rotateAbsolute(double angle, Runnable completionRoutine) {
        mode = Mode.AUTO;
        
        setCompletionRoutine(completionRoutine);
        desiredAngle = Math.toRadians(angle);
    }

    public void driveManualImplementation(double forwardSpeed, double strafeSpeed) {
        mode = Mode.MANUAL;

        double absoluteForward = -.1 * (forwardSpeed * Math.cos(gyroscope.getYaw()) + strafeSpeed * Math.sin(gyroscope.getYaw()));
        double absoluteStrafe = -.1 * (-forwardSpeed * Math.sin(gyroscope.getYaw()) + strafeSpeed * Math.cos(gyroscope.getYaw())); 

        this.forwardSpeed = absoluteForward;
        this.strafeSpeed = absoluteStrafe;
    }

    @Override
    public void driveManual(double forwardSpeed, double strafeSpeed) {
        setCompletionRoutine(null);
        driveManualImplementation(forwardSpeed, strafeSpeed);
    }

    @Override
    public void stop() {
        driveManualImplementation(0.0, 0.0);
        desiredAngle = gyroscope.getYaw();
        angularSpeed = 0.0;
    }

    @Override
    public void init() {
        currentCompletionRoutine = null;
        stop();
    }

    private void setCompletionRoutine(Runnable completionRoutine) {
        if (currentCompletionRoutine != null) {
            throw new IllegalStateException("Tried to perform an autonomous action while one was already in progress!");
        }

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

    private void manualControlPeriodic() {
        angularSpeed = 0.25 * rotationController.calculate(gyroscope.getYaw(), desiredAngle);
        
        driveBase.driveCartesian(forwardSpeed, strafeSpeed, angularSpeed);
    }

    @Override
    public void periodic() {
        if (mode == Mode.MANUAL) {
            manualControlPeriodic();
            Debug.logPeriodic("Yaw" + Double.toString(gyroscope.getYaw()));
            Debug.logPeriodic("desired angle " + Double.toString(desiredAngle));
            Debug.logPeriodic("angular speed" + Double.toString(angularSpeed));
        } else if (mode == Mode.AUTO) {
            angularSpeed = rotationController.calculate(gyroscope.getYaw(), desiredAngle);
        
            driveBase.drivePolar(autoSpeed, autoAngleDegrees, angularSpeed);

            // Check if we've completed our travel
            double averageDistanceTraveledLeft = Math.abs((frontLeftMotor.getEncoder().getPosition() + rearLeftMotor.getEncoder().getPosition()) / 2);
            double averageDistanceTraveledRight = Math.abs((frontRightMotor.getEncoder().getPosition() + rearRightMotor.getEncoder().getPosition()) / 2);
            double averageDistanceTraveled = Math.abs((averageDistanceTraveledLeft + averageDistanceTraveledRight) / 2);
            Debug.logPeriodic("Total Distance: " + averageDistanceTraveled);
            Debug.logPeriodic("Rear left encoder: " + rearLeftMotor.getEncoder().getPosition());
            Debug.logPeriodic("Rear right encoder: " + rearRightMotor.getEncoder().getPosition());
            Debug.logPeriodic("Front left encoder: " + frontLeftMotor.getEncoder().getPosition());
            Debug.logPeriodic("Front right encoder: " + frontRightMotor.getEncoder().getPosition());
            Debug.logPeriodic("---");
            if (averageDistanceTraveled > desiredDistance) {
                handleActionEnd();
            } 
        } else {
            throw new IllegalArgumentException("The drive base controller is in an invalid drive mode.");
        }
    }
}