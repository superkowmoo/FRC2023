package frc.robot;

public class AutonomousMode implements IRobotMode {

    private IDrive drive;
    private double speed = 0.8;

    private ILauncher launcher;

    public AutonomousMode(IDrive drive, ILauncher launcher) {
        this.drive = drive;
        this.launcher = launcher;
    }
    
    public void init() {
        autoMove();
    }

    public void autoMove() { 
        drive.driveDistance(-30, speed, 0, () -> autoShoot()); // change speed and angle
    }

    public void autoShoot() {
        launcher.autoShoot();
    }

    @Override
    public void periodic() {

    }
}
