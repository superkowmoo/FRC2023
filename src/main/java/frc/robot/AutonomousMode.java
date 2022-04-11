package frc.robot;

public class AutonomousMode implements IRobotMode {

    private IDrive drive;
    private double speed = 0.15;

    private ILauncher launcher;

    public AutonomousMode(IDrive drive, ILauncher launcher) {
        this.drive = drive;
        this.launcher = launcher;
    }

    public void init() {
        autoShoot();
    }

    public void autoShoot() {
        launcher.autoShoot(() -> autoMove());
        Debug.log("autoShoot");
    }

    public void autoMove() {
        drive.driveDistance(52, -speed, 0, null);
        Debug.log("autoMove2");
    }

    @Override
    public void periodic() {

    }
}