package frc.robot;

public class AutonomousMode implements IRobotMode {

    private IDrive drive;
    private double speed = 0.1;

    private ILauncher launcher;

    public AutonomousMode(IDrive drive, ILauncher launcher) {
        this.drive = drive;
        this.launcher = launcher;
    }
    
    public void init() {
        autoMove();
    }

    public void autoMove() { 
        drive.driveDistance(20, speed, 0, () -> autoShoot());
        Debug.log("autoMove1");
        Debug.log("current drive mode: " + drive.getCurrentDriveMode());
    }

    public void autoShoot() {
        launcher.autoShoot(() -> autoMove2());
        Debug.log("autoShoot");
    }

    public void autoMove2() {
        drive.driveDistance(35, -speed, 0, null);
        Debug.log("autoMove2");
    }

    /*public void autoMove() {
        drive.driveDistance(40, -speed, 0, null);
        Debug.log("autoMove1");
        Debug.log("current drive mode: " + drive.getCurrentDriveMode());
    }*/

    @Override
    public void periodic() {

    }
}
