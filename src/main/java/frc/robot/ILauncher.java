package frc.robot;

public interface ILauncher {

    public void init();

    public void stop();

    public void intake();

    public void intakeReverse();

    public void advance();

    public void reverse();

    public void shoot();

    public void shooterReverse();

    public void autoShoot(Runnable completionRoutine);

    public void periodic();

}
