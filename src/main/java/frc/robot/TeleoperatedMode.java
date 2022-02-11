package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;

public class TeleoperatedMode implements IRobotMode {
    
    private XboxController XboxController;
    private IDrive drive;
    private ILaucher laucher;
    private ILimelight limelight;
    private IColorWheel wheel;

    private String data;
    private double distanceFromTarget;

    private static final double LEFT_STICK_EXPONENT = 3.0;
    private static final double RIGHT_STICK_EXPONENT = 3.0;
    private static final double ROTATION_THRESOLD = 0.3;
    // check values above

    public TeleoperatedMode(IDrive drive, ILauncher launcher, ILimelight limelight, IColorWheel wheel){ 
        xboxController = new XboxController(*****);

        this.drive = drive;
        this.laucher = launcher;
        this.limelight = limelight;
        this.wheel = wheel;
    }

    @Override
    public void init(){
        drive.init();
    }

     @Override
     public void periodic(){

        double leftX = xboxControllers.getx(Hand.kleft);
        double leftY = -xboxController.getY(Hand.kLeft);

        leftX = Math.pow(leftX, LEFT_STICK_EXPONENT)
     }

}
