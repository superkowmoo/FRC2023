package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;

public class TeleoperatedMode implements IRobotMode {
    
    private XboxController xboxController;
    private IDrive drive;
    private ILauncher launcher;

    private static final double LEFT_STICK_EXPONENT = 3.0;
    private static final double RIGHT_STICK_EXPONENT = 3.0;
    private static final double ROTATION_THRESHOLD = 0.3;
    // check values above

    public TeleoperatedMode(IDrive drive, ILauncher launcher){ 
        xboxController = new XboxController(PortMap.USB.XBOXCONTROLLER);

        this.drive = drive;
        this.launcher = launcher;
    }

    @Override
    public void init(){
        drive.init();
    }

     @Override
     public void periodic(){

        double leftX = xboxController.getLeftX();
        double leftY = -xboxController.getLeftY();

        leftX = Math.pow(leftX, LEFT_STICK_EXPONENT);
        leftY = Math.pow(leftY, LEFT_STICK_EXPONENT);

        drive.driveManual(leftY, leftX);

        //Process Rotation Control
        double rightX = xboxController.getRightX();
        double rightY = -xboxController.getRightY();

        rightX = Math.pow(rightX, RIGHT_STICK_EXPONENT);
        rightY = Math.pow(rightY, RIGHT_STICK_EXPONENT);

        double angle = Math.atan2(rightX, rightY);

        // Think Pythagorean Thereom
        if(Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2)) > ROTATION_THRESHOLD) {
            drive.rotateAbsolute(angle);
        }
         // Process Peripheral control
         if (xboxController.getRightBumper()){
            launcher.shoot();
        }

        if (xboxController.getLeftBumper()) {
            launcher.intake();
        }

        if (xboxController.getAButton()) {
            launcher.reverse();
        }

        if (xboxController.getBButton()) {
            launcher.advance();
        }
     }

}
