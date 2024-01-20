package frc.robot;

public class Constants {
    /* ALL CONSTANTS GO HERE */
    
    //Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    //CanIDs
    public static final int kArmRotateLeftCanID = 40;
    public static final int kArmRotateRightCanID = 40;
    public static final int kArmIntakeSpinCanID = 40;
    public static final int kFrontIntakeRotateCanID = 40;
    public static final int kFrontIntakeSpinCanID = 40;
    public static final int kLeftClimbCanID = 40;
    public static final int kRightClimbCanID = 40;
    public static final int kKickupCanID = 40;
    public static final int kFrontIntakeCancoderCanID = 40;
    public static final int kArmCancoderCanID = 40;
    public static final int kPartPresentLEDCanID = 40;

    //PWM
    public static final int kIntakePivotReleasePWM = 0;

    //Analog IO
    public static final int kBeamBreakIntakeAnalog = 0;
    public static final int kBeamBreakShooterAnalog = 1;
    public static final double kBeamBreakCrossover = 0.8;
}
