package frc.robot;

public class Constants {
    /* ALL CONSTANTS GO HERE */
    
    //Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    //CanIDs
    public static final int kShooterRotateLeftCanID = 40;
    public static final int kShooterRotateRightCanID = 40;
    public static final int kShooterIntakeSpinCanID = 40;
    public static final int kFrontIntakeRotateCanID = 40;
    public static final int kFrontIntakeSpinCanID = 40;
    public static final int kLeftClimbCanID = 40;
    public static final int kRightClimbCanID = 40;
    public static final int kFrontIntakeCancoderCanID = 40;
    public static final int kArmCancoderCanID = 40;
    public static final int kSignalLightCanID = 38;

    //PWM
    public static final int kIntakePivotReleasePWM = 0;

    //Analog IO
    public static final int kBeamBreakIntakeAnalog = 0;
    public static final int kBeamBreakShooterAnalog = 1;
    public static final double kIntakeBeamBreakCrossover = 0.4;
    public static final double kShooterBeamBreakCrossover = 0.4;

    //Positions
    /*
     * FrontIntakePositions (Down, Up, Home)
     * ArmPositions (Intaking, Traveling, Podium Shoot, Subwoofer Shoot, Amp Score, Trap Score, Ready To Climb)
     * Climber (Climber Extended, Climber Retracted, Home)
     */

    //Speeds
    /* Shooter (Max Speed, Podium Shoot, Subwoofer Shoot, Amp Score)
     * ShooterIntake (Intaking, Spitting, Max Speed?)
     * FrontIntake (Intaking, Spitting, Max Speed?)
     */

    //Swerves
    /*
     * Steer (Offsets, Max Speed?, whatever else is in swerve modules)
     * Drive (Max Speed, whatever else is in swerve modules)
     */
}
