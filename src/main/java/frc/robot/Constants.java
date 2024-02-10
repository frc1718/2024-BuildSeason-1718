package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    public static final int kShooterIntakePivotReleasePWM = 0;

    //Analog IO
    public static final int kBeamBreakIntakeAnalog = 0;
    public static final int kBeamBreakShooterAnalog = 1;
    public static final double kIntakeBeamBreakCrossover = 0.4;
    public static final double kShooterBeamBreakCrossover = 0.4;

    //Positions
    /*
     * FrontIntakePositions (Down, Up, Home)
     * ArmPositions (Home, Podium Shoot, Subwoofer Shoot, Amp Score, Trap Score, Ready To Climb)
     * Climber (Climber Extended, Climber Retracted, Home)
     */

    public static final int kShooterArmSubwooferPos = 0;
    public static final int kShooterArmPodiumPos = 0;
    public static final int kShooterArmAmpPos = 0;
    public static final int kShooterArmHomePos = 0;
    public static final int kShooterArmPreClimbPos = 0;
    public static final int kShooterArmTolerancePos = 0;
    public static final int kShooterArmSpitPos = 0;
    public static final int kShooterArmTrapPos = 0;

    public static final int kClimberHomePos = 0;
    public static final int kClimberClimbPos = 0;
    public static final int kClimberPreClimbPos = 0;
    public static final int kClimberTolerancePos = 0;
    
    public static final int kFrontIntakeDownPos = 0;
    public static final int kFrontIntakeUpPos = 0;
    public static final int kFrontIntakeHomePos = 0; 
    public static final int kFrontIntakeTolerancePos = 0;

    public static final int kShooterIntakePivotReleasedPos = 0;

    //Speeds
    /* Shooter (Max Speed, Podium Shoot, Subwoofer Shoot, Amp Score)
     * ShooterIntake (Intaking, Spitting, Max Speed?)
     * FrontIntake (Intaking, Spitting, Max Speed?)
     */

    public static final int kShooterSubwooferSpeed = 0;
    public static final int kShooterPodiumSpeed = 0;
    public static final int kShooterAmpSpeed = 0;
    public static final int kShooterIdleSpeed = 0;
    public static final int kShooterMaxSpeed = 0;
    public static final int kShooterStopSpeed = 0;
    
    public static final int kShooterIntakeTrapSpeed = 0;
    public static final int kShooterIntakeSuckSpeed = 0;
    public static final int kShooterIntakeStopSpeed = 0;
    public static final int kShooterIntakeSpitSpeed = 0;
    public static final int kShooterIntakeShootSpeed = 0;
    public static final int kShooterIntakeIndexSpeed = 0;
    public static final int kShooterIntakeReverseIndexSpeed = 0;
    public static final int kShooterIntakeMaxSpeed = 0;

    public static final int kFrontIntakeStopSpeed = 0;
    public static final int kFrontIntakeSuckSpeed = 0;
    public static final int kFrontIntakeSpitSpeed = 0;
    public static final int kFrontIntakeMaxSpeed = 0;

    public static final int kShooterSpeedTolerance = 0;
    //Swerves
    /*
     * Steer (Offsets, Max Speed?, whatever else is in swerve modules)
     * Drive (Max Speed, whatever else is in swerve modules)
     */

    public static final int kLFSteerOffset = 0;
    public static final int kRFSteerOffset = 0;
    public static final int kLRSteerOffset = 0;
    public static final int kRRSteerOffset = 0;

    public static final int kDriveMaxVelocity = 0;

    
     //Name of the limelight camera.
     public static final String kLimelightName = "limelight";

     //Pose of the blue speaker.
     //Used to be a pose, but only the X and Y are needed.  Changed to a translation to clean up the actual calculation.
     public static final Translation2d kBlueSpeakerLocation = new Translation2d(0.0, 5.5);
}
