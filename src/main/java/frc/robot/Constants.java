package frc.robot;

public class Constants {
    /* ALL CONSTANTS GO HERE */
    
    // Start Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    //End Controllers

    // Start CanIDs
    public static final int kShooterArmRotateLeftCanID = 40;
    public static final int kShooterArmRotateRightCanID = 40;
    public static final int kSpinLeftShooterCanID = 40;
    public static final int kSpinRightShooterCanID = 40;
    public static final int kShooterIntakeSpinCanID = 40;
    public static final int kFrontIntakeRotateCanID = 40;
    public static final int kFrontIntakeSpinCanID = 40;
    public static final int kLeftClimbCanID = 40;
    public static final int kRightClimbCanID = 40;
    public static final int kFrontIntakeCancoderCanID = 40;
    public static final int kShooterArmCancoderCanID = 40;
    public static final int kSignalLightCanID = 38;
    // End CanIDs

    // Start PWM
    public static final int kShooterIntakePivotReleasePWM = 0;
    //End PWM

    // Start Analog IO
    public static final int kBeamBreakIntakeAnalog = 0;
    public static final int kBeamBreakShooterAnalog = 1;
    public static final double kIntakeBeamBreakCrossover = 0.4;
    public static final double kShooterBeamBreakCrossover = 0.4;
    // End Analog IO

    // Start Positions
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
    // End Positions

    // Start Speeds
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
    // End Speeds
    
    // Start Swerves
    public static final int kLFSteerOffset = 0;
    public static final int kRFSteerOffset = 0;
    public static final int kLRSteerOffset = 0;
    public static final int kRRSteerOffset = 0;

    public static final int kDriveMaxVelocity = 0;
    // End Swerves

    // Start Motor Setting Values
    // Start shooter settings
    public static final int kShooterProportional = 0;
    public static final int kShooterIntegral = 0;
    public static final int kShooterDerivative = 0;
    public static final int kShooterVelocityFeedFoward = 0;
    public static final int kShooterMaxForwardVoltage = 0;
    public static final int kShooterMaxReverseVoltage = 0;
    public static final int kShooterSupplyCurrentLimit = 0;
    public static final int kShooterVoltageClosedLoopRampPeriod = 0;
    // End shooter Settings

    // Start ShooterArmRotate settings
    public static final int kShooterArmRotateProportional = 0;
    public static final int kShooterArmRotateIntegral = 0;
    public static final int kShooterArmRotateDerivative = 0;
    public static final int kShooterArmRotateVelocityFeedFoward = 0;
    public static final int kShooterArmRotateStaticFeedFoward = 0;
    public static final int kShooterArmRotateMaxForwardVoltage = 0;
    public static final int kShooterArmRotateMaxReverseVoltage = 0;
    public static final int kShooterArmRotateMotionMagicCruiseVelocity = 0;
    public static final int kShooterArmRotateMotionMagicAcceleration = 0;
    public static final int kShooterArmRotateMotionMagicJerk = 0;
    public static final int kShooterArmRotateSupplyCurrentLimit = 0;
    public static final int kShooterArmRotateVoltageClosedLoopRampPeriod = 0;
    // End ShooterArmRotate settings

    // Start FrontIntakeRotate Settings
    public static final int kFrontIntakeRotateProportional = 0;
    public static final int kFrontIntakeRotateIntegral = 0;
    public static final int kFrontIntakeRotateDerivative = 0;
    public static final int kFrontIntakeRotateVelocityFeedFoward = 0;
    public static final int kFrontIntakeRotateStaticFeedFoward = 0;
    public static final int kFrontIntakeRotateMaxForwardVoltage = 0;
    public static final int kFrontIntakeRotateMaxReverseVoltage = 0;
    public static final int kFrontIntakeRotateMotionMagicCruiseVelocity = 0;
    public static final int kFrontIntakeRotateMotionMagicAcceleration = 0;
    public static final int kFrontIntakeRotateMotionMagicJerk = 0;
    public static final int kFrontIntakeSupplyCurrentLimit = 0;
    public static final int kFrontIntakeVoltageClosedLoopRampPeriod = 0;
}
