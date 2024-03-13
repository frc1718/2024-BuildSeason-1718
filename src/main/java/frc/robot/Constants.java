package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    /* ALL CONSTANTS GO HERE */
    
    // Start Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    //End Controllers

    // Start CanIDs
    public static final int kShooterArmRotateLeftCanID = 18;
    public static final int kShooterArmRotateRightCanID = 19;
    public static final int kSpinLeftShooterCanID = 20;
    public static final int kSpinRightShooterCanID = 21;
    public static final int kLeftClimbCanID = 22;
    public static final int kRightClimbCanID = 23;
    public static final int kShooterIntakeSpinCanID = 24;
    public static final int kFrontIntakeRotateCanID = 25;
    public static final int kFrontIntakeSpinCanID = 26;
    public static final int kFrontIntakeRotateCancoderCanID = 27;
    public static final int kShooterArmCancoderCanID = 28;
    public static final int kShooterIntakeRotateCanID = 31;
    public static final int kSignalLightCanID = 38;

    // End CanIDs

    // Start PWM
    public static final int kShooterIntakePivotReleasePWM = 0;
    //End PWM

    // Start Analog IO
    public static final int kBeamBreakIntakeAnalog = 0;
    public static final int kBeamBreakShooterAnalog = 1;
    public static final double kIntakeBeamBreakCrossover = 0.9;
    public static final double kShooterBeamBreakCrossover = 0.9;
    // End Analog IO

    // Start Motor Enables
    //public static final int kMotorEnableRightFrontSwerveDrive = 1;
    //public static final int kMotorEnableRightFrontSwerveSteer = 1;
    //public static final int kMotorEnableRightRearSwerveDrive = 1;
    //public static final int kMotorEnableRightRearSwerveSteer = 1;
    //public static final int kMotorEnableLeftFrontSwerveDrive = 1;
    //public static final int kMotorEnableLeftFrontSwerveSteer = 1;
    //public static final int kMotorEnableLeftRearSwerveDrive = 1;
    //public static final int kMotorEnableLeftRearSwerveSteer = 1;
    public static final int kMotorEnableFrontIntakeSpin = 1;
    public static final int kMotorEnableFrontIntakeRotate = 1;
    public static final int kMotorEnableLeftShooterSpin = 1;
    public static final int kMotorEnableRightShooterSpin = 1;
    public static final int kMotorEnableShooterIntakeSpin = 1;
    public static final int kMotorEnableShooterArmRotate = 1;
    public static final int kMotorEnableClimber = 1;
    // End Motor Enables

    //Enable Print Statemens
    public static final boolean kPrintOperatorHome = false;
    public static final boolean kPrintOperatorPreClimb = false;
    public static final boolean kPrintOperatorShooterModeAmp = false;
    public static final boolean kPrintOperatorShooterModeMiddleAuto = false;
    public static final boolean kPrintOperatorShooterModePass = false;
    public static final boolean kPrintOperatorShooterModePodium = false;
    public static final boolean kPrintOperatorShooterModeRightAuto = false;
    public static final boolean kPrintOperatorShooterModeSubwoofer = false;
    public static final boolean kPrintOperatorShootWithPose = false;

    public static final boolean kPrintDriverClimb = false;
    public static final boolean kPrintDriverShoot = false;
    public static final boolean kPrintDriverShootWithLimelight = false;
    public static final boolean kPrintDriverShootTrap = false;
    public static final boolean kPrintDriverSpit = false;
    public static final boolean kPrintDriverSuck = false;
    public static final boolean kPrintDriverSuckNoFront = false;

    public static final boolean kPrintGeneralNotePosition = false;
    public static final boolean kPrintGeneralSetMotorsToCoast = false;
    public static final boolean kPrintGeneralStowArmAndIntake = false;
    
    public static final boolean kPrintLEDsLightLEDOnNotePresent = false;

    public static final boolean kPrintSubsystemBeamBreak = false;
    public static final boolean kPrintSubsystemClimber = false;
    public static final boolean kPrintSubsystemFrontIntake = false;
    public static final boolean kPrintSubsystemLED = false;
    public static final boolean kPrintSubsystemShooterIntakeSubsystem = false;
    public static final boolean kPrintSubsystemShooterSubsystem = false;

    // Start Positions
    public static final double kShooterArmSubwooferPos = -0.1; //-0.11
    public static final double kShooterArmPodiumPos = -0.0473;  //-0.0388
    public static final double kShooterArmAmpPos = 0.135;   //0.135
    public static final double kShooterArmHomePos = -0.136; //-0.136
    public static final double kShooterArmMiddleAutoPos = -0.0388;  //-0.0388
    public static final double kShooterArmRightAutoPos = -0.388;    //-0.388
    public static final double kShooterArmPreClimbPos = 0.135;  //0.135
    public static final double kShooterArmTolerancePos = 0.01;  //0.01
    public static final double kShooterArmSpitPos = -0.06;  //-0.06
    public static final double kShooterArmTrapPos = 0.135;  //0.135
        /* Safeties - DO NOT CHANGE THIS LINE */ public static final double kShooterArmUpSafety = 0.135;    //0.135
        /* Safeties - DO NOT CHANGE THIS LINE */ public static final double kShooterArmDownSafety = -0.136; //-0.136


    public static final int kClimberHomePos = 0;    
    public static final double kClimberClimbPos = -38.5;
    public static final double kClimberPreClimbPos = 17.5;
    public static final int kClimberTolerancePos = 1;
    
    public static final double kFrontIntakeDownPos = 0.00;  //Down so we can suck in
    public static final double kFrontIntakeHomePos = 0.3; //Starting position.  Used to be .38 all the way back.  Changed to speed up.
    public static final double kFrontIntakeTolerancePos = 0.015;
    public static final double kFrontIntakeClearPos = 0.239; //was .239Clear is clear of the shooterarm motion
        /* Safeties - DO NOT CHANGE THIS LINE */ public static final double kFrontIntakeUpSafety = 0.38;
        /* Safeties - DO NOT CHANGE THIS LINE */ public static final double kFrontIntakeDownSafety = -0.01;
    

    public static final int kShooterIntakePivotReleasedPos = 0;
    // End Positions

    // Start Speeds
    public static final double kShooterSubwooferSpeed = 70;  //Was 70, at 10 for testing in build room
    public static final int kShooterPodiumSpeed = 70;  //Was 70, at 10 for testing in build room
    public static final int kShooterAmpSpeed = 20;  //Was 20, at 7 for testing in build room
    public static final int kShooterPoseSpeed = 0;
    public static final int kShooterIdleSpeed = 30; //Was 30, at 5 for testing in build room
    public static final int kShooterMaxSpeed = 80;
    public static final int kShooterStopSpeed = 0;
    public static final int kShooterPassSpeed = 40;
    
    public static final int kShooterIntakeTrapSpeed = 0;
    public static final int kShooterIntakeSuckSpeed = 80;
    public static final int kShooterIntakeStopSpeed = 0;
    public static final int kShooterIntakeSpitSpeed = -60;
    public static final int kShooterIntakeShootSpeed = 75;
    public static final int kShooterIntakeIndexSpeed = 10;
    public static final int kShooterIntakeReverseIndexSpeed = 10;
    public static final int kShooterIntakeMaxSpeed = 100;

    public static final int kFrontIntakeStopSpeed = 0;
    public static final int kFrontIntakeSuckSpeed = 40;
    public static final int kFrontIntakeSpitSpeed = -35;
    public static final int kFrontIntakeMaxSpeed = 75;

    public static final int kShooterSpeedTolerance = 1;

    public static final int kShooterShotSpeedDrop = 70;  //The amount the shooter declines when we shoot a note

    // End Speeds
    
    // Start Swerves
    public static final int kLFSteerOffset = 0;
    public static final int kRFSteerOffset = 0;
    public static final int kLRSteerOffset = 0;
    public static final int kRRSteerOffset = 0;

    public static final int kDriveMaxVelocity = 0;
    // End Swerves

    // Start Motor Setting Values
    
    //Start ShooterIntakeRotate settings
    public static final double kShooterIntakeRotateProportional = 0; // An error of 1 rotation per second results in 2V output
    public static final double kShooterIntakeRotateIntegral = 0; // An error of 1 rotation per second increases output by 0.5V every second
    public static final double kShooterIntakeRotateDerivative = 0; // A change of 1 rotation per second squared results in 0.01 volts output
    public static final double kShooterIntakeRotateFeedFoward = 0; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    public static final double kShooterIntakeRotateMaxForwardVoltage = 11;
    public static final double kShooterIntakeRotateMaxReverseVoltage = -11;
    public static final double kShooterIntakeRotateSupplyCurrentLimit = 20;
    public static final double kShooterIntakeRotateVoltageClosedLoopRampPeriod = 0;
    public static final InvertedValue kShooterIntakeRotateDirection = InvertedValue.Clockwise_Positive;
    public static final double kShooterIntakeRotateMotionMagicCruiseVelocity = 0;
    public static final double kShooterIntakeRotateMotionMagicAcceleration = 0;
    

    // Start LeftShooter settings
    public static final InvertedValue kLeftShooterDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftShooterProportional = 0.5;
    public static final double kLeftShooterIntegral = 0;
    public static final double kLeftShooterDerivative = 0;
    public static final double kLeftShooterVelocityFeedFoward = 0.135;
    public static final int kLeftShooterMaxForwardVoltage = 11;
    public static final int kLeftShooterMaxReverseVoltage = -11;
    public static final int kLeftShooterSupplyCurrentLimit = 50;
    public static final int kLeftShooterVoltageClosedLoopRampPeriod = 0;
    // End LeftShooter settings
    
    // Start RightShooter settings
    public static final InvertedValue kRightShooterDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kRightShooterProportional = 0.5;
    public static final double kRightShooterIntegral = 0;
    public static final double kRightShooterDerivative = 0;
    public static final double kRightShooterVelocityFeedFoward = .135;
    public static final int kRightShooterMaxForwardVoltage = 11;
    public static final int kRightShooterMaxReverseVoltage = -11;
    public static final int kRightShooterSupplyCurrentLimit = 50;
    public static final int kRightShooterVoltageClosedLoopRampPeriod = 0;
    // End RightShooter settings

    // Start ShooterArmRotate settings
    public static final InvertedValue kShooterArmRotateDirection = InvertedValue.Clockwise_Positive;
    public static final double kShooterArmRotateProportional = 60;
    public static final double kShooterArmRotateIntegral = 0;
    public static final double kShooterArmRotateDerivative = 0;
    public static final double kShooterArmRotateGravity = 0.3;
    public static final double kShooterArmRotateVelocityFeedFoward = 41;
    //public static final int kShooterArmRotateStaticFeedFoward = 0;
    public static final int kShooterArmRotateMaxForwardVoltage = 11;
    public static final int kShooterArmRotateMaxReverseVoltage = -11;
    public static final double kShooterArmRotateMotionMagicCruiseVelocity = 0.235;
    public static final double kShooterArmRotateMotionMagicAcceleration = 2;
    public static final int kShooterArmRotateMotionMagicJerk = 0;
    public static final int kShooterArmRotateSupplyCurrentLimit = 60;
    public static final int kShooterArmRotateVoltageClosedLoopRampPeriod = 0;
    // End ShooterArmRotate settings
    public static final int kShooterArmRotateCancoderRotorToSensorRatio = 300;
    public static final SensorDirectionValue kShooterArmRotateCancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final AbsoluteSensorRangeValue kShooterArmRotateCancoderRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    public static final double kShooterArmRotateCancoderOffset = 0.38159;
    public static final InvertedValue kRightShooterArmRotateDirection = InvertedValue.CounterClockwise_Positive;
    // ShooterArmRotateCancoder settings above /\     /\     /\

    // Start FrontIntakeRotate settings
    public static final InvertedValue kFrontIntakeRotateDirection = InvertedValue.Clockwise_Positive;
    public static final double kFrontIntakeRotateProportional = 30;
    public static final double kFrontIntakeRotateIntegral = 0;
    public static final double kFrontIntakeRotateDerivative = 0;
    public static final double kFrontIntakeRotateGravity = 0.28;
    public static final double kFrontIntakeRotateVelocityFeedFoward = 7.2;
    //public static final int kFrontIntakeRotateStaticFeedFoward = 0;
    public static final int kFrontIntakeRotateMaxForwardVoltage = 11;
    public static final int kFrontIntakeRotateMaxReverseVoltage = -11;
    public static final double kFrontIntakeRotateMotionMagicCruiseVelocity = 4;
    public static final double kFrontIntakeRotateMotionMagicAcceleration = 10;
    public static final double kFrontIntakeRotateMotionMagicJerk = 0;
    public static final int kFrontIntakeRotateSupplyCurrentLimit = 20;
    public static final int kFrontIntakeRotateVoltageClosedLoopRampPeriod = 0;
    public static final int KFrontIntakeCancoderOffset = 0;
    // End FrontIntakeRotate settings
    public static final double kFrontIntakeRotateRotorToSensorRatio = 56.25;
    public static final SensorDirectionValue kFrontIntakeRotateCancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final AbsoluteSensorRangeValue kFrontIntakeRotateCancoderRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    public static final double kFrontIntakeRotateCancoderOffset = -0.176514;
    // FrontIntakeRotateCancoder settings above /\     /\     /\

    // Start FrontIntakeSpin settings
    public static final InvertedValue kFrontIntakeSpinDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kFrontIntakeSpinProportional = .07;
    public static final double kFrontIntakeSpinIntegral = 0;
    public static final double kFrontIntakeSpinDerivative = 0;
    public static final double kFrontIntakeSpinVelocityFeedFoward = 0.122;
    public static final int kFrontIntakeSpinMaxForwardVoltage = 11;
    public static final int kFrontIntakeSpinMaxReverseVoltage = -11;
    public static final int kFrontIntakeSpinSupplyCurrentLimit = 20;
    public static final int kFrontIntakeSpinVoltageClosedLoopRampPeriod = 0;
    // End FrontIntakeSpin settings

    // Start ShooterIntakeSpin settings
    public static final InvertedValue kShooterIntakeSpinDirection = InvertedValue.Clockwise_Positive;
    public static final double kShooterIntakeSpinProportional = 0.2;
    public static final double kShooterIntakeSpinIntegral = 0;
    public static final double kShooterIntakeSpinDerivative = 0;
    public static final double kShooterIntakeSpinVelocityFeedFoward = 0.14;
    public static final int kShooterIntakeSpinMaxForwardVoltage = 11;
    public static final int kShooterIntakeSpinMaxReverseVoltage = -11;
    public static final int kShooterIntakeSpinSupplyCurrentLimit = 40;
    public static final int kShooterIntakeSpinVoltageClosedLoopRampPeriod = 0;
    // End ShooterIntakeSpin settings

    // Start Climber settings
    public static final InvertedValue kLeftClimberDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kLeftClimberProportional = 15;
    public static final double kLeftClimberIntegral = 0;
    public static final double kLeftClimberDerivative = 0.2;
    public static final double kLeftClimberVelocityFeedFoward = 0;
    //public static final int kLeftClimberStaticFeedFoward = 0;
    public static final int kLeftClimberMaxForwardVoltage = 11;
    public static final int kLeftClimberMaxReverseVoltage = -11;
    public static final int kLeftClimberSupplyCurrentLimit = 50;
    public static final double kLeftClimberVoltageClosedLoopRampPeriod = 0.05;
    // End Climber settings

    //Start Right Climber settings
    public static final InvertedValue kRightClimberDirection = InvertedValue.Clockwise_Positive;
    public static final double kRightClimberProportional = 15;
    public static final double kRightClimberIntegral = 0;
    public static final double kRightClimberDerivative = 0.2;
    public static final double kRightClimberVelocityFeedFoward = 0;
    //public static final int kRightClimberStaticFeedFoward = 0;
    public static final int kRightClimberMaxForwardVoltage = 11;
    public static final int kRightClimberMaxReverseVoltage = -11;
    public static final int kRightClimberSupplyCurrentLimit = 50;
    public static final double kRightClimberVoltageClosedLoopRampPeriod = 0.05;
    // End Right Climber settings
    
    //Start custom Swerve settings
    public static final int kSwerveDriveMaxForwardVoltage = 11;
    public static final int kSwerveDriveMaxReverseVoltage = -11;
    //A note of the current limiting: it only kicks in if the current is past the threshold for the threshold time.
    //So, with the values below, the supply current will be limited to 60 A if the motor draws 80 A or higher for 1 sec.
    public static final int kSwerveDriveSupplyCurrentLimit = 60;
    public static final boolean kSwerveDriveSupplyCurrentLimitEnable = true;
    public static final int kSwerveDriveSupplyCurrentThreshold = 80;
    public static final int kSwerveDriveSupplyTimeThreshold = 1;

    public static final int kSwerveSteerMaxForwardVoltage = 11;
    public static final int kSwerveSteerMaxReverseVoltage = -11;
    public static final int kSwerveSteerSupplyCurrentLimit = 60;
    public static final boolean kSwerveSteerSupplyCurrentLimitEnable = true;
    public static final int kSwerveSteerSupplyCurrentThreshold = 80;
    public static final int kSwerveSteerSupplyTimeThreshold = 1;

    //Name of the limelight camera.
    public static final String kLimelightName = "limelight";

    //Allowable TX tolerance for aiming at an AprilTag.
    public static final double kTXTolerance = 1.5;    //A complete guess.

    //Distance between the center of the speaker AprilTag and the floor.
    //The bottom of the AprilTag is 4 ft. 3-7/8 in. from the ground.
    //Add 4-1/2 in. to the center of the AprilTag.
    //56-3/8 in. is 1.432 meters.
    public static final double kSpeakerAprilTagHeight = 1.45;
    
    //Distance between the lens of the limelight and the floor.
    //Lens of the limelight is 16-9/10 in. from the ground.
    //16-9/10 in. is 0.429 meters.
    public static final double kLimelightHeight = 0.435;

    //Pose of the blue speaker.
    //Used to be a pose, but only the X and Y are needed.  Changed to a translation to clean up the actual calculation.
    public static final Translation2d kBlueSpeakerLocation = new Translation2d(0.0, 5.5);

    //Default pose for testing.
    //For correct swerve driving, seedFieldRelative needs to be called at least once.
    //During competitions, the PathPlanner code will call seedFieldRelative with the starting position of the autonomous routine.
    //For testing, this will be called during robotInit.
    //Since the driver station defaults to 'Red 1' as the alliance station when not connected to the FMS, the default pose is for the red alliance.
    //Change the rotation to 0 radians if you want the default pose to be the blue alliance.
    public static final Rotation2d kDefaultRotation = new Rotation2d(Math.PI);

    static {
        
    }
    public static final Pose2d kDefaultPose = new Pose2d(0, 0, kDefaultRotation);

    //Interpolation for the shoot with pose command.  The values that correspond to shooting from the subwoofer and podium can also be added.
    //Both tables use distance (in meters) as the key.
    public static final InterpolatingDoubleTreeMap kShooterSpeedTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kShooterArmTable = new InterpolatingDoubleTreeMap();

    /*
    static {
        //Populating the shooter speed table.  Values are in rotations per second.
        //As of Feb-13, these values are completely made up.
        kShooterSpeedTable.put(0.0, 100.0);
        kShooterSpeedTable.put(2.0, 200.0);
        kShooterSpeedTable.put(4.0, 300.0);
        kShooterSpeedTable.put(6.0, 400.0);
        kShooterSpeedTable.put(8.0, 500.0);
        kShooterSpeedTable.put(10.0, 600.0);       
    }
    */

    static {
        //Populating the shooter arm position table.  Values are in rotations.
        //(Distance in meters, Angle in rotations)
        kShooterArmTable.put(1.0033, -0.095);
        kShooterArmTable.put(2.6289, -0.0388);
        kShooterArmTable.put(4.97, -0.0035);
        //kShooterArmTable.put(6.0, 55.0);
        //kShooterArmTable.put(8.0, 45.0);
        //kShooterArmTable.put(10.0, 35.0);       
    }
}
