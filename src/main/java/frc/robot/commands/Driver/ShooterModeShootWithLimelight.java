// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

/**
* Aims with limelight
*/
public class ShooterModeShootWithLimelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  private final SwerveRequest.FieldCentricFacingAngle m_aim = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband((TunerConstants.kSpeedAt12VoltsMps) * 0.1)
  .withRotationalDeadband((1.5 * Math.PI) * 0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withVelocityX(0)
  .withVelocityY(0);

  private final SwerveRequest.FieldCentric m_PIDAim = new SwerveRequest.FieldCentric()
  .withDeadband((TunerConstants.kSpeedAt12VoltsMps) * 0.1)
  .withRotationalDeadband((1.5 * Math.PI) * 0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withVelocityX(0)
  .withVelocityY(0);

  /*
   * Check for Fiducal 'Whatever it is (7?)'                            CASE 1
   * If present, get TX - If not, quit command (or signal in some way)  CASE 1
   * Add (or subtract) TX as angle, to / from current robot angle       CASE 1
   * That is your setpoint                                              CASE 1
   * Set swerve control the face the setpoint angle
   * Do so until TX is 0, or near 0
   */
  
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;
  private double m_shooterPoseSpeed = 70;
  private double m_shooterArmPosePos = 0;
  private double m_distanceToAprilTag = 0;
  private double m_angleToAprilTag = 0;
  private double m_currentRobotHeading = 0;
  private double m_newAngleHeading = 0;
  private int m_debounceCounter = 0;
  private int m_debounceLimit = 0;
  private Rotation2d m_limeLightRotation;
  private double m_limeLightToAprilTagVerticalDistance = (Constants.kSpeakerAprilTagHeight - Constants.kLimelightHeight);
  private double m_verticalAngleToAprilTag = 0;
  private double m_quickErrCalc = 0;
  private double m_kP = 0.01;

  /**
   * Constructs an instance of the aim with limelight command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param drivetrain An instance of the drivetrain subsystem.
   * Required.
   */
  public ShooterModeShootWithLimelight(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, ShooterIntakeSubsystem shooterIntakeSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_drivetrain = drivetrain;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

    m_aim.HeadingController.setPID(20, 0, 0.05);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_drivetrain);
    addRequirements(m_shooterIntakeSubsystem);
  }

  public Rotation2d helpMe(double degrees) {
    return new Rotation2d(degrees);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    System.out.println("==========================");
    System.out.println("Command Operator: AimWithLimelight");
   
    //Initialize State Machine
    m_stateMachine = 1;

    m_debounceCounter = 0;
    m_debounceLimit = 0;
    
    //Set initial speeds and positions
    //m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);
    //m_shooterSubsystem.setShooterMode("ShootWithPose");

    m_isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine) {     
      case 1:  // Search for AprilTag
        System.out.println("AimWithLimelight - Case 1 - Search for AprilTag");
        
        if (LimelightHelpers.getTV(Constants.kLimelightName)) {//As I understand, the pipeline defines the AprilTag to look for.  There may be a way to further refine.
          System.out.println("AimWithLimelight - Case 1 - AprilTag found!");
          //Get angles.
          m_angleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
          System.out.println("AimWithLimelight - Case 1 - Angle to AprilTag: " + m_angleToAprilTag);
          //m_currentRobotHeading = m_drivetrain.getState().Pose.getRotation().getDegrees();
          m_currentRobotHeading = m_drivetrain.getPigeon2().getAngle();
          System.out.println("AimWithLimelight - Case 1 - Current Robot Heading: " + m_currentRobotHeading);
          m_newAngleHeading = m_angleToAprilTag + m_currentRobotHeading;
          System.out.println("AimWithLimelight - Case 1 - New Angle Heading: " + m_newAngleHeading);
          m_limeLightRotation = helpMe(m_newAngleHeading);

          m_verticalAngleToAprilTag = LimelightHelpers.getTY(Constants.kLimelightName);
          System.out.println("AimWithLimelight - Case 1 - Vertical Angle to AprilTag: " + m_verticalAngleToAprilTag);
          m_distanceToAprilTag = m_limeLightToAprilTagVerticalDistance / Math.tan(m_verticalAngleToAprilTag);
          System.out.println("AimWithLimelight - Case 1 - Distance to AprilTag: " + m_distanceToAprilTag);

          m_stateMachine = m_stateMachine + 1;
        }
        else {//Stop the command.
          System.out.println("AimWithLimelight - Case 1 - No AprilTag found!");
          //Signal to the driver with the LEDs, or something.
          m_isFinished = true;
        }        
        break;

      case 2:  //Rotate to the new angle.
        System.out.println("AimWithLimelight - Case 2 - Rotate to position");
        m_quickErrCalc = m_newAngleHeading - m_drivetrain.getPigeon2().getAngle();
        System.out.println("AimWithLimelight - Case 2 - Angle Error: " + m_quickErrCalc);
        m_drivetrain.setControl(m_PIDAim.withRotationalRate(-(m_quickErrCalc * m_kP) * (1.5 * Math.PI)));
        if (Math.abs(LimelightHelpers.getTX(Constants.kLimelightName)) <= Constants.kTXTolerance) {
          if (m_debounceCounter >= m_debounceLimit) {
            m_stateMachine = m_stateMachine + 1;
          } else {
            m_debounceCounter++;
          }
          System.out.println("AimWithLimelight - Case 2 - Properly rotated!");
        }
        else {
          m_debounceCounter = 0;
        }
        break;

      case 3: //Set arm to proper position and spin up shooter
      m_shooterArmPosePos = Constants.kShooterArmTable.get(m_distanceToAprilTag);
      m_shooterSubsystem.setShooterArmPosition(m_shooterArmPosePos);
        m_shooterSubsystem.setShooterSpeed(m_shooterPoseSpeed);
        if (m_shooterSubsystem.getShooterArmInPosition(m_shooterArmPosePos)) {
          m_stateMachine = m_stateMachine + 1;
        }
        break;

      case 4:
        if (m_shooterSubsystem.getShooterUpToSpeed(m_shooterPoseSpeed)) {
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeShootSpeed);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator ShooerModeShootWithPose: Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
