// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The pre-climb command moves all of the related subsystems into the 'Pre-Climb' state.
 * The front intake roller and shooter motors are stopped and both the front intake and shooter arm are moved into position.
 * The climber is moved into position so the hooks can hit the chain.
 */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final CommandXboxController m_Controller;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  
  private Rotation2d m_RotationTarget = new Rotation2d(0.0);
  private Alliance m_Alliance;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI;
  public double driveSign;
  private String driveRequest = "";
  private int m_LimelightStateMachine = 1;
  private boolean m_isFinished = false;
  private double m_AngleToAprilTag = 0;
  private double m_CurrentRobotHeading;
  private double m_NewAngleHeading;
  private PIDController aimPID = new PIDController(0.058, 0, 0.0013); // 0.055, 0, 0.0013
  private double limeLightController = 0;
  Trigger m_DriverLeftTrigger;
  private boolean LimeLightShootingFlag = false;

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
    .withRotationalDeadband(0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity);
    
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

  public Drive(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, ShooterSubsystem shooter, ClimberSubsystem climber, double DriveSign) {
    m_Drivetrain = drivetrain;
    m_Controller = controller;
    m_ShooterSubsystem = shooter;
    m_ClimberSubsystem = climber;
    driveSign = DriveSign;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("==========================");
    System.out.println("Command Operator: Home");

    //Configure the PID Controller for the 'driveFacingAngle' drive request.
    driveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveFacingAngle.HeadingController.setPID(2, 0, 0.1);

    m_DriverLeftTrigger = m_Controller.x();

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_ShooterSubsystem.getShooterModeDoingSomething()) && (Math.abs(m_Controller.getRightX()) < 0.1) && m_ShooterSubsystem.getShooterMode() == "ShootPass" && m_ShooterSubsystem.getShooterMode() != "ShootPodium" && m_ShooterSubsystem.getShooterMode() != "ShootAmp" && m_ShooterSubsystem.getShooterMode() != "ShootSubwoofer") {
      
      driveRequest = "driveFacingAngle";
      m_LimelightStateMachine = 1;

      m_Alliance = DriverStation.getAlliance().get();
      //Turn to Pose Stuff - Not Limelight Shoot
      if (m_Alliance == Alliance.Blue) {
        switch(m_ShooterSubsystem.getShooterMode()) {
          /*
          case "ShootAmp":
            m_RotationTarget = Constants.kBlueAmpAngle;
          break;
          */
          /*case "ShootPodium":
            m_RotationTarget = Constants.kBlueSpeakerLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle();
          break; */
          case "ShootPass":
            m_RotationTarget = Constants.kBluePassAngle;
          break;
          default:
            m_RotationTarget = new Rotation2d(0.0);
        }
      } else if (m_Alliance == Alliance.Red) {
          switch(m_ShooterSubsystem.getShooterMode()) {
            /*
            case "ShootAmp":
              m_RotationTarget = Constants.kRedAmpAngle;
            break;
            */
            /*case "ShootPodium":
              m_RotationTarget = Constants.kRedSpeakerLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).getAngle();
            break; */
            case "ShootPass":
              m_RotationTarget = Constants.kRedPassAngle;
            break;
            default:
              m_RotationTarget = new Rotation2d(0.0);
          }
          SmartDashboard.putNumber("ROTATION TARGET", m_RotationTarget.getDegrees());
          SmartDashboard.putNumber("PID Out", driveFacingAngle.HeadingController.getLastAppliedOutput());
        }
    } else if (m_ClimberSubsystem.getPreClimbActuated()) {

        driveRequest = "robotCentric";
        m_LimelightStateMachine = 1;

          /* switch(m_ClimberSubsystem.getClimbLocation()) {
            case "RightClimb":
            m_RotationTarget = Constants.kBlueRightClimbAngle;
            break;
            case "LeftClimb":
            m_RotationTarget = Constants.kBlueLeftClimbAngle;
            break;
            case "FarClimb":
            m_RotationTarget = Constants.kBlueFarClimbAngle;
            break;
            default:
            m_RotationTarget = new Rotation2d(0.0); */

    } else if ((!m_ShooterSubsystem.getShooterModeDoingSomething()) && m_Controller.leftBumper().getAsBoolean() && (LimelightHelpers.getTV(Constants.kLimelightName) || LimeLightShootingFlag)) {
        
        driveRequest = "limeLightAim";
        LimeLightShootingFlag = true;
        switch (m_LimelightStateMachine) {
          case 1: //Assume the AprilTag is already in view.
            m_AngleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
            m_CurrentRobotHeading = m_Drivetrain.getPigeon2().getAngle();
            m_NewAngleHeading = m_AngleToAprilTag + m_CurrentRobotHeading;
            //m_RotationTarget = Rotation2d.fromDegrees(m_NewAngleHeading);
            SmartDashboard.putNumber("LIMELIGHT TX", m_AngleToAprilTag);
            SmartDashboard.putNumber("ROBOT HEADING (Pigeon)", m_CurrentRobotHeading);
            SmartDashboard.putNumber("LIMELIGHT ROTATION TARGET", m_RotationTarget.getDegrees());
            
            m_LimelightStateMachine++;
          break;
          case 2: //Wait for the robot to turn within tolerance.
           // if (Math.abs(aimPID.getPositionError()) <= Constants.kTXTolerance) {
          //  m_LimelightStateMachine++;
           // }
          break;
          case 3:
          break;
          case 4:
          break;
          default:

        }
    } else {
        driveRequest = "";
        m_LimelightStateMachine = 1;
        LimeLightShootingFlag = false;
    }
    
    switch (driveRequest) {
      case "driveFacingAngle":  //Always face the rotation target
        m_Drivetrain.setControl(driveFacingAngle.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * driveSign)
          .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * driveSign)
          .withTargetDirection(m_RotationTarget));
      break;
      case "robotCentric":  //Enter 'First-Person Mode'
        m_Drivetrain.setControl(robotCentric.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * driveSign) // Drive forward with
                                                                                            // negative Y (forward)
          .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * driveSign) // Drive left with negative X (left)
          .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate));
      break;
      case "limeLightAim":  //Use the limelight to aim to an AprilTag

           limeLightController = aimPID.calculate(m_Drivetrain.getPigeon2().getAngle(), m_NewAngleHeading);

           if (limeLightController > 1) {
            limeLightController = 1;
           } else if (limeLightController < -1) {
            limeLightController = -1;
           }
           //System.out.println("Pigeon Angle: "+ m_Drivetrain.getPigeon2().getAngle());
           //System.out.println("LimeLightControllerValue: "+ limeLightController);
           //System.out.println("Desired Angle: "+  m_NewAngleHeading);

            m_Drivetrain.setControl(drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * driveSign) // Drive forward with                                                                    
          .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * driveSign) // Drive left with negative X (left)
          .withRotationalRate(-limeLightController * MaxAngularRate)); // Drive counterclockwise with negative X (left) 
      
      
       break;                                                              
      default:  //Just drive normally
        m_Drivetrain.setControl(drive.withVelocityX(-m_Controller.getLeftY() * MaxSpeed * driveSign) // Drive forward with
                                                                                            // negative Y (forward)
          .withVelocityY(-m_Controller.getLeftX() * MaxSpeed * driveSign) // Drive left with negative X (left)
          .withRotationalRate(-m_Controller.getRightX() * MaxAngularRate)); // Drive counterclockwise with negative X (left)      
      break;
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator Home: Command Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
