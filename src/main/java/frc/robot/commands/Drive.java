// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.Driver.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private boolean m_isFinished = false;

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
    driveFacingAngle.HeadingController.setPID(20, 0, 0.05);

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_ShooterSubsystem.getShooterModeDoingSomething()) && (Math.abs(m_Controller.getRightX()) < 0.1)) {
      
      driveRequest = "driveFacingAngle";

      m_Alliance = DriverStation.getAlliance().get();
    
      if (m_Alliance == Alliance.Blue) {
        switch(m_ShooterSubsystem.getShooterMode()) {
          case "ShootAmp":
            m_RotationTarget = Constants.kBlueAmpAngle;
          break;
          case "ShootPodium":
            m_RotationTarget = Constants.kBlueSpeakerLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle();
          break;
          case "ShootPass":
            m_RotationTarget = Constants.kBluePassLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle();
          break;
          default:
            m_RotationTarget = new Rotation2d(0.0);
        }
      } else if (m_Alliance == Alliance.Red) {
          switch(m_ShooterSubsystem.getShooterMode()) {
            case "ShootAmp":
              m_RotationTarget = Constants.kRedAmpAngle;
            break;
            case "ShootPodium":
              m_RotationTarget = Constants.kRedSpeakerLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).getAngle();
            break;
            case "ShootPass":
              m_RotationTarget = Constants.kRedPassLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).getAngle();
            break;
            default:
              m_RotationTarget = new Rotation2d(0.0);
          }
        }
    } else if (m_ClimberSubsystem.getPreClimbActuated()) {

        driveRequest = "robotCentric";

        switch(m_ShooterSubsystem.getShooterMode()) {
          case "ShootAmp":
            m_RotationTarget = Constants.kRedAmpAngle;
          break;
          case "ShootPodium":
            m_RotationTarget = Constants.kRedSpeakerLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle();
          break;
          case "ShootPass":
            m_RotationTarget = Constants.kRedPassLocation.minus(m_Drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle();
          break;
          default:
            m_RotationTarget = new Rotation2d(0.0);

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
        }
      } else {
        driveRequest = "";
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
