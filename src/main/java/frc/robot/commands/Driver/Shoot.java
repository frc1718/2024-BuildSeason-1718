// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

/**
 * The shoot command sets the shooter speed and arm position based on the current shooter mode.
 * It also performs the actual <i>shooting</i>.
 */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;
  private final BeamBreakSubsystem m_beamBreakSubsystem;

  private boolean m_isFinished = false;
  private boolean m_IgnoreLimelight = true;

  private double m_shooterArmPosition = 0;
  private double m_shooterSpeed = 0;
  private double m_frontIntakePosition = 0;
  private double m_frontIntakeSpeed = 0;

  private double m_VerticalAngleToAprilTag;
  private double m_HorizontalAngleToAprilTag;
  private double m_DistanceToAprilTag;
  private double m_DistanceBetweenAprilTagAndLimelight = Constants.kSpeakerAprilTagHeight - Constants.kLimelightHeight;

  private int m_stateMachine = 1;

  Timer shootTimer = new Timer();

  /**
   * Constructs an instance of the shoot command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param climbSubsystem An instance of the climber subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public Shoot(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climbSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {

    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_climberSubsystem = climbSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_climberSubsystem);
    addRequirements(m_shooterIntakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    if (Constants.kPrintDriverShoot){
      System.out.println("========================");
      System.out.println("Driver Command: Shoot");
    }
    //Initialize state machine
    m_stateMachine = 1;

    shootTimer.reset();

    m_isFinished = false;

    //If preclimb has not been actuated, shooter runs its normal commmands here
    if(!m_climberSubsystem.getPreClimbActuated()){
      
      //Set values based on shooter mode the operator has selected
      switch(m_shooterSubsystem.getShooterMode()){     
        case "ShootAmp":
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: We got ShootAmp from the Operator!");}
          m_shooterArmPosition = Constants.kShooterArmAmpPos;
          m_shooterSpeed = Constants.kShooterAmpSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
          break;
        case "ShootTrap":
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: We got ShootTrap from the Operator!");}
          m_shooterArmPosition = Constants.kShooterArmTrapPos;
          m_shooterSpeed = Constants.kShooterStopSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeDownPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
          break;
        case "ShootPodium":
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: We got ShootPodium from the Operator!");}
          m_shooterArmPosition = Constants.kShooterArmPodiumPos;
          m_shooterSpeed = Constants.kShooterPodiumSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
          break;
        case "ShootSubwoofer":
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: We got ShootSubwoofer from the Operator!");}
          m_shooterArmPosition = Constants.kShooterArmSubwooferPos;
          m_shooterSpeed = Constants.kShooterSubwooferSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
          break;
        case "ShootWithPose":
          //We need to calculate the shooter arm position and shooter speed here for shoot with pose.
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: We got ShootWithPose from the Operator!");}
          break;
        case "DoNothing": //In "DoNothing", we need to also check if there is a valid limelight target.
          if (LimelightHelpers.getTV(Constants.kLimelightName)) {
            m_shooterSpeed = Constants.kShooterLimelightSpeed;
            m_frontIntakePosition = Constants.kFrontIntakeClearPos;
            m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
            m_IgnoreLimelight = false;
            m_stateMachine=0;
          } else {
              m_shooterArmPosition = Constants.kShooterArmHomePos;
              m_shooterSpeed = Constants.kShooterStopSpeed;
              m_frontIntakePosition = Constants.kFrontIntakeHomePos;
              m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
              m_IgnoreLimelight = true;            
          }
        break;
        case "AutoShooterModePos1":
          m_shooterArmPosition = Constants.kShooterArmAutoPos1Pos;
          m_shooterSpeed = Constants.kShooterAutoPos1Speed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
        break;
        case "AutoShooterModePos2":
          m_shooterArmPosition = Constants.kShooterArmAutoPos2Pos;
          m_shooterSpeed = Constants.kShooterAutoPos2Speed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
        break;
        case "AutoShooterModePos3":
          m_shooterArmPosition = Constants.kShooterArmAutoPos3Pos;
          m_shooterSpeed = Constants.kShooterAutoPos3Speed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
        break;
        case "ShootPass":
          m_shooterArmPosition = Constants.kShooterArmPassPos;
          m_shooterSpeed = Constants.kShooterPassSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          m_IgnoreLimelight = true;
        break;
      }

      // Set the values for the subsystems
      m_shooterSubsystem.setShooterSpeed(m_shooterSpeed);
      m_frontIntakeSubsystem.setFrontIntakePosition(m_frontIntakePosition);
      m_frontIntakeSubsystem.setFrontIntakeSpeed(m_frontIntakeSpeed);
      m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

       m_HorizontalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTX(Constants.kLimelightName));

      if ((m_shooterSubsystem.getShooterMode()=="DoNothing") && (Math.abs(m_HorizontalAngleToAprilTag) < 2) && (m_stateMachine==0)) { 
            m_VerticalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTY(Constants.kLimelightName));     
            m_DistanceToAprilTag = m_DistanceBetweenAprilTagAndLimelight / (Math.tan(m_VerticalAngleToAprilTag) * Math.cos(m_HorizontalAngleToAprilTag));
            m_shooterArmPosition = Constants.kShooterArmTable.get(m_DistanceToAprilTag);
            m_stateMachine=1;
      }


    //Statemachine woop woop
    switch(m_stateMachine){  
      case 1:  //Do Nothing
      if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine);}
        if((m_shooterSubsystem.getShooterMode()=="DoNothing") && (!LimelightHelpers.getTV(Constants.kLimelightName))){ //Are we do nothing and No limelight
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine + " Nothing To Do! Exiting!");}
          m_isFinished=true;
        } else {
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine + " Complete!");}
          m_stateMachine = m_stateMachine + 1;
        }
      break;
      case 2:  //Check Note Present
        if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine);}
        if (!m_beamBreakSubsystem.getNotePresent()) {
          m_isFinished = true;
        } else {
          m_stateMachine = m_stateMachine + 1;
        }
      break;
      case 3:  //Front intake in position
        if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine);}
        //System.out.println(m_frontIntakeSubsystem.getFrontIntakePosition());
        if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine + " Complete!");}
          m_shooterSubsystem.setShooterArmPosition(m_shooterArmPosition);
          m_shooterSubsystem.setShooterSpeed(m_shooterSpeed);
          m_stateMachine = m_stateMachine + 1;
        }        
      break;
      case 4:  // Shooter up to speed and arm is in position, run shooter intake to shoot
          if (Constants.kPrintDriverShoot){
            System.out.println("Driver Command Shoot: Case " + m_stateMachine);
            System.out.println("Driver Command Shooter Target:" + m_shooterSpeed);
            System.out.println("Driver Command Shooter Speed: " + m_shooterSubsystem.getLeftShooterSpeed());
          }
          if ((m_shooterSubsystem.getShooterUpToSpeed(m_shooterSpeed) && m_shooterSubsystem.getShooterArmInPosition(m_shooterArmPosition)) && (m_IgnoreLimelight || (Math.abs(LimelightHelpers.getTX(Constants.kLimelightName)) <= Constants.kTXTolerance))) {
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeShootSpeed);
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case " + m_stateMachine + " Complete!");}
          shootTimer.start();
          m_stateMachine = m_stateMachine + 1;
        }
      break;
      case 5:  //Wait for the shooter speed to drop a certain amount for the shot to be complete.  Note should be gone, timeer
      if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case" + m_stateMachine);}
        if (!m_beamBreakSubsystem.getNotePresent() && shootTimer.get() > 0.250) {
          m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
          m_shooterSubsystem.setShooterMode("DoNothing");
          if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Case 3 Complete!");}
          m_isFinished = true;
        }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Always have to set ready to shoot back to false at the end of a shot.
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterPodiumSpeed);
    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);

    m_shooterSubsystem.setShooterMode("DoNothing");
    
    if (m_isFinished==true) {
      if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Was completed!");}
    } else {
      if (Constants.kPrintDriverShoot){System.out.println("Driver Command Shoot: Was intterupted!");} 
    }
    if (Constants.kPrintDriverShoot){System.out.println("=======================================");}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
