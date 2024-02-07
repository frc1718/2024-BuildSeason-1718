// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;

  private boolean m_isFinished = false;

  private int m_shooterArmPosition = 0;
  private int m_shooterSpeed = 0;
  private int m_frontIntakePosition = 0;
  private int m_frontIntakeSpeed = 0;
  private int m_shooterState=1;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public Shoot(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climbSubsystem) {

    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_climberSubsystem = climbSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_climberSubsystem);
    
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    System.out.println("========================");
    System.out.println("Driver Command: Shoot");

    m_isFinished = false;

    //If preclimb has not been actuated, shooter runs its normal commmands here
    if(!m_climberSubsystem.getPreClimbActuated()){
      
      //Set values based on shooter mode the operator has selected
      switch(m_shooterSubsystem.getShooterMode()){     
        case "ShootAmp":
          System.out.println("Driver Command Shoot: We got ShootAmp from the Operator!");
          m_shooterArmPosition = Constants.kShooterArmAmpPos;
          m_shooterSpeed = Constants.kShooterAmpSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          break;
        case "ShootTrap":
          System.out.println("Driver Command Shoot: We got ShootTrap from the Operator!");
          m_shooterArmPosition = Constants.kShooterArmTrapPos;
          m_shooterSpeed = Constants.kShooterStopSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeDownPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          break;
        case "ShootPodium":
          System.out.println("Driver Command Shoot: We got ShootPodium from the Operator!");
          m_shooterArmPosition = Constants.kShooterArmPodiumPos;
          m_shooterSpeed = Constants.kShooterPodiumSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          break;
        case "ShootSubwoofer":
          System.out.println("Driver Command Shoot: We got ShootSubwoofer from the Operator!");
          m_shooterArmPosition = Constants.kShooterArmSubwooferPos;
          m_shooterSpeed = Constants.kShooterSubwooferSpeed;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
          break;
        case "ShootWithPose":
          //We need to calculate the shooter arm position and shooter speed here for shoot with pose.
          System.out.println("Driver Command Shoot: We got ShootWithPose from the Operator!");
          m_shooterArmPosition =0;
          m_shooterSpeed = 0;
          m_frontIntakePosition = Constants.kFrontIntakeClearPos;
          m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
        break;
      }
    }

    // Set the values for the subsystems
    m_shooterSubsystem.setShooterSpeed(m_shooterSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(m_frontIntakePosition);
    m_frontIntakeSubsystem.setFrontIntakeSpeed(m_frontIntakeSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  //Set values based on shooter mode the operator has selected
      switch(m_shooterState){     
        case 1:  //Front intake in position
          System.out.println("Driver Command Shoot: Case 1");
          if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
            m_shooterSubsystem.setShooterArmPosition(m_shooterArmPosition);
            m_shooterState=m_shooterState+1;
          }        
          break;
        case 2:  // Shooter up to speed and arm is in position
          System.out.println("Driver Command Shoot: Case 2");
          if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed) && m_shooterSubsystem.getShooterArmInPosition(m_shooterArmPosition)) {
            System.out.println("Driver Command Shoot: We are ready to shoot!");
            m_shooterSubsystem.setShooterReadyToShoot(true);
            m_shooterState=m_shooterState+1;
          }
          break;
        case 3:  //Run the shooter intake if we are ready to shoot
          System.out.println("Driver Command Shoot: Case 3");       
          if (m_shooterSubsystem.getShooterReadyToShoot()) {
            System.out.println("Driver Command Shoot: We are running the shooter intake to shoot!");
            m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeShootSpeed);
            m_shooterState=m_shooterState+1;
          }
          break;
        case 4:  //Wait for the shooter speedto drop a certain amount for the shot to be complete.
          System.out.println("Driver Command Shoot: Case 4");
          if (m_shooterSubsystem.getShooterSpeed() < m_shooterSpeed - Constants.kShooterShotSpeedDrop) {
            m_shooterSubsystem.setShooterReadyToShoot(false);
            m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
            m_shooterState=m_shooterState+1;
          }
          break;
        case 5:  //When the arm is at home position move the intake to stow position
          System.out.println("Driver Command Shoot: Case 5");
          if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmHomePos)) {
            m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeStowPos);
            m_isFinished= true;
          }
          break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Always have to set ready to shoot back to false at the end of a shot.
    m_shooterSubsystem.setShooterReadyToShoot(false);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterIdleSpeed);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    m_shooterState=1;
    System.out.println("Driver Command Shoot: Was completed!");
    System.out.println("=======================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
