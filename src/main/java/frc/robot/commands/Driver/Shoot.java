// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ClimberSubsystem m_climberSubsystem;

  private boolean m_isFinished = false;
  private int m_shooterArmPosition = 0;
  private int m_shooterSpeed = 0;
  private int m_frontIntakePosition = 0;
  private int m_frontIntakeSpeed = 0;
  private int m_intakeSpeed = 0;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public Shoot(FrontIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climbSubsystem) {

    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = intakeSubsystem;
    m_climberSubsystem = climbSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.println("Driver Command: Shoot");

    //If preclimb has not been actuated, shooter runs its normal commmands here
    if(!m_climberSubsystem.getPreClimbActuated()){

      //Set values based on shooter mode the operator has selected
      if(m_shooterSubsystem.getShooterMode()=="ShootAmp"){
        System.out.println("Driver Command Shoot: We got ShootAmp from the Operator!");
        m_shooterArmPosition = Constants.kShooterArmAmpPos;
        m_shooterSpeed = Constants.kShooterAmpSpeed;
        m_frontIntakePosition = Constants.kFrontIntakeHomePos;
        m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;

      } else if (m_shooterSubsystem.getShooterMode()=="ShootTrap"){
        System.out.println("Driver Command Shoot: We got ShootTrap from the Operator!");
        m_shooterArmPosition = Constants.kShooterArmTrapPos;
        m_shooterSpeed = Constants.kShooterStopSpeed;
        m_frontIntakePosition = Constants.kFrontIntakeDownPos;
        m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;

      } else if (m_shooterSubsystem.getShooterMode()=="ShootPodium"){
        System.out.println("Driver Command Shoot: We got ShootPodium from the Operator!");
        m_shooterArmPosition = Constants.kShooterArmPodiumPos;
        m_shooterSpeed = Constants.kShooterPodiumSpeed;
        m_frontIntakePosition = Constants.kFrontIntakeHomePos;
        m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;

      } else if (m_shooterSubsystem.getShooterMode()=="ShootSubwoofer"){
        System.out.println("Driver Command Shoot: We got ShootSubwoofer from the Operator!");
        m_shooterArmPosition = Constants.kShooterArmSubwooferPos;
        m_shooterSpeed = Constants.kShooterSubwooferSpeed;
        m_frontIntakePosition = Constants.kFrontIntakeHomePos;
        m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;

      } else if (m_shooterSubsystem.getShooterMode()=="ShootWithPose"){
        //We need to calculate the shooter arm position and shooter speed here for shoot with pose.
        System.out.println("Driver Command Shoot: We got ShootWithPose from the Operator!");
        m_shooterArmPosition =0;
        m_shooterSpeed = 0;
        m_frontIntakePosition = Constants.kFrontIntakeHomePos;
        m_frontIntakeSpeed = Constants.kFrontIntakeStopSpeed;
      } 
    }

    // Set the values for the subsystems
      m_shooterSubsystem.setShooterArmPosition(m_shooterArmPosition);
      m_shooterSubsystem.setShooterSpeed(m_shooterSpeed);
      m_frontIntakeSubsystem.setFrontIntakePosition(m_frontIntakePosition);
      m_frontIntakeSubsystem.setFrontIntakeSpeed(m_frontIntakeSpeed);

    //Check to see when we are ready to shoot
    if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed)) {
      System.out.println("Driver Command Shoot: We are ready to shoot!");
      m_shooterSubsystem.setShooterReadyToShoot(true);
    }

    //Run the shooter intake if we are ready to shoot
    if (m_shooterSubsystem.getShooterReadyToShoot())
    {
      System.out.println("Driver Command Shoot: We are running the shooter intake to shoot!");
      m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeShootSpeed);
    }

    //If the note is no longer present in either sensor, and we were ready to shoot, then we've shot
    if (!m_shooterSubsystem.getNotePresentShooter() && !m_shooterSubsystem.getNotePresentIntake() && m_shooterSubsystem.getShooterReadyToShoot()) {
      m_isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Driver Command Shoot: Was interrupted!");
    m_shooterSubsystem.setShooterReadyToShoot(false);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Driver Command Shoot: Was completed!");
    m_shooterSubsystem.setShooterReadyToShoot(false);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    return m_isFinished;
  }
}
