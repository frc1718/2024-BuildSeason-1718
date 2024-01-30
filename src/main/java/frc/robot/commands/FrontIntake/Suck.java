// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FrontIntake;

import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class Suck extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   * @param frontIntakeSubsystem
   * @param shooterSubsystem The subsystem used by this command.
   */
  public Suck(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);  
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  //Check to see if front intake and shooter arm are in position
  if (m_shooterSubsystem.shooterArmInPosition(Constants.kShooterArmHomePos) && m_frontIntakeSubsystem.frontIntakeInPosition(Constants.kFrontIntakeDownPos)) 
    { 
      //If neither beam break is broken
      if (!m_shooterSubsystem.getNotePresentIntake() && !m_shooterSubsystem.getNotePresentShooter())
      {
        m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeSuckSpeed);
        m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSuckSpeed);
      
      //else if the front beam break is broken and the second isn't stop
      } else if (m_shooterSubsystem.getNotePresentIntake() && !m_shooterSubsystem.getNotePresentShooter() )
      {
        m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
        m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
      
      //else if both beam breaks are broken, back up the note at index speed
      } else if (m_shooterSubsystem.getNotePresentIntake() && m_shooterSubsystem.getNotePresentShooter()) 
      {
        m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
        m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeReverseIndexSpeed);
        
      //else if only the second beam break is broken, shouldn't ever happen unless something really wacky is happening
      } else
      {
        m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
        m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeReverseIndexSpeed);
      }

    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontIntakeSubsystem.setFrontIntakeSpeed(0);
    m_frontIntakeSubsystem.setFrontIntakePosition(0);
    m_shooterSubsystem.setShooterIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
