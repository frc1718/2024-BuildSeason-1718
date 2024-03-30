// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import frc.robot.Constants;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The note position command gets the note into the correct position in the shooter: fully in contact with the intake beam break, but barely in contact with the shooter beam break.
 */
public class NotePosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;
  private final BeamBreakSubsystem m_beamBreakSubsystem;

  private boolean m_isFinished = false;

  private int m_stateMachine = 1;
  Timer reverseTimer = new Timer();

  /**
   * Constructs an instance of the note position command.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public NotePosition(ShooterIntakeSubsystem shooterIntakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {

    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.kPrintGeneralNotePosition){
      System.out.println("========================");
      System.out.println("General NotePosition Started");
    }
    
    //Initialize state machine
    m_stateMachine = 1;

    m_isFinished = false;

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine){
      case 1:  //If note isn't here, why are you here?  If it is here but only in the front intake, move it into the shooter
        if (!m_beamBreakSubsystem.getNotePresent()){
          m_isFinished = true;
        } else if (m_beamBreakSubsystem.getNotePresentIntake() && !m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeIndexSpeed); 
          m_stateMachine=m_stateMachine+1;
        } else {  //We have a note at both intake and shooter
          m_stateMachine=m_stateMachine+1;
        }
      break;
      case 2:  //If Note is breaking Shooter, then back up note and start timer
        if (Constants.kPrintGeneralNotePosition){System.out.println("General NotePosition: Case 2");}
        if (m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeIndexSpeed);
          reverseTimer.reset();
          reverseTimer.start();
          m_stateMachine = m_stateMachine + 1; 
        }
      break; 
      case 3:  //If timer is complete, run note back forward
       if (reverseTimer.get() > 0.25){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeIndexSpeed);  
           m_stateMachine = m_stateMachine + 1;
        } 

      break;
      case 4:   //If Note is breaking Shooter, then back up note and start timer
        if (Constants.kPrintGeneralNotePosition){System.out.println("General NotePosition: Case 2");}
        if (m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeIndexSpeed);
          reverseTimer.reset();
          reverseTimer.start();
          m_stateMachine = m_stateMachine + 1; 
        }
      break; 
      case 5:  //If timer is complete, run note back forward
        if (reverseTimer.get() > 0.25){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeIndexSpeed);  
           m_stateMachine = m_stateMachine + 1;
        } 
      break;
      case 6:  //Stop and back note up again
        if (m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeIndexSpeed);
        }
        m_stateMachine = m_stateMachine + 1;  
      break; 
      case 7:  //If note is clear of shooter beam, stop
                if (!m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
        }
        m_isFinished = true;
      break;
    }   

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    if (Constants.kPrintGeneralNotePosition){
      System.out.println("General NotePosition Completed");
      System.out.println("================================");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return m_isFinished;
  }
}
