// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import frc.robot.Constants;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
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

  /**
   * Constructs an instance of the note position command.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public NotePosition(ShooterIntakeSubsystem shooterIntakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {

    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========================");
    System.out.println("General NotePosition Started");
    
    //Initialize state machine
    m_stateMachine = 1;

    m_isFinished = false;

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine){
      case 1:  //If note isn't here, why are you here?
          if (!m_beamBreakSubsystem.getNotePresent()){
            m_isFinished = true;
          }
          else{
          m_stateMachine=m_stateMachine+1;
          }
        break;
      case 2:  //If note is breaking shooter beam, back it up.
        System.out.println("General NotePosition: Case 1");
        if (m_beamBreakSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeIndexSpeed);  
        } else {
          System.out.println("General NotePosition: Case 1 Complete");
          m_stateMachine = m_stateMachine + 1; 
        }    
        break; 
      case 3:  //If Note is breaking intake beam, stop indexing.
        System.out.println("General NotePosition: Case 2");
        if (m_beamBreakSubsystem.getNotePresentIntake()) {
          System.out.println("General NotePosition: Case 2 Complete");
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);  
          m_isFinished = true;
        } else {
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeIndexSpeed);
        }
        break;
    }   

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("General NotePosition Completed");
    System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return m_isFinished;
  }
}
