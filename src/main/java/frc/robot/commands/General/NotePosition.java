// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands.General;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NotePosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  private boolean m_isFinished = false;

  private int m_stateMachine = 1;

  public NotePosition(ShooterIntakeSubsystem shooterIntakeSubsystem) {

    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterIntakeSubsystem);

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
      case 1:  //If Note is breaking intake beam, stop indexing.
        System.out.println("General NotePosition: Case 1");
        if (m_shooterIntakeSubsystem.getNotePresentIntake()) {
          System.out.println("General NotePosition: Case 1 Complete");
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);  
          m_stateMachine = m_stateMachine + 1;
        } else {
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeIndexSpeed);
        }
        break;
      case 2:  //If note is breaking shooter beam, back it up.
        System.out.println("General NotePosition: Case 2");
        if (m_shooterIntakeSubsystem.getNotePresentShooter()){
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeIndexSpeed);  
        } else {
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);  
          m_isFinished = true;
        }
    }   

    //Remove once all logic is in place.
    m_isFinished=true;
    
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
