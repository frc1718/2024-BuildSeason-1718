// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * The suck command positions the front intake and shooter arm so a note can be picked up off the floor and moved into the shooter.
 */
public class SuckNoFront extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;
  private final BeamBreakSubsystem m_beamBreakSubsystem;
  private int m_stateMachine = 1;
  private boolean m_isFinished = false;

    /**
   * Constructs an instance of the suck command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public SuckNoFront(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_shooterSubsystem);
    addRequirements(m_shooterIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.kPrintDriverSuckNoFront){
      System.out.println("=====================");
      System.out.println("Driver Command: SuckNoFront");
    }  
    //Reset state machine
    m_stateMachine = 1;

    m_isFinished = false;

    //Set required positions
    if (m_beamBreakSubsystem.getNotePresent()){
      m_isFinished=true;
    } else
    {
      m_isFinished = false;
      m_shooterSubsystem.setShooterSpeed(0);
      m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSuckSpeed);

    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

      //State Machine
      switch(m_stateMachine){     
        case 1:  //Front intake in position
        if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command SuckNoFront: Case 1");}
          if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
            if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command SuckNoFront: Case 1 Complete!");}
            m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
            m_stateMachine = m_stateMachine + 1;
          }        
        break;
        case 2:  // Shooter Arm In Position
          if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command SuckNoFront: Case 2");}
          if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmHomePos)) {
            if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command SuckNoFront: Case 2 Complete!");}
            m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
            m_stateMachine = m_stateMachine + 1;
          }
        break;
        case 3:  // Note in system
          System.out.println("Driver Command SuckNoFront: Case 3");
          if (m_beamBreakSubsystem.getNotePresentShooter()) {
            m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
            if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command SuckNoFront: Case 3 Complete!");}
            m_isFinished=true;
          }
        break;
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    if (m_isFinished=true) {
      if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command: SuckNoFront Finished");}
    } else {
      if (Constants.kPrintDriverSuckNoFront){System.out.println("Driver Command: SuckNoFront Interrupted!");}
    }
    if (Constants.kPrintDriverSuckNoFront){System.out.println("=====================");}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
