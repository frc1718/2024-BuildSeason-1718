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
import frc.robot.commands.General.NotePosition;
import frc.robot.commands.General.StowArmAndIntake;

/**
 * The suck command positions the front intake and shooter arm so a note can be picked up off the floor and moved into the shooter.
 */
public class Suck extends Command {
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
  public Suck(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_beamBreakSubsystem = beamBreakSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_shooterIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("=====================");
    System.out.println("Driver Command: Suck");
  
    //Reset state machine
    m_stateMachine = 1;

    //Set required positions
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);  
    m_shooterSubsystem.setShooterSpeed(0);
    
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

      //State Machine
      switch(m_stateMachine){     
        case 1:  //Front intake in position
          System.out.println("Driver Command Suck: Case 1");
          if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeDownPos) && (m_shooterSubsystem.getShooterUpToSpeed(0))) {
            System.out.println("Driver Command Suck: Case 1 Complete!");
            m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
            m_stateMachine = m_stateMachine + 1;
          }        
        break;
        case 2:  // Shooter Arm In Position
          System.out.println("Driver Command Suck: Case 2");
          if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmHomePos)) {
            m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSuckSpeed);
            m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeSuckSpeed);
            System.out.println("Driver Command Suck: Case 2 Complete!");
            m_stateMachine = m_stateMachine + 1;
          }
        break;
        case 3:  // Shooter Arm In Position
          System.out.println("Driver Command Suck: Case 3");
          if (m_beamBreakSubsystem.getNotePresentShooter()) {
            m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
            m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);

            System.out.println("Driver Command Suck: Case 3 Complete!");
          }
        break;
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
  m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
  m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeHomePos);
  System.out.println("Driver Command: Suck Finished");
  System.out.println("=====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
