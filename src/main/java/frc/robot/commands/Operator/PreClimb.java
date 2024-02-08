// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands.Operator;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class PreClimb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;

  private boolean m_isFinished = false;
  private boolean preClimbActuated = false;
  private int m_stateMachine=1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public PreClimb(ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem, FrontIntakeSubsystem frontIntakeSubsystem) {
    m_climberSubsystem = climberSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("==========================");
    System.out.println("Command Operator: PreClimb");

    //Initialize State Machine
    m_stateMachine = 1;

    //In the initialize step, set the desired starting positions and speeds of each system
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);
 
    m_climberSubsystem.setClimberDesiredPosition(Constants.kClimberPreClimbPos);

    m_shooterSubsystem.setShooterSpeed(Constants.kShooterStopSpeed);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    
    m_isFinished = false;

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine){     
      case 1:  //Front intake in position
        System.out.println("Operator Command Preclimb: Case 1 Started");
        if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeDownPos)) {
          m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmPreClimbPos);
          System.out.println("Operator Command Preclimb: Case 1 Complete");
          m_stateMachine=m_stateMachine+1;
        }        
        break;
      case 2:  // Arm in position
        System.out.println("Operator Command Preclimb: Case 2 Started");
        if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmPreClimbPos)) {
          System.out.println("Operator Command Preclimb: Case 2 Complete");
          m_stateMachine=m_stateMachine+1;
        }
        break;
      case 3:  // Climber in position
        System.out.println("Operator Command Preclimb: Case 3 Started");       
        if (m_climberSubsystem.getClimberInPosition(Constants.kClimberPreClimbPos)) {
          System.out.println("Operator Command Preclimb: Case 3 Complete");
          m_climberSubsystem.setPreClimbActuated(true);
          m_isFinished= true;
        }
        break;
    }
    //Remove this once the conditional check below is working.  For now, this makes sure the preclimb command finishes.
    m_isFinished = true;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator PreClimb: Command Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
