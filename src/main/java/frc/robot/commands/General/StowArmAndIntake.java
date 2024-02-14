// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The stow arm and intake command moves the front intake roller and shooter arm into their safe 'stow' positions.
 * The front intake roller is moved so the shooter arm can clear it, then the shooter arm is moved to it's home position.
 * <p> Stow, home; both terms are somewhat interchangable here.
 */
public class StowArmAndIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private boolean m_isFinished = false;

  private int m_stateMachine = 1;

  /**
   * Constructs an instance of the stow arm and intake command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   */
  public StowArmAndIntake(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {

    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========================");
    System.out.println("General StowArmAndIntake Started");
    
    //Initialize state machine
    m_stateMachine = 1;
    
    //Set Positions and speeds
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);

    m_isFinished = false;

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine){     
      case 1:  //Move Front Intake to Clear Position
        System.out.println("General StowArmAndIntake: Case 1");
        if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
          System.out.println("General StowArmAndIntake: Case 1 Complete");
          m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
          m_stateMachine = m_stateMachine + 1;
        }
        break;
      case 2:  //When arm is home, end command
        System.out.println("General StowArmAndIntake: Case 2");
        if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmHomePos)){
          System.out.println("General StowArmAndIntake: Case 2 Complete.");
          m_isFinished = true;
        }
    }   

    //Remove once all logic is in place.
    m_isFinished=true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("General StowArmAndIntake Completed");
    System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return m_isFinished;
  }
}
