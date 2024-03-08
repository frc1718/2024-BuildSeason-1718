// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The climb command controls the actual climbing of the chain.
 * The robot can only attempt to climb once the climber subsytem has reached the pre-climb position.
 */
public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private boolean m_isFinished = false;

  private int m_stateMachine = 1;
  /**
   * Constructs an instance of the climb command.
   * @param climberSubsystem An instance of the climber subsystem.
   * Required.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   */
  public Climb(ClimberSubsystem climberSubsystem, FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_climberSubsystem = climberSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.kPrint){
      System.out.println("========================");
      System.out.println("Driver Command: Climb");
    }

    //Initialize state machine
    m_stateMachine = 1;

    m_isFinished = false;
    
    //Set Positions and speeds
    if (m_climberSubsystem.getPreClimbActuated()) { 
      m_climberSubsystem.setClimberDesiredPosition(Constants.kClimberPreClimbPos);
      m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);
      m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
      m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmPreClimbPos);
    } 

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine) {     
      case 1:  //PreClimbActuated
        if (Constants.kPrint){
          //System.out.println("Driver Command Climb: Case 1");
        }
        if (m_climberSubsystem.getPreClimbActuated()) {
          if (Constants.kPrint){
          //System.out.println("Driver Command Climb: Case 1 Complete");
          }
          m_climberSubsystem.setClimberDesiredPosition(Constants.kClimberClimbPos);
          m_stateMachine = m_stateMachine + 1;
        } else {
          //System.out.println("Driver Command: Preclimb has not been actuated!  Abort!");
          m_isFinished = true;
        }
        break;
      case 2:  //Climb Complete
        System.out.println("Driver Command Climb: Case 2");
        if (m_climberSubsystem.getClimberInPosition(Constants.kClimberClimbPos)){
          //System.out.println("Driver Command Climb: Case 2 Complete.");
          m_climberSubsystem.setClimberDesiredPosition(Constants.kClimberClimbPos);
        }
    }   

    //Remove once climber is in place.  Here so the command finishes for debugging.
    m_isFinished = true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_isFinished=true) {
      //System.out.println("Driver Command: Climb completed!");
    } else {
      //System.out.println("Driver Command: Climb interrupted!");
    }
      //System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return m_isFinished;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("Climb");
    builder.addIntegerProperty("State Machine", () -> {return this.m_stateMachine;}, null);
  }
}
