// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

/**
 * The shoot trap command rotates the shooter arm to the correct position for scoring a note in the trap.
 * <p>As a bonus, it also spits it out.
 */
public class ShootTrap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  private boolean m_isFinished = false;

  private int m_stateMachine = 0;

  /**
   * Constructs an instance of the shoot trap command.
   * 
   * @param climbSubsystem An instance of the climber subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public ShootTrap(ShooterIntakeSubsystem shooterIntakeSubsystem) {
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(m_shooterIntakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    if (Constants.kPrintDriverShootTrap){
      System.out.println("========================");
      System.out.println("Driver Command: ShootTrap");
    }
    m_isFinished = false;
    m_stateMachine=1;
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {/*
    switch(m_stateMachine) {
      case 1:  //Set values based on shooter mode the operator has selected
        if (m_climberSubsystem.getPreClimbActuated()) {
          if (Constants.kPrintDriverShootTrap){System.out.println("Driver Command ShootTrap: Case 1!");}
          m_shooterIntakeSubsystem.setShooterIntakeRotate(Constants.kShooterIntakeTrapRotations);
          m_stateMachine = m_stateMachine + 1;
        }
      break;
      case 2:
          if (Constants.kPrintDriverShootTrap){System.out.println("Driver Command ShootTrap: Case 2!");}





      break;
      case 3:
        if (Constants.kPrintDriverShootTrap){System.out.println("Driver Command ShootTrap: Case 3!");}
        //System.out.println(m_shooterIntakeSubsystem.getShooterIntakePosition());
        if (m_shooterIntakeSubsystem.getShooterIntakeInPosition(Constants.kShooterIntakeTrapRotations)){
          System.out.println("Inside Set Shooter Intake Speed");
          m_shooterIntakeSubsystem.setShooterIntakeSpeed(-15);
        }
      break;
    }
    */
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Always have to set ready to shoot back to false at the end of a shot.
    if (Constants.kPrintDriverShootTrap){
      System.out.println("Driver Command ShootTrap: Was completed!");
      System.out.println("=======================================");
 
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
