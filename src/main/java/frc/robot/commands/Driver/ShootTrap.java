// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class ShootTrap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  private boolean m_isFinished = false;

  private int m_shooterSpeed = 0;
  private int m_frontIntakePosition = 0;
  private int m_frontIntakeSpeed = 0;

  private int m_stateMachine = 0;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShootTrap(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climbSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem) {

    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_climberSubsystem = climbSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

    m_stateMachine = 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_climberSubsystem);
    addRequirements(m_shooterIntakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    System.out.println("========================");
    System.out.println("Driver Command: ShootTrap");

    m_isFinished = false;

    // Set the values for the subsystems
    m_shooterSubsystem.setShooterSpeed(m_shooterSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(m_frontIntakePosition);
    m_frontIntakeSubsystem.setFrontIntakeSpeed(m_frontIntakeSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  //Set values based on shooter mode the operator has selected
    if (m_climberSubsystem.getPreClimbActuated()) {
      System.out.println("Driver Command ShooTrapt: Case 1 Complete!");
      m_shooterIntakeSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeShootSpeed);
      m_isFinished = true;
    } else {
      System.out.println("Driver Command ShootTrap: Preclimb Not Actuated!");
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Always have to set ready to shoot back to false at the end of a shot.
    System.out.println("Driver Command ShootTrap: Was completed!");
    System.out.println("=======================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
