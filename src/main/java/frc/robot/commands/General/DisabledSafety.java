// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The note position command gets the note into the correct position in the shooter: fully in contact with the intake beam break, but barely in contact with the shooter beam break.
 */
public class DisabledSafety extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimberSubsystem  m_climber;

  private boolean m_isFinished = false;

  /**
   * Constructs an instance of the note position command.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public DisabledSafety(ShooterIntakeSubsystem shooterIntakeSubsystem, FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem ) {

    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_climber = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterIntakeSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========================");
    System.out.println("General DisabledSafety started");
  
    m_isFinished = false;
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);

    m_shooterSubsystem.setShooterArmPosition(m_shooterSubsystem.getShooterArmPosition());
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterStopSpeed);

    m_frontIntakeSubsystem.setFrontIntakePosition(m_frontIntakeSubsystem.getFrontIntakePosition());
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);

    m_isFinished=true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("General DisabledSafety Completed");
    System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return m_isFinished;
  }
}
