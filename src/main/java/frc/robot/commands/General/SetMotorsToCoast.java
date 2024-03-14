// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The set motors to coast command sets the climber, shooter arm, and front intake motors to coast while the command is running.
 * Useful for manually moving the components in the pits and in queue.
 */
public class SetMotorsToCoast extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  Debouncer m_debouncer= new Debouncer(.1, Debouncer.DebounceType.kBoth);

  boolean m_isFinished = false;

  /**
   * Constructs an instance of the set motors to coast command.
   * @param climberSubsystem An instance of the climber subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   * Required.
   */
  public SetMotorsToCoast(ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem, FrontIntakeSubsystem frontIntakeSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem) {
    m_climberSubsystem = climberSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    addRequirements(m_shooterIntakeSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished=false;
    m_climberSubsystem.SetLeftClimberNeutralMode(NeutralModeValue.Coast);
    m_climberSubsystem.SetRightClimberNeutralMode(NeutralModeValue.Coast);
    m_shooterSubsystem.SetShooterArmLeftNeutralMode(NeutralModeValue.Coast);
    m_shooterSubsystem.SetShooterArmRightNeutralMode(NeutralModeValue.Coast);
    m_frontIntakeSubsystem.SetFrontIntakeRotateNeutralMode(NeutralModeValue.Coast);
    m_shooterIntakeSubsystem.SetShooterIntakeRotateNeutralMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.SetLeftClimberNeutralMode(NeutralModeValue.Brake);
    m_climberSubsystem.SetRightClimberNeutralMode(NeutralModeValue.Brake);
    m_shooterSubsystem.SetShooterArmLeftNeutralMode(NeutralModeValue.Brake);
    m_shooterSubsystem.SetShooterArmRightNeutralMode(NeutralModeValue.Brake);
    m_frontIntakeSubsystem.SetFrontIntakeRotateNeutralMode(NeutralModeValue.Brake);
    m_shooterIntakeSubsystem.SetShooterIntakeRotateNeutralMode(NeutralModeValue.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
