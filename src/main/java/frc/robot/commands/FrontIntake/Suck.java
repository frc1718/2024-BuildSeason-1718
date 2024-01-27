// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Suck extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   * @param intakeSubsystem
   * @param shooterSubsystem The subsystem used by this command.
   */
  public Suck(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    addRequirements(m_shooterSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.runFrontIntake(0);
    m_intakeSubsystem.intakeToPosition(0);
    m_shooterSubsystem.runShooterIntake(0);
    m_shooterSubsystem.shooterArmToPosition(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.runFrontIntake(0);
    m_shooterSubsystem.runShooterIntake(0);
    m_shooterSubsystem.runShooterIntake(0);
    m_shooterSubsystem.shooterArmToPosition(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
