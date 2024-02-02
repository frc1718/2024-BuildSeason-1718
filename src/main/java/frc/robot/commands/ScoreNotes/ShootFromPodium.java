// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoreNotes;

import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootFromPodium extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_isFinished = false;
  private boolean m_readyToShoot = false;

  /**
   * Creates a new ExampleCommand.
   * @param intakeSubsystem
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShootFromPodium(FrontIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    addRequirements(m_shooterSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setFrontIntakeSpeed(0);
    m_shooterSubsystem.setShooterIntakeSpeed(0);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterPodiumSpeed);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmPodiumPos);

    m_shooterSubsystem.setShooterMode("ShootFromPodium");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed) && m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmPodiumPos)) {
      m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeShootSpeed);
      m_readyToShoot = true;
    }
    if (!m_shooterSubsystem.getNotePresentShooter() && m_readyToShoot) {
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setFrontIntakeSpeed(0);
    m_shooterSubsystem.setShooterIntakeSpeed(0);
    m_shooterSubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
