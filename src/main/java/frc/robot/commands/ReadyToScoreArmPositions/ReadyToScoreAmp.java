// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReadyToScoreArmPositions;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ReadyToScoreAmp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ReadyToScoreAmp(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmPreAmpPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.getNotePresentShooter()) {
      m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeReverseIndexSpeed);
    } else {
      m_shooterSubsystem.setShooterSpeed(Constants.kShooterAmpSpeed);
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
