// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterModeSubwoofer extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ShooterSubsystem m_shooterSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  
  private boolean m_isFinished = false;
  private boolean m_readyToShoot = false;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterModeSubwoofer(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("==========================");
    System.out.println("Command Operator: ShooterModeSubwoofer");
    m_shooterSubsystem.setShooterIntakeSpeed(0);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmSubwooferPos);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterSubwooferSpeed);

    m_shooterSubsystem.setShooterMode("ShootSubwoofer");

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed)) {
      m_readyToShoot = true;
      m_isFinished = true;
      System.out.println("Command Operator ShooterModeSubwoofer: Ready to shoot");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Command Operator ShooterModeSubwoofer: Command Finished");
    System.out.println("==========================");
    return m_isFinished;
  }
}
