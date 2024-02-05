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
public class ShooterModePodium extends Command {
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
  public ShooterModePodium(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("==========================");
    System.out.println("Command Operator: ShooterModePodium");
    m_shooterSubsystem.setShooterIntakeSpeed(0);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmPodiumPos);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterPodiumSpeed);

    m_shooterSubsystem.setShooterMode("ShootPodium");

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Command finishes if shooter gets up to speed
    if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed)) {
      m_readyToShoot = true;
      System.out.println("Command Operator ShooterModePodium: shooter up to speed");
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Command Operator ShooterModePodium: Command Finished");
    System.out.println("==========================");
    return m_isFinished;
  }
}
