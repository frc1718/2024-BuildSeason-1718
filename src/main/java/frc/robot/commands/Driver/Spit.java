// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class Spit extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  boolean m_isFinished=false;
  /**
   * Creates a new ExampleCommand.
   * @param shooterSubsystem
   * @param frontIntakeSubsystem The subsystem used by this command.
   */
  
  public Spit(FrontIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_frontIntakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("====================");
    System.out.println("Driver Command: Spit");

    //Stop front intake, move front intake down, stop intake, stop shooter, move shooter arm up high enough to eject
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmSpitPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Make sure everything is in the correct position, then spit
    if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeDownPos) && m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmSpitPos))
    {
      System.out.println("Driver Command: Intake and Arm in Position to SpiT");
      m_frontIntakeSubsystem.setFrontIntakeSpeed(-Constants.kFrontIntakeMaxSpeed);
      m_shooterSubsystem.setShooterIntakeSpeed(-Constants.kFrontIntakeMaxSpeed);
      m_shooterSubsystem.setShooterSpeed(Constants.kShooterMaxSpeed);
    }

    if (!m_shooterSubsystem.getNotePresentIntake() && !m_shooterSubsystem.getNotePresentShooter())
    System.out.println("Command Driver Spit: No note present.");
    m_isFinished = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Stop front intake, stop shooter intake, stop shooter, move front intake to home, move shooter arm to home
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterStopSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeHomePos);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);

    System.out.println("====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    System.out.println("Driver Command: Spit Finished");


    return m_isFinished;
  }
}
