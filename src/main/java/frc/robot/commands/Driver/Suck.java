// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class Suck extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_isFinished = false;

  public Suck(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("=====================");
    System.out.println("Driver Command: Suck");
  
    //Set required positions
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeDownPos);  
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSuckSpeed);
    
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
asdfasfafdsds still need to have state machine here to get the note in the right place
also need to consider automating the sensors

    //Need shooter arm in position before we start front intake sucking.
    if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmHomePos)) {
      m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeSuckSpeed);
    }

    //Remove this line when we have an actual sensor.  This is here just to insure the command ends.
    m_isFinished=true;

    if (m_shooterSubsystem.getNotePresentIntake()) {
      m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
      m_isFinished = true;
    }

    //Code to place the note correctly in the intake is going to be in the shooter subsystem default.
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
  m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
  m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeHomePos); 
  System.out.println("=====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Driver Command: Suck Finished");
    
    return m_isFinished;
  }
}
