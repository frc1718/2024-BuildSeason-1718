// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Climb(ClimberSubsystem climberSubsystem, FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_climberSubsystem = climberSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_climberSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========================");
    System.out.println("Driver Command: Climb");
    
    m_isFinished = false;

    //Tell the climber to climb
    if(m_climberSubsystem.getPreClimbActuated()){
      System.out.println("Driver Command Climb: Preclimb was actuated!");
      m_climberSubsystem.setClimberDesiredPosition(Constants.kClimberClimbPos);
    } else {
      System.out.println("Driver Command Climb: Preclimb wasn't actuated yet!");
      m_isFinished=true;
    }
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Remove once climber is in place.  Here so the command finishes for debuggin.
    m_isFinished=true;

    //Need to set a flag to see if we've reached climb and check it
    if (m_climberSubsystem.getClimberInPosition(Constants.kClimberClimbPos)){
      System.out.println("Driver Command: Climb Reached Climb Height");
      m_isFinished= true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Driver Command: Climb completed!");
    
    return m_isFinished;
  }
}
