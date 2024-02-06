// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands.Default;

import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class FrontIntakeDefault extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;

  boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FrontIntakeDefault(FrontIntakeSubsystem frontIntakeSubsystem) {
    m_frontIntakeSubsystem = frontIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("========================");
    System.out.println("FrontIntakeDefault: Started");
    
    m_isFinished = false;

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeStowPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("================================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("FrontIntakeDefault: Finished");
    
    return m_isFinished;
  }
}
