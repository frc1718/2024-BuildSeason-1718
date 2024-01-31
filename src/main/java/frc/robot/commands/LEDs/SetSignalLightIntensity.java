// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetSignalLightIntensity extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_LEDSubsystem;
  double intensity;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetSignalLightIntensity(LEDSubsystem subsystem, double input) {
    m_LEDSubsystem = subsystem;
    intensity = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDSubsystem.SetLightIntensity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LEDSubsystem.SetLightIntensity(intensity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
