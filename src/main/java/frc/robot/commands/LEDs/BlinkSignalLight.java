// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** An example command that uses an example subsystem. */
public class BlinkSignalLight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_LEDSubsystem;
  double intensity;
  Timer timey = new Timer();
  Double Delay;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BlinkSignalLight(LEDSubsystem subsystem, double IntensityInput, double DelayInput) {
    m_LEDSubsystem = subsystem;
    intensity = IntensityInput;
    Delay = DelayInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command: BlinkSignalLight");
    m_LEDSubsystem.SetLightIntensity(0);
    timey.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timey.hasElapsed(Delay)) {
      if (m_LEDSubsystem.GetLightIntensity() < intensity/2) {
        m_LEDSubsystem.SetLightIntensity(intensity);
      } else {
        m_LEDSubsystem.SetLightIntensity(0);
      }
      timey.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LEDSubsystem.SetLightIntensity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
