// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import frc.robot.commands.Driver.Shoot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The light LED on note present command does exactly what you think it does.
 * If either beam break detects a note, the LEDs will turn off.
 */
public class LightLEDOnNotePresent extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_LEDSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  Debouncer m_debouncer= new Debouncer(.1, Debouncer.DebounceType.kBoth);

  boolean m_isFinished = false;

  /**
   * Constructs an instance of the light LED on note present command.
   * @param LEDSubsystem An instance of the LED subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   */
  public LightLEDOnNotePresent(LEDSubsystem subsystem, ShooterIntakeSubsystem shooterIntakeSubsystem) {
    m_LEDSubsystem = subsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished=false;
    System.out.println("Command LightLEDOnNotePresent: Started");
    m_LEDSubsystem.SetLightIntensity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //System.out.println("Command LightLEDOnNotePresent: Running");
      m_LEDSubsystem.SetLightIntensity(1);

      if (!m_shooterIntakeSubsystem.getNotePresent()){
        m_LEDSubsystem.SetLightIntensity(0);
        m_isFinished=true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
