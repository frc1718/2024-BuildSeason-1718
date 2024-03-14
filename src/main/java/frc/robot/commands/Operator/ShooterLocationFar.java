// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The shooter mode amp command sets the shooter mode variable of the shooter subsystem to <i>ShootAmp</i>.
 * In preparation, the speed and position of the shooter is set to fire a shot at the amp.
 */
public class ShooterLocationFar extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climberSubsystem;
  
  private boolean m_isFinished = false;

  /**
   * Constructs an instance of the shooter mode amp command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   */
  public ShooterLocationFar(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.setClimbLocation("FarClimb");
    m_isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
