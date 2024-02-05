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
public class ShooterModeShootWithPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  
  private boolean m_isFinished = false;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterModeShootWithPose(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("==========================");
    System.out.println("Command Operator: ShooterModeShootWithPose");
   
    m_shooterSubsystem.setShooterIntakeSpeed(0);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterIdleSpeed);
    m_shooterSubsystem.setShooterMode("ShootWithPose");

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Need to put the code here that constantly evaluates the position and moves the shooter arm.
    //Will execute until cancelled by the shoot command
    //This command will NEVER finish because it will constantly reposition the arm.
    m_isFinished=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Command Operator ShooerModeShootWithPose: Finished");
    System.out.println("==========================");
    return m_isFinished;

  }
}
