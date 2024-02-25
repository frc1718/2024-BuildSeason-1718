// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;

/**
 * The shooter mode subwoofer command sets the shooter mode variable of the shooter subsystem to <i>ShootSubwoofer</i>.
 * In preparation, the speed and position of the shooter is set to fire a shot from right against the subwoofer.
 */
public class ShooterModeSubwoofer extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ShooterSubsystem m_shooterSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;

  /**
   * Constructs an instance of the shooter mode subwoofer command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   */
  public ShooterModeSubwoofer(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("==========================");
    System.out.println("Command Operator: ShooterModeSubwoofer");

    //Initialize state machine
    m_stateMachine = 1;

    //Set positions and speeds
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterSubwooferSpeed);
    m_shooterSubsystem.setShooterMode("ShootSubwoofer");

    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine) {     
      case 1:  // Front intake in position
        System.out.println("Operator Command ShooterModeSubwoofer: Case 1 Started");
        if (m_frontIntakeSubsystem.getFrontIntakeIsClear()) {
          m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmSubwooferPos);
          System.out.println("Operator Command ShooterModeSubwoofer: Case 1 Complete");
          m_stateMachine = m_stateMachine + 1;
        }        
        break;
      case 2:  // Arm in position
        System.out.println("Operator Command ShooterModeSubwoofer: Case 2 Started");
        if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmSubwooferPos)) {
          System.out.println("Operator Command ShooterModeSubwoofer: Case 2 Complete");
          m_stateMachine = m_stateMachine + 1;
        }
        break;
      case 3:  // Shooter up to speed
        System.out.println("Operator Command ShooterModeSubwoofer: Case 3 Started");       
        if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterSubwooferSpeed)) {
          System.out.println("Operator Command ShooterModeSubwoofer: Case 3 Complete");
          m_isFinished = true;
        }
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator ShooterModeSubwoofer: Command Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_isFinished;
  }
}
