// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;

/**
 * The shooter mode podium command sets the shooter mode variable of the shooter subsystem to <i>ShootPodium</i>.
 * In preparation, the speed and position of the shooter is set to fire a shot from the podium.
 */
public class AutoShooterModePodium extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 
  private final ShooterSubsystem m_shooterSubsystem;
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;

  private boolean m_isFinished = false;
  private int m_stateMachine = 0;

  /**
   * Constructs an instance of the shooter mode amp command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   */
  public AutoShooterModePodium(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.kPrintOperatorShooterModePodium){
      System.out.println("==========================");
      System.out.println("Command Operator: ShooterModePodium");
    }

    //Initialize State Machine
    m_stateMachine = 1;    

    //Set Initial Positions and Speeds
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterPodiumSpeed);

    //Set Shooter Mode
    m_shooterSubsystem.setShooterMode("ShootPodium");

    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_stateMachine) {     
      case 1:  // Front intake in position
        if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 1 Started");}
        if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
          m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmAutoPodiumPos);
          if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 1 Complete");}
          m_stateMachine = m_stateMachine + 1;
        }        
        break;
      case 2:  // Arm in position
      if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 2 Started");}
        if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmAutoPodiumPos)) {
          if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 2 Complete");}
          m_stateMachine = m_stateMachine + 1;
        }
        break;
      case 3:  // Shooter up to speed
      if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 3 Started");}
        if (m_shooterSubsystem.getShooterUpToSpeed(Constants.kShooterPodiumSpeed)) {
          if (Constants.kPrintOperatorShooterModePodium){System.out.println("Operator Command ShooterModePodium: Case 3 Complete");}
          m_isFinished = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stateMachine = 1;
    if (Constants.kPrintOperatorShooterModePodium){
      System.out.println("Command Operator ShooterModePodium: Command Finished");
      System.out.println("==========================");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
