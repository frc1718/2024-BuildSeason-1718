// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterModeShootWithPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;
  private int m_shooterPoseSpeed = 0;
  private int m_shooterArmPosePos = 0;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param shooterSubsystem The subsystem used by this command.
   */
  public ShooterModeShootWithPose(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
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
    System.out.println("Command Operator: ShooterModeShootWithPose");
   
    //Initialize State Machine
    m_stateMachine = 1;

    //Set initial speeds and positions
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);
    m_shooterSubsystem.setShooterMode("ShootWithPose");

    m_isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //We should be calculating the pose requirements here over and over.
    m_shooterPoseSpeed = 1;
    m_shooterArmPosePos = 1;

    switch(m_stateMachine) {     
      case 1:  // Front intake in position
        System.out.println("Operator Command ShootWithPose: Case 1 Started");
        if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
          System.out.println("Operator Command ShootWithPose: Case 1 Complete");
          m_stateMachine = m_stateMachine + 1;
        }        
        break;
      case 2:  // Arm in position
        System.out.println("Operator Command ShootWithPose: Case 2 Started");
          //This is a unique case, we are going to stay here until interrupted.  Most like by shoot command.
          m_shooterSubsystem.setShooterArmPosition(m_shooterArmPosePos);
          m_shooterSubsystem.setShooterSpeed(m_shooterPoseSpeed);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator ShooerModeShootWithPose: Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
