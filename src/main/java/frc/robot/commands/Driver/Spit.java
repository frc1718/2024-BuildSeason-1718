// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.FrontIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class Spit extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private boolean m_isFinished = false;
  private int m_stateMachine = 1;
  
  public Spit(FrontIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_frontIntakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("====================");
    System.out.println("Driver Command: Spit");

    //Initialize State Machine
    m_stateMachine = 1;

    //Stop front intake, move front intake down, stop intake, stop shooter, move shooter arm up high enough to eject
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeClearPos);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kFrontIntakeStopSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

asdfafdasf working here while I go through and make sure initialize sends things to positions

      //State Machine
      switch(m_stateMachine){     
        case 1:  //Front intake in position
          System.out.println("Driver Command Spit: Case 1");
          if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
            System.out.println("Driver Command Spit: Case 1 Complete!");
            m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmSpitPos);
            m_stateMachine = m_stateMachine + 1;
          }        
        break;
        case 2:  // Shooter Arm In Position
          System.out.println("Driver Command Spit: Case 2");
          if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmAmpPos))) {
            m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSpitSpeed);
            m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeSpitSpeed);
            m_shooterSubsystem.setShooterSpeed(Constants.kShooterMaxSpeed);
            System.out.println("Driver Command Spit: Case 2 Complete!");
            m_stateMachine = m_stateMachine + 1;
          }
        break;

        afds;lkjasfd;lkjfdsa;lkjafds;lkj
      }
    //Check that front intake is out of the way before moving arm
    if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos)) {
      System.out.println("Driver Command: Front Intake is in position to spit");
      m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmSpitPos);
    }

    //Make sure everything is in the correct position, then spit
    if (m_frontIntakeSubsystem.getFrontIntakeInPosition(Constants.kFrontIntakeClearPos) && m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmSpitPos))
    {
      System.out.println("Driver Command: Front Intake and Arm in Position to Spit");
      m_frontIntakeSubsystem.setFrontIntakeSpeed(-Constants.kFrontIntakeMaxSpeed);
      m_shooterSubsystem.setShooterIntakeSpeed(-Constants.kShooterIntakeMaxSpeed);
      m_shooterSubsystem.setShooterSpeed(Constants.kShooterMaxSpeed);
    }



    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Stop front intake, stop shooter intake, stop shooter, move front intake to home, move shooter arm to home
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_shooterSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterStopSpeed);
    m_frontIntakeSubsystem.setFrontIntakePosition(Constants.kFrontIntakeHomePos);
    m_shooterSubsystem.setShooterArmPosition(Constants.kShooterArmHomePos);

    System.out.println("====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    System.out.println("Driver Command: Spit Finished");


    return m_isFinished;
  }
}
