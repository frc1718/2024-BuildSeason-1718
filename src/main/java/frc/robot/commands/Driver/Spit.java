// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driver;

import frc.robot.Constants;
import frc.robot.commands.General.StowArmAndIntake;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * The spit command ejects a note out of the shooter onto the floor.
 * The shooter arm must be lifted to correctly spit the note onto the floor without interferring with the drivetrain.
 */
public class Spit extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;

  private boolean m_isFinished = false;
  private int m_stateMachine = 1;

  /**
   * Constructs an instance of the spit command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param shooterIntakeSubsystem An instance of the shooter intake subsystem.
   */
  public Spit(FrontIntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem) {
    m_frontIntakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;

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
    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kFrontIntakeStopSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
          if (m_shooterSubsystem.getShooterArmInPosition(Constants.kShooterArmSpitPos)) {
            m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeSpitSpeed);
            m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeSpitSpeed);
            m_shooterSubsystem.setShooterSpeed(Constants.kShooterMaxSpeed);
            System.out.println("Driver Command Spit: Case 2 Complete!");
            m_stateMachine = m_stateMachine + 1;
          }
        break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Stop front intake, stop shooter intake, stop shooter, move front intake to home, move shooter arm to home
    m_frontIntakeSubsystem.setFrontIntakeSpeed(Constants.kFrontIntakeStopSpeed);
    m_shooterIntakeSubsystem.setShooterIntakeSpeed(Constants.kShooterIntakeStopSpeed);
    m_shooterSubsystem.setShooterSpeed(Constants.kShooterIdleSpeed);  
    new StowArmAndIntake(m_frontIntakeSubsystem, m_shooterSubsystem);

    System.out.println("Driver Command: Spit Finished");
    System.out.println("====================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
