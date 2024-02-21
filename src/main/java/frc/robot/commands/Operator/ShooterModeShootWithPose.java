// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import frc.robot.Constants;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FrontIntakeSubsystem;

/**
* The shooter mode shoot with pose command sets the shooter mode variable of the shooter subsystem to <i>ShootWithPose</i>.
* In preparation, the speed and position of the shooter is set to fire a shot from the current position on the field.
*/
public class ShooterModeShootWithPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final FrontIntakeSubsystem m_frontIntakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final CommandSwerveDrivetrain m_drivetrain;

  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  
  private boolean m_isFinished = false;
  private int m_stateMachine = 1;
  private double m_shooterPoseSpeed = 0;
  private double m_shooterArmPosePos = 0;
  private double m_distanceToSubwoofer = 0;

  /**
   * Constructs an instance of the shooter mode shoot with pose command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param drivetrain An instance of the drivetrain subsystem.
   * Required.
   */
  public ShooterModeShootWithPose(FrontIntakeSubsystem frontIntakeSubsystem, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain) {
    m_shooterSubsystem = shooterSubsystem;
    m_frontIntakeSubsystem = frontIntakeSubsystem;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_frontIntakeSubsystem);
    addRequirements(m_drivetrain);
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

    //Atempt to lock the drivetrain into position.
    m_drivetrain.setControl(m_brake);

    m_isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //We should be calculating the pose requirements here over and over.
    m_distanceToSubwoofer = Constants.kBlueSpeakerLocation.getDistance(m_drivetrain.getState().Pose.getTranslation());
    m_shooterPoseSpeed = Constants.kShooterSpeedTable.get(m_distanceToSubwoofer);
    m_shooterArmPosePos = Constants.kShooterArmTable.get(m_distanceToSubwoofer);

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
          //This is a unique case, we are going to stay here until interrupted.  Most likely by shoot command.
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
