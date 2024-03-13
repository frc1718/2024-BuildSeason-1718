// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //This code is for a manual control of the climber if we want it
  //in case of encoders messing up and we don't want the climber
  //going to positions it physically can't.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.Driver.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The pre-climb command moves all of the related subsystems into the 'Pre-Climb' state.
 * The front intake roller and shooter motors are stopped and both the front intake and shooter arm are moved into position.
 * The climber is moved into position so the hooks can hit the chain.
 */
public class DriveWhileFacingAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final CommandXboxController m_Controller;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private final Rotation2d m_RotationTarget = new Rotation2d(0.0);

  private boolean m_isFinished = false;

  public DriveWhileFacingAngle(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, ShooterSubsystem shooter) {
    m_Drivetrain = drivetrain;
    m_Controller = controller;
    m_ShooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("==========================");
    System.out.println("Command Operator: Home");


    //In the initialize step, set the desired starting positions and speeds of each system
    
    m_isFinished = false;

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.applyRequest(() -> driveFacingAngle.withVelocityX(-m_Controller.getLeftY() * MaxSpeed).withVelocityY(-m_Controller.getLeftY() * MaxSpeed).withTargetDirection(m_RotationTarget));
    if (m_ShooterSubsystem.getShooterMode() == "DoNothing") {
      m_isFinished = true;
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator Home: Command Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
