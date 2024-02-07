// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.LEDs.BlinkSignalLight;
import frc.robot.commands.LEDs.SetSignalLightIntensity;
import frc.robot.commands.Operator.PreClimb;
import frc.robot.commands.Operator.ShooterModeAmp;
import frc.robot.commands.Operator.ShooterModePodium;
import frc.robot.commands.Operator.ShooterModeShootWithPose;
import frc.robot.commands.Operator.ShooterModeSubwoofer;
import frc.robot.commands.Default.FrontIntakeDefault;
import frc.robot.commands.Driver.Climb;
import frc.robot.commands.Driver.Shoot;
import frc.robot.commands.Driver.Spit;
import frc.robot.commands.Driver.Suck;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //Attempting to create a selector object.
  File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
  File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/paths");
  Selector chirpSelect = new Selector(chirpFolder, ".chrp");
  Selector autonSelect = new Selector(autonFolder, ".path");

  // Set driver controller up
  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort); // My driveController
  
  // Set operator controller up
  private final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

   
  //Currently disabled to prevent missing motor error messages
  /* Setting up bindings for necessary control of the swerve drive platform */
  //private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  //Currently disabled to prevent missing motor error messages
  //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
 
  //Drivertrain not currently active
  // private final Telemetry logger = new Telemetry(MaxSpeed);

  //Open the Shooter Subsystem
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  //Open the Intake Subsystem
  private final FrontIntakeSubsystem frontIntake = new FrontIntakeSubsystem();

  //Open the Subsystem
  private final ClimberSubsystem climber = new ClimberSubsystem();

  //Open the LED Subsystem
  private final LEDSubsystem LED = new LEDSubsystem();

  // Sets Default Command
  // Assign default commands
  // ShooterSubsystem.setDefaultCommand(new TankDrive(() -> -m_joystick.getLeftY(), () -> -m_joystick.getRightY(), m_drivetrain));

  private void configureBindings() {
    //Schedules drivertain
    //drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //    drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
    //        .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //        .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //    ));

    //=============================================================================
    //======================Driver Controller Assignments==========================
    //=============================================================================

    // Schedules Tilt modules without driving wheels?  Maybe?
    //driveController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));
    
    // Currently disabled to prevent motor missing errors
    // Schedules Brake Swerve Drivetrain Binds (x-lock wheels) Driver
    //driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    
    driveController.leftBumper().onTrue(new Shoot(frontIntake, shooter, climber));
    driveController.rightBumper().whileTrue(new Suck(frontIntake, shooter));
    driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter)); 
    driveController.leftTrigger(.5).onTrue(new Climb(climber,frontIntake,shooter));
 
     
    // Schedules reset the field - Binds centric heading on back and start button push
    //driveController.back().and(driveController.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    //First try at the front intake stowing when the arm is at home
    //shooter.shooterArmInHomePositionTrigger().onTrue(new FrontIntakeDefault(frontIntake) );

    //LED Stuff blinks randomly
    //Trigger PickupStatus = new Trigger(shooter::getNotePresentIntake);
    //PickupStatus.onTrue(new BlinkSignalLight(LED, 1, 0.5));
    //PickupStatus.onFalse(new SetSignalLightIntensity(LED, 0));

    //Trigger NoteLocationStatus = new Trigger(shooter::getNotePresentShooter);
    //NoteLocationStatus.onTrue(new SetSignalLightIntensity(LED, 0.75));
    //NoteLocationStatus.onFalse(new SetSignalLightIntensity(LED, 0));

    //======================================================================
    //=========================Operator Controller Assignments==============
    //======================================================================
    // Y - Podium
    // B - Amp
    // X - Shoot From Pose
    // A - Subwoofer
    // Right Top + Left Top Bumper Climb Mode - Hold Down Both at Once
    //
    operatorController.y().onTrue(new ShooterModePodium(frontIntake, shooter));
    operatorController.b().onTrue(new ShooterModeAmp(frontIntake, shooter));
    operatorController.x().onTrue(new ShooterModeShootWithPose(frontIntake, shooter));
    operatorController.a().onTrue(new ShooterModeSubwoofer(frontIntake, shooter));
    operatorController.leftBumper().and(operatorController.rightBumper()).debounce(2).onTrue(new PreClimb(climber,shooter,frontIntake));

    // Schedules Play music - Binds Dpad Up
    //operatorController.povUp().onTrue();

    /* Setting up bindings for selecting an autonomous to run. */
    /* Up / Down on the D-Pad of the driver controller. */
    /* Until we start generating paths and creating auton routines, this will cycle through .chrp files.*/
    driveController.povDown().onTrue(new InstantCommand(() -> {
      if (RobotState.isDisabled()) {
        chirpSelect.decrementSelection();
      }
    }).ignoringDisable(true));


    driveController.povUp().onTrue(new InstantCommand(() -> {
      if (RobotState.isDisabled()) {
        chirpSelect.incrementSelection();
      }
    }).ignoringDisable(true));

    //if (Utils.isSimulation()) {
    //  drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    //}
    //drivetrain.registerTelemetry(logger::telemeterize);



  }


  private void configureCustomNTValues(){
    //All of the NT publishing we would like to do, that won't be setup in the classes themselves, gets setup here.
    SmartDashboard.putData("Chirp Selector", chirpSelect);
    SmartDashboard.putData("Auton Selector", autonSelect);    
    SmartDashboard.putData("BeamBreak", shooter);
  }


  public RobotContainer() {
    configureBindings();
    configureCustomNTValues();

    //===================Default Commands==============================
    //shooter.setDefaultCommand(new FrontIntakeDefault(frontIntake, shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
