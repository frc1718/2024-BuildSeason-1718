// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.LEDs.BlinkSignalLight;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.commands.LEDs.SetSignalLightIntensity;
import frc.robot.commands.PreStageShooter.ShooterModeAmp;
import frc.robot.commands.PreStageShooter.ShooterModePodium;
import frc.robot.commands.PreStageShooter.ShooterModeShootWithPose;
import frc.robot.commands.PreStageShooter.ShooterModeSubwoofer;
import frc.robot.commands.ScoreNotes.ScoreAmp;
import frc.robot.commands.ScoreNotes.ShootFromPodium;
import frc.robot.commands.ScoreNotes.ShootFromSubwoofer;
import frc.robot.commands.ScoreNotes.ShootOnMoveWithPose;
import frc.robot.commands.Climb.Climb;
import frc.robot.commands.Climb.PreClimb;
import frc.robot.commands.FrontIntake.Spit;
import frc.robot.commands.FrontIntake.Suck;

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
  File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
  Selector chirpSelect = new Selector(chirpFolder, ".chrp");
  Selector autonSelect = new Selector(autonFolder, ".auto");

  // Set driver controller up
  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort); // My driveController
  
  // Set operator controller up
  private final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

   
  //Currently disabled to prevent missing motor error messages
  /* Setting up bindings for necessary control of the swerve drive platform */
  //The drivetrain is public so the limelight pose can be added to it in Robot.java.
  //Could perhaps move that code into the drivetrain default command lambda, or into the periodic() method in CommandSwerveDrivetrain.
  //public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  //Currently disabled to prevent missing motor error messages
  //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  //Testing an idea: hold a button and always aim at the goal.
  //Copying a fair amount of this from the FieldCentric drive.
  //private final SwerveRequest.FieldCentricFacingAngle rootyTootyPointAndShooty = new SwerveRequest.FieldCentricFacingAngle()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //private final Telemetry logger = new Telemetry(MaxSpeed);

  //Open the Shooter Subsystem
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  //Open the Intake Subsystem
  private final FrontIntakeSubsystem frontIntake = new FrontIntakeSubsystem();

  //Open the Climber Subsystem
  private final ClimberSubsystem climber = new ClimberSubsystem();

  //Open the LED Subsystem
  private final LEDSubsystem LED = new LEDSubsystem();

  private void configureBindings() {
    //Schedules drivertain
    //drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        //drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with
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
    
    // Making the shoot button situation specific - shoot will always shoot in any more

    //If climb is not enabled, these commands are valid
    if (!climber.preClimbActuated(false)) {
      if (shooter.getShooterMode()=="ScoreAmp") {
        driveController.leftBumper().whileTrue(new ScoreAmp(frontIntake, shooter));
      } else if (shooter.getShooterMode()=="ScoreSubwoofer") {  
        driveController.leftBumper().whileTrue(new ShootFromSubwoofer(frontIntake, shooter));
      } else if (shooter.getShooterMode()=="ScorePodium") {
        driveController.leftBumper().whileTrue(new ShootFromPodium(frontIntake, shooter));
      }
      driveController.rightBumper().whileTrue(new Suck(frontIntake, shooter));
      driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter));
    }


    //If climb is enabled, these commands are valid
    if (climber.preClimbActuated(false)) {
      driveController.leftTrigger(.5).whileTrue(new Climb(climber,frontIntake,shooter));
      // Schedules Descend - Binds ********
      // At this time, we aren't going to descend because we don't have time.
    }
     
    // Schedules reset the field - Binds centric heading on back and start button push
    //driveController.back().and(driveController.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    //LED Stuff
    Trigger PickupStatus = new Trigger(shooter::getNotePresentIntake);
    PickupStatus.onTrue(new BlinkSignalLight(LED, 1, 0.5));
    PickupStatus.onFalse(new SetSignalLightIntensity(LED, 0));

    Trigger NoteLocationStatus = new Trigger(shooter::getNotePresentShooter);
    NoteLocationStatus.onTrue(new SetSignalLightIntensity(LED, 0.75));
    NoteLocationStatus.onFalse(new SetSignalLightIntensity(LED, 0));

    //======================================================================
    //=========================Operator Controller Assignments==============
    //======================================================================
    // Y - Podium
    // B - Amp
    // X - Shoot From Pose
    // A - Subwoofer
    // Right Top + Left Top Bumper Climb Mode - Hold Down Both at Once
    //
    operatorController.y().onTrue(new ShooterModePodium(frontIntake,shooter));
    operatorController.b().onTrue(new ShooterModeAmp(frontIntake,shooter));
    operatorController.x().onTrue(new ShooterModeShootWithPose(frontIntake,shooter));
    operatorController.a().onTrue(new ShooterModeSubwoofer(frontIntake,shooter));
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

    driveController.a().onTrue(new InstantCommand(() -> {
      //I'm not sure if 'tableName' refers to limelight name, or a network table.
      //Need to implement a way to automatically name the files with unique names.
      LimelightHelpers.takeSnapshot("limelight", "snapshot");
    }));


    //if (Utils.isSimulation()) {
      //drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    //}
    //drivetrain.registerTelemetry(logger::telemeterize);
  }


  private void configureCustomNTValues(){
    //All of the NT publishing we would like to do, that won't be setup in the classes themselves, gets setup here.
    SmartDashboard.putData("Chirp Selector", chirpSelect);
    SmartDashboard.putData("Auton Selector", autonSelect);    
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(climber);
    SmartDashboard.putData(frontIntake);
    SmartDashboard.putData(LED);
  }

  private void registerAutonCommands(){
    //ALL COMMANDS THAT COULD BE USED IN AUTONOMOUS NEED TO BE REGISTERED HERE.
    //These are currently added as an example.
    //PrintCommand is basic.
    //If I understand the commands correctly, Auton Light will end almost immediately.
    //But Auton Blink should never end.
    NamedCommands.registerCommand("Print YAY", new PrintCommand("YAY"));
    NamedCommands.registerCommand("Auton Light",new SetSignalLightIntensity(LED, 1.00));
    NamedCommands.registerCommand("Auton Blink", new BlinkSignalLight(LED, 1.00, 0.5));
  }

  public RobotContainer() {
    configureBindings();
    //Not sure if this is the correct placement for registering autonomous commands.
    registerAutonCommands();
    configureCustomNTValues();
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("Selected Autonomous: " + chirpSelect.getCurrentSelectionName()); //Using the CHRP list for debugging.
    //This should load the selected autonomous file.
    //return drivetrain.getAutoPath(autonSelect.getCurrentSelectionName());
  }
}
