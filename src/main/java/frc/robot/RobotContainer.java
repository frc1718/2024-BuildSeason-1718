// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.LEDs.LightLEDOnNotePresent;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.commands.Operator.PreClimb;
import frc.robot.commands.Operator.ShooterModeAmp;
import frc.robot.commands.Operator.ShooterModePodium;
import frc.robot.commands.Operator.ShooterModeShootWithPose;
import frc.robot.commands.Operator.ShooterModeSubwoofer;
import frc.robot.commands.Driver.Climb;
import frc.robot.commands.Driver.Shoot;
import frc.robot.commands.Driver.ShootTrap;
import frc.robot.commands.Driver.Spit;
import frc.robot.commands.Driver.Suck;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.CommandSwerveDrivetrain;

//Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

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
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  //Currently disabled to prevent missing motor error messages
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  //Testing an idea: hold a button and always aim at the goal.
  //Copying a fair amount of this from the FieldCentric drive.
  private final SwerveRequest.FieldCentricFacingAngle rootyTootyPointAndShooty = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  //Open Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final FrontIntakeSubsystem frontIntake = new FrontIntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final LEDSubsystem LED = new LEDSubsystem();
  private final ShooterIntakeSubsystem shooterIntakeSubsystem = new ShooterIntakeSubsystem();

  private void configureBindings() {
    //Schedules drivertain
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    //=============================================================================
    //======================Driver Controller Assignments==========================
    //=============================================================================

    // Schedules Tilt modules without driving wheels?  Maybe?
    driveController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));
    
    // Currently disabled to prevent motor missing errors
    // Schedules Brake Swerve Drivetrain Binds (x-lock wheels) Driver
    driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    
    //I bet there's a much better place to put this assignment, but I don't know where.
    //These gains are also completely made up.  Not terrible in simulation, but don't trust them.
    rootyTootyPointAndShooty.HeadingController.setPID(20, 0, 0.05);

    //Before this can be used, theres an issue with being 'above' or 'below' the coordinates of the speaker.
    //I think it's because of the -180 to +180 crossover, but unsure how to fix it currently.
    driveController.start().whileTrue(drivetrain.applyRequest(() -> rootyTootyPointAndShooty.withVelocityX(-driveController.getLeftY() * MaxSpeed)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed)
            .withTargetDirection(Constants.kBlueSpeakerLocation.minus(drivetrain.getState().Pose.getTranslation()).getAngle())
            ));

    driveController.leftBumper().onTrue(new Shoot(frontIntake, shooter, climber, shooterIntakeSubsystem));
    driveController.rightBumper().whileTrue(new Suck(frontIntake, shooter, shooterIntakeSubsystem));
    driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter, shooterIntakeSubsystem)); 
    driveController.leftTrigger(.5).onTrue(new Climb(climber,frontIntake,shooter));
    driveController.y().onTrue(new ShootTrap(frontIntake, shooter, climber, shooterIntakeSubsystem));
 
     
    // Schedules reset the field - Binds centric heading on back and start button push
    driveController.back().and(driveController.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //LED Stuff blinks randomly
    //Trigger PickupStatus = new Trigger(shooterIntakeSubsystem::getNotePresentIntake);
    //PickupStatus.onTrue(new BlinkSignalLight(LED, 1, 0.5));
    //PickupStatus.onFalse(new SetSignalLightIntensity(LED, 0));

    Trigger NoteLocationStatus = new Trigger(shooterIntakeSubsystem::getNotePresent);
    NoteLocationStatus.onTrue(new LightLEDOnNotePresent(LED, shooterIntakeSubsystem));
    
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
    //operatorController.x().onTrue(new ShooterModeShootWithPose(frontIntake, shooter, drivetrain));  Since this command relies on the drivetrain, it is commented out for now.
    operatorController.a().onTrue(new ShooterModeSubwoofer(frontIntake, shooter));
    operatorController.leftBumper().and(operatorController.rightBumper()).debounce(2).onTrue(new PreClimb(climber,shooter,frontIntake, shooterIntakeSubsystem));

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

    driveController.povLeft().onTrue(new InstantCommand(() -> {
      frontIntake.setServoPosition(0.5);;
    }));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);



  }

  /**
   * Add all of the subsystems to {@link SmartDashboard}, so the data is published automatically.
   */
  private void configureCustomNTValues(){
    //All of the NT publishing we would like to do, that won't be setup in the classes themselves, gets setup here.
    SmartDashboard.putData("Chirp Selector", chirpSelect);
    SmartDashboard.putData("Auton Selector", autonSelect);    
    //SmartDashboard.putData(shooter);
    //SmartDashboard.putData(climber);
    //SmartDashboard.putData(frontIntake);
    //SmartDashboard.putData(LED);
  }

  /**
   * Register all of the commands that can be used in autonomous.
   * If a command is not registered here, referencing it in a PathPlanner autonomous won't do anything.
   */
  private void registerAutonCommands(){
    //ALL COMMANDS THAT COULD BE USED IN AUTONOMOUS NEED TO BE REGISTERED HERE.
    //These are currently added as an example.
    //PrintCommand is basic.
    //If I understand the commands correctly, Auton Light will end almost immediately.
    //But Auton Blink should never end.
    NamedCommands.registerCommand("Print YAY", new PrintCommand("YAY"));
    NamedCommands.registerCommand("Auton Light",new LightLEDOnNotePresent(LED, shooterIntakeSubsystem));
    NamedCommands.registerCommand("Auton Blink", new PrintCommand("Auton Blink no longer exists."));
  }

  public RobotContainer() {

    //Start the CTRE logger for sysID use
    // SignalLogger.start();

    configureBindings();
    //Not sure if this is the correct placement for registering autonomous commands.
    registerAutonCommands();
    configureCustomNTValues();

    //===================Default Commands==============================
    //shooter.setDefaultCommand(new FrontIntakeDefault(frontIntake, shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("Selected Autonomous: " + chirpSelect.getCurrentSelectionName()); //Using the CHRP list for debugging.
    //This should load the selected autonomous file.
    //return drivetrain.getAutoPath(autonSelect.getCurrentSelectionName());
  }
}
