// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LEDs.LightLEDOnNotePresent;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.commands.DriveWhileFacingAngle;
import frc.robot.commands.Operator.Home;
import frc.robot.commands.Operator.PreClimb;
import frc.robot.commands.Operator.ShooterModeAmp;
import frc.robot.commands.Operator.ShooterModeMiddleAuto;
import frc.robot.commands.Operator.ShooterModePass;
//import frc.robot.commands.Operator.ShooterModePas
//import frc.robot.commands.Operator.ShooterModePass;
import frc.robot.commands.Operator.ShooterModePodium;
import frc.robot.commands.Operator.ShooterModeRightAuto;
import frc.robot.commands.Operator.ShooterModeShootWithPose;
import frc.robot.commands.Operator.ShooterModeSubwoofer;
import frc.robot.commands.Driver.Climb;
import frc.robot.commands.Driver.Shoot;
import frc.robot.commands.Driver.ShootTrap;
import frc.robot.commands.Driver.Spit;
import frc.robot.commands.Driver.Suck;
import frc.robot.commands.Driver.SuckNoFront;
import frc.robot.commands.General.SetMotorsToCoast;
import frc.robot.commands.General.NotePosition;
import frc.robot.commands.General.StowArmAndIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.CommandSwerveDrivetrain;

//Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //Attempting to create a selector object.
  File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
  File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
  Selector chirpSelect = new Selector(chirpFolder, ".chrp");
  Selector autonSelect = new Selector(autonFolder, ".auto");
  double driveSign = 1;

  public Rotation2d resetRotationBlue = new Rotation2d(0);
  public Rotation2d resetRotationRed = new Rotation2d (Math.PI);
  public Pose2d resetPoseBlue = new Pose2d(0,0,resetRotationBlue);
  public Pose2d resetPoseRed = new Pose2d(0,0,resetRotationRed);
  public Pose2d resetPose = new Pose2d();

  public Command builtAutonomousCommand;

  // Set driver controller up
  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort); // My driveController
  
  // Set operator controller up
  private final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  /* Setting up bindings for necessary control of the swerve drive platform */
  //The drivetrain is public so the limelight pose can be added to it in Robot.java.
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
                                                               
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  //Testing an idea: hold a button and always aim at the goal.
  //Copying a fair amount of this from the FieldCentric drive.
  private final SwerveRequest.FieldCentricFacingAngle rootyTootyPointAndShooty = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity); 
  
  //An example of the robot centric swerve request.
  //Useful for helping the driver line up with the chain.
  private final SwerveRequest.RobotCentric climbAlignment = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  //Open Subsystems
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final FrontIntakeSubsystem frontIntake = new FrontIntakeSubsystem();
  public final ClimberSubsystem climber = new ClimberSubsystem();
  public final LEDSubsystem LED = new LEDSubsystem();
  public final ShooterIntakeSubsystem shooterIntake = new ShooterIntakeSubsystem();
  public final BeamBreakSubsystem beamBreak = new BeamBreakSubsystem();

  public final Orchestra music = new Orchestra();

  private void configureBindings() {
    //Schedules drivertain
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * driveSign) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed * driveSign) // Drive left with negative X (left)
            .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
    
    //=============================================================================
    //======================Driver Controller Assignments==========================
    //=============================================================================

    // Schedules Tilt modules without driving wheels?  Maybe?
    //driveController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));
    
    // Schedules Brake Swerve Drivetrain Binds (x-lock wheels) Driver
    driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    
    //I bet there's a much better place to put this assignment, but I don't know where.
    //These gains are also completely made up.  Not terrible in simulation, but don't trust them.
    rootyTootyPointAndShooty.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    rootyTootyPointAndShooty.HeadingController.setPID(20, 0, 0.05);

    driveController.start().whileTrue(drivetrain.applyRequest(() -> rootyTootyPointAndShooty.withVelocityX(-driveController.getLeftY() * MaxSpeed)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed)
            .withTargetDirection(Constants.kBlueSpeakerLocation.minus(drivetrain.getState().Pose.getTranslation()).unaryMinus().getAngle())
            )).whileTrue(new InstantCommand(() -> {SmartDashboard.putNumber("Test Angle", Constants.kBlueSpeakerLocation.minus(drivetrain.getState().Pose.getTranslation()).getAngle().getDegrees());}));
    
    driveController.leftBumper().onTrue(new Shoot(frontIntake, shooter, climber, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak)));
    driveController.rightBumper().onTrue(new Suck(frontIntake, shooter, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak)));
    //driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak))); 
    driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak))); 
    driveController.leftTrigger(.5).onTrue(new Climb(climber,frontIntake,shooter));
    driveController.y().whileTrue(new SuckNoFront(frontIntake, shooter, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak))); 
    driveController.b().whileTrue(new ShootTrap(climber, shooterIntake)); 
    // Schedules reset the field - Binds centric heading on back and start button push

    driveController.back().and(driveController.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(resetPose)));

    /* Setting up bindings for selecting an autonomous to run. */
    /* Up / Down on the D-Pad of the driver controller. */
    /* Until we start generating paths and creating auton routines, this will cycle through .chrp files.*/
    driveController.povDown().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {autonSelect.decrementSelection();}).andThen(() -> {builtAutonomousCommand = AutoBuilder.buildAuto(autonSelect.getCurrentSelectionName());}).ignoringDisable(true));
    driveController.a().and(RobotState::isDisabled).whileTrue(new SetMotorsToCoast(climber, shooter, frontIntake, shooterIntake).ignoringDisable(true));
    driveController.povUp().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {autonSelect.incrementSelection();}).andThen(() -> {builtAutonomousCommand = AutoBuilder.buildAuto(autonSelect.getCurrentSelectionName());}).ignoringDisable(true));

    //driveController.start().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {builtAutonomousCommand = AutoBuilder.buildAuto(autonSelect.getCurrentSelectionName());}).ignoringDisable(true));

    /*driveController.a().onTrue(new InstantCommand(() -> {
      //I'm not sure if 'tableName' refers to limelight name, or a network table.
      //Need to implement a way to automatically name the files with unique names.
      LimelightHelpers.takeSnapshot("limelight", "snapshot");
    }));*/

    /*driveController.povLeft().onTrue(new InstantCommand(() -> {
      frontIntake.setServoPosition(0.5);;
    }));*/

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    //driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //Limelight piece-by-piece debugging
    //driveController.back().onTrue(new ShooterModeShootWithLimelight(frontIntake, shooter, drivetrain, shooterIntake, beamBreak));
    
    Trigger NoteLocationStatus = new Trigger(beamBreak::getNotePresent);
    NoteLocationStatus.onTrue(new LightLEDOnNotePresent(LED, beamBreak)).onFalse(new LightLEDOnNotePresent(LED, beamBreak));
    
    Trigger ShootingModePass = new Trigger(shooter::getShooterModeDoingSomething);
    ShootingModePass.whileTrue(new DriveWhileFacingAngle(drivetrain, driveController, shooter));

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
    //Line below cauases a crash because it's not ready
    //operatorController.x().onTrue(new ShooterModeShootWithPose(frontIntake, shooter, drivetrain));
    operatorController.a().onTrue(new ShooterModeSubwoofer(frontIntake, shooter));
    operatorController.leftBumper().and(operatorController.rightBumper()).debounce(0.5).onTrue(new PreClimb(climber,shooter,frontIntake, shooterIntake));
    operatorController.start().onTrue(new Home(climber, shooter, frontIntake, shooterIntake));
    operatorController.x().onTrue(new ShooterModePass(frontIntake, shooter));
    
    /* Setting up bindings for selecting a CHIRP file. */
    /* Up / Down on the D-Pad of the operator controller. */
    /* While disabled, press the back button to load the selected file and play the song. */
    /* Press the back button again to stop the song. */
    operatorController.povDown().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {chirpSelect.decrementSelection();}).ignoringDisable(true));
    operatorController.povUp().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {chirpSelect.incrementSelection();}).ignoringDisable(true));
    //Breaking this into multiple lines, because it's a lot to parse.
    operatorController.back().and(RobotState::isDisabled).onTrue(
      new InstantCommand(() -> {music.loadMusic(Filesystem.getDeployDirectory() + "/chirp" + chirpSelect.getCurrentSelectionName() + ".chrp");})
      .andThen(() -> {music.play();}).ignoringDisable(true))
      .onFalse(new InstantCommand(() -> {music.stop();}).ignoringDisable(true));
    
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
    //SmartDashboard.putData("Chirp Selector", chirpSelect);
    SmartDashboard.putData("Auton Selector", autonSelect);  
    SmartDashboard.putData(beamBreak);
    SmartDashboard.putData(climber);
    SmartDashboard.putData(frontIntake);
    SmartDashboard.putData(shooter);
  }

  /**
   * Register all of the commands that can be used in autonomous.
   * If a command is not registered here, referencing it in a PathPlanner autonomous won't do anything.
   */
  private void registerAutonCommands(){
    //ALL COMMANDS THAT COULD BE USED IN AUTONOMOUS NEED TO BE REGISTERED HERE.
    NamedCommands.registerCommand("Shoot", new Shoot(frontIntake, shooter, climber, shooterIntake, beamBreak));
    NamedCommands.registerCommand("ShootTrap", new ShootTrap(climber, shooterIntake));
    NamedCommands.registerCommand("Spit", new Spit(frontIntake, shooter, shooterIntake, beamBreak));
    NamedCommands.registerCommand("Suck", new Suck(frontIntake, shooter, shooterIntake, beamBreak));
    NamedCommands.registerCommand("NotePosition", new NotePosition(shooterIntake, beamBreak));
    NamedCommands.registerCommand("StowArmAndIntake", new StowArmAndIntake(frontIntake, shooter));
    NamedCommands.registerCommand("LightLEDOnNotePresent", new LightLEDOnNotePresent(LED, beamBreak));
    NamedCommands.registerCommand("ShooterModeAmp", new ShooterModeAmp(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModePodium", new ShooterModePodium(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModeShootWithPose", new ShooterModeShootWithPose(frontIntake, shooter, drivetrain));
    NamedCommands.registerCommand("ShooterModeSubwoofer", new ShooterModeSubwoofer(frontIntake, shooter));
    NamedCommands.registerCommand("Home", new Home(climber,shooter,frontIntake, shooterIntake));
    NamedCommands.registerCommand("ApplyBrake", drivetrain.applyRequest(() -> brake));
    NamedCommands.registerCommand("ShooterModeRightAuto", new ShooterModeRightAuto(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModeMiddleAuto", new ShooterModeMiddleAuto(frontIntake, shooter));
  }

  /**
   * Add all of the motors from each subsystem (except the drivetrain) to the Orchestra.
   */
  private void registerMotorsToOrchestra() {
    shooter.addToOrchestra(music);
    frontIntake.addToOrchestra(music);
    climber.addToOrchestra(music);
    shooterIntake.addToOrchestra(music);
  }

  public RobotContainer() {

    //Start the CTRE logger for sysID use
    //SignalLogger.start();

    registerMotorsToOrchestra();
    configureBindings();
    registerAutonCommands();
    configureCustomNTValues();

    //===================Default Commands==============================
    //shooter.setDefaultCommand(new FrontIntakeDefault(frontIntake, shooter));
  }

  public Command getAutonomousCommand() {
    //This should load the selected autonomous file.
    return builtAutonomousCommand;
  }
}
