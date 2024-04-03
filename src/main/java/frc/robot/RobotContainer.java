// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDs.LightLEDOnNotePresent;
import frc.robot.commands.Operator.Home;
import frc.robot.commands.Operator.PreClimb;
import frc.robot.commands.Operator.ShooterModeAmp;
import frc.robot.commands.Drive;
import frc.robot.commands.Auto.AutoShooterModePodium;
import frc.robot.commands.Auto.AutoShooterModePos1;
import frc.robot.commands.Auto.AutoShooterModePos2;
import frc.robot.commands.Auto.AutoShooterModePos3;
import frc.robot.commands.Operator.ShooterModePass;
import frc.robot.commands.Operator.ShooterModePodium;
import frc.robot.commands.Operator.ShooterModeSubwoofer;
import frc.robot.commands.Driver.Climb;
import frc.robot.commands.Driver.Shoot;
import frc.robot.commands.Driver.ShootTrap;
import frc.robot.commands.Driver.ShooterModeShootWithLimelight;
import frc.robot.commands.Driver.Spit;
import frc.robot.commands.Driver.Suck;
import frc.robot.commands.Driver.SuckNoFront;
import frc.robot.commands.General.SetMotorsToCoast;
import frc.robot.commands.General.NotePosition;
import frc.robot.commands.General.StowArmAndIntake;
import frc.robot.generated.TunerConstants;

//Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CornerRollerSubsystem;
import frc.robot.subsystems.FrontIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double driveSign = 1;
  public Pose2d resetPose = new Pose2d();

  public Command builtAutonomousCommand;

  //Attempting to create a selector object.
  File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
  File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
  Selector chirpSelect = new Selector(chirpFolder, ".chrp");
  Selector autonSelect = new Selector(autonFolder, ".auto");
  
  // Set up of the driver and operator controller
  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  /* Setting up bindings for necessary control of the swerve drive platform */
  /* The drivetrain is public so the limelight pose can be added to it in Robot.java. */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop

  //Testing an idea: hold a button and always aim at the goal.
  //Copying a fair amount of this from the FieldCentric drive.
  private final SwerveRequest.FieldCentricFacingAngle rootyTootyPointAndShooty = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity);  

  //An example of the robot centric swerve request.
  //Useful for helping the driver line up with the chain.
  //private final SwerveRequest.RobotCentric climbAlignment = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1);                                                                   
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

  //Open Subsystems
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final FrontIntakeSubsystem frontIntake = new FrontIntakeSubsystem();
  public final ClimberSubsystem climber = new ClimberSubsystem();
  public final LEDSubsystem LED = new LEDSubsystem();
  public final ShooterIntakeSubsystem shooterIntake = new ShooterIntakeSubsystem();
  public final BeamBreakSubsystem beamBreak = new BeamBreakSubsystem();
  public final CornerRollerSubsystem cornerRoller = new CornerRollerSubsystem();

  public final Orchestra music = new Orchestra();

  private void configureBindings() {
    //Schedules drivertain
    drivetrain.setDefaultCommand(new Drive(drivetrain,driveController,shooter,climber,driveSign)); 

    /********************************************************************/
    /*                  Driver Controller Assignments                   */
    /********************************************************************/
    /********************************************************************/
    /*  Y - Suck Without Front Intake                                   */
    /*  B - Trap Shot                                                   */
    /*  X - Drivetrain Brake Mode                                       */
    /*  A (While Disabled) - Set Motors to Coast                        */
    /*  Left Bumper - Shoot                                             */
    /*  Right Bumper - Suck                                             */
    /*  Left Trigger - Climb                                            */
    /*  Right Trigger - Spit                                            */
    /*  Back - Enable 'ShootWithLimelight' Shooter mode                 */
    /*  Start + Back - Reset Robot Pose                                 */
    /*  D-Pad Up (While Disabled) - Increment & Select Autonomous       */
    /*  D-Pad Down (While Disabled) - Decrement & Select Autonomous     */
    /********************************************************************/

    driveController.y().whileTrue(new SuckNoFront(frontIntake, shooter, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak)));
    driveController.b().whileTrue(new ShootTrap(climber, shooterIntake));
    driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController.a().and(RobotState::isDisabled).whileTrue(new SetMotorsToCoast(climber, shooter, frontIntake, shooterIntake).ignoringDisable(true));    

    driveController.leftBumper().onTrue(new Shoot(frontIntake, shooter, climber, shooterIntake, beamBreak)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak)));
    driveController.rightBumper().onTrue(new Suck(frontIntake, shooter, shooterIntake, beamBreak, cornerRoller)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak)));
    driveController.leftTrigger(.5).onTrue(new Climb(climber,frontIntake,shooter));
    driveController.rightTrigger(.5).whileTrue(new Spit(frontIntake, shooter, shooterIntake, beamBreak, cornerRoller)).onFalse(Commands.parallel(new StowArmAndIntake(frontIntake, shooter), new NotePosition(shooterIntake, beamBreak))); 


    //driveController.back().onTrue(new ShooterModeShootWithLimelight(frontIntake, shooter, drivetrain, shooterIntake, beamBreak));
    
    // Schedules reset the field - Binds centric heading on back and start button push

    driveController.back().and(driveController.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(resetPose)));

    /************************************************************/
    /* Setting up bindings for selecting an autonomous to run.  */
    /* Up / Down on the D-Pad of the driver controller.         */
    /************************************************************/
    driveController.povUp().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {autonSelect.incrementSelection();}).andThen(() -> {builtAutonomousCommand = AutoBuilder.buildAuto(autonSelect.getCurrentSelectionName());}).ignoringDisable(true));
    driveController.povDown().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {autonSelect.decrementSelection();}).andThen(() -> {builtAutonomousCommand = AutoBuilder.buildAuto(autonSelect.getCurrentSelectionName());}).ignoringDisable(true));

    /******************************************************************************************************************/
    /*  Bindings for drivetrain characterization                                                                      */
    /*  These bindings require multiple buttons pushed to swap between quastatic and dynamic                          */
    /*  Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction                                   */
    /*  driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));       */
    /*  driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));       */
    /*  driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));  */
    /*  driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));  */
    /******************************************************************************************************************/
    
    /****************************************************************/
    /*                      Trigger Assignments                     */
    /****************************************************************/
    /****************************************************************/
    /*  NoteLocationStatus - Toggles Indication LEDs                */
    /*  ShootingModePass - Drivetrain Drive While Facing Angle Mode */
    /****************************************************************/

    Trigger NoteLocationStatus = new Trigger(beamBreak::getNotePresent);
    Trigger ShootingModePass = new Trigger(shooter::getShooterModeDoingSomething);

    NoteLocationStatus.onTrue(new LightLEDOnNotePresent(LED, beamBreak)).onFalse(new LightLEDOnNotePresent(LED, beamBreak));
    //ShootingModePass.whileTrue(new DriveWhileFacingAngle(drivetrain, driveController, shooter));

    /******************************************************************/
    /*                Operator Controller Assignments                 */
    /******************************************************************/
    /******************************************************************/
    /*  Y - Set Podium Shot                                           */
    /*  B - Set Amp Shot                                              */
    /*  X - Set Pass Shot                                             */
    /*  A - Set Subwoofer Shot                                        */
    /*  Start - Send Subsystems to Home Position                      */
    /*  Left Bumper + Right Bumper - Enter Pre-Climb Mode             */
    /*  D-Pad Up (While Disabled) - Increment CHIRP File Selection    */
    /*  D-Pad Down (While Disabled) - Decrement CHIRP File Selection  */
    /*  Back (While Disabled) - Play / Stop Current CHIRP Selection   */
    /*  Start + Right Trigger (While Disabled) - Theremin Mode        */
    /******************************************************************/

    operatorController.y().onTrue(new ShooterModePodium(frontIntake, shooter));
    operatorController.b().onTrue(new ShooterModeAmp(frontIntake, shooter));
    operatorController.x().onTrue(new ShooterModePass(frontIntake, shooter));
    operatorController.a().onTrue(new ShooterModeSubwoofer(frontIntake, shooter));
    operatorController.leftBumper().and(operatorController.rightBumper()).onTrue(new PreClimb(climber,shooter,frontIntake, shooterIntake));
    operatorController.start().onTrue(new Home(climber, shooter, frontIntake, shooterIntake));

    /**************************************************************************************/
    /* Setting up bindings for selecting a CHIRP file.                                    */
    /* Up / Down on the D-Pad of the operator controller.                                 */
    /* While disabled, press the back button to load the selected file and play the song. */
    /* Press the back button again to stop the song.                                      */
    /**************************************************************************************/
    operatorController.povUp().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {chirpSelect.incrementSelection();}).andThen(() -> {music.loadMusic(Filesystem.getDeployDirectory() + "/chirp/" + chirpSelect.getCurrentSelectionName() + ".chrp");}).ignoringDisable(true));
    operatorController.povDown().and(RobotState::isDisabled).onTrue(new InstantCommand(() -> {chirpSelect.decrementSelection();}).andThen(() -> {music.loadMusic(Filesystem.getDeployDirectory() + "/chirp/" + chirpSelect.getCurrentSelectionName() + ".chrp");}).ignoringDisable(true));

    //Breaking this into multiple lines, because it's a lot to parse.
    operatorController.back().and(RobotState::isDisabled).onTrue(
      new InstantCommand(() -> {music.play();}).ignoringDisable(true))
      .onFalse(new InstantCommand(() -> {music.stop();}).ignoringDisable(true));
    
    /************************************************************/
    /* Turn the robot into a Theremin.                          */
    /* All motors are included.                                 */
    /* Use the right operator trigger to control the frequency. */
    /* Currently not usable.                                    */
    /************************************************************/
    /*operatorController.start().and(operatorController.rightTrigger(0.1)).and(RobotState::isDisabled).whileTrue(
      new InstantCommand(() -> {
        shooter.setMusicToneFrequency(operatorController.getRightTriggerAxis());
        frontIntake.setMusicToneFrequency(operatorController.getRightTriggerAxis());
        climber.setMusicToneFrequency(operatorController.getRightTriggerAxis());
        shooterIntake.setMusicToneFrequency(operatorController.getRightTriggerAxis());
        cornerRoller.setMusicToneFrequency(operatorController.getRightTriggerAxis());
        drivetrain.setMusicToneFrequency(operatorController.getRightTriggerAxis());}
    ).ignoringDisable(true)).onFalse(
      new InstantCommand(() -> {
        shooter.setMusicToneFrequency(0);
        frontIntake.setMusicToneFrequency(0);
        climber.setMusicToneFrequency(0);
        shooterIntake.setMusicToneFrequency(0);
        cornerRoller.setMusicToneFrequency(0);
        drivetrain.setMusicToneFrequency(0);}).ignoringDisable(true));
    */

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
    NamedCommands.registerCommand("Spit", new Spit(frontIntake, shooter, shooterIntake, beamBreak, cornerRoller));
    NamedCommands.registerCommand("Suck", new Suck(frontIntake, shooter, shooterIntake, beamBreak, cornerRoller));
    NamedCommands.registerCommand("NotePosition", new NotePosition(shooterIntake, beamBreak));
    NamedCommands.registerCommand("StowArmAndIntake", new StowArmAndIntake(frontIntake, shooter));
    NamedCommands.registerCommand("LightLEDOnNotePresent", new LightLEDOnNotePresent(LED, beamBreak));
    NamedCommands.registerCommand("ShooterModeAmp", new ShooterModeAmp(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModePodium", new ShooterModePodium(frontIntake, shooter));
    NamedCommands.registerCommand("AutoShooterModePodium", new AutoShooterModePodium(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModeSubwoofer", new ShooterModeSubwoofer(frontIntake, shooter));
    NamedCommands.registerCommand("Home", new Home(climber,shooter,frontIntake, shooterIntake));
    NamedCommands.registerCommand("ApplyBrake", drivetrain.applyRequest(() -> brake));
    NamedCommands.registerCommand("ShooterModePos1", new AutoShooterModePos1(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModePos2", new AutoShooterModePos2(frontIntake, shooter));
    NamedCommands.registerCommand("ShooterModePos3", new AutoShooterModePos3(frontIntake, shooter));
  }

  /**
   * Add all of the motors from each subsystem (except the drivetrain) to the Orchestra.
   */
  private void registerMotorsToOrchestra() {
    shooter.addToOrchestra(music);
    frontIntake.addToOrchestra(music);
    climber.addToOrchestra(music);
    shooterIntake.addToOrchestra(music);
    cornerRoller.addToOrchestra(music);
    drivetrain.addToOrchestra(music);
  }

  public RobotContainer() {
    //Start the CTRE logger for sysID use
    //SignalLogger.start();

    registerMotorsToOrchestra();
    configureBindings();
    registerAutonCommands();
    configureCustomNTValues();
  }

  public Command getAutonomousCommand() {
    //This loads the selected autonomous file.
    return builtAutonomousCommand;
  }
}
