// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.General.NotePosition;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private NotePosition m_notePositionCommand;
  private Command m_autonLoading;
  public static boolean robotClimbed=false;
  
  public boolean hasFilteredAutonRoutines = false;

  //Use this to enable / disable reading data from the limelight.
  //The terminal gets clogged up if a limelight isn't actually connected.
  boolean enableLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_notePositionCommand = new NotePosition(m_robotContainer.shooterIntake, m_robotContainer.beamBreak);

    //Load a 'dummy' auton just to load the PathPlannerAuto class.
    //Hopefully, this makes the auton load faster.
    m_autonLoading = new PathPlannerAuto("Blue - Score 0").ignoringDisable(true);
    m_autonLoading.schedule();

    //Setting up port forwarding for all limelight related ports.
    //Only setting the port-forwarding once in the code.
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    //Set a custom brownout voltage for the RoboRIO.
    //Only works with the RIO2.
    RobotController.setBrownoutVoltage(Constants.kCustomBrownout);

    //Start a simple recording to the data log.
    //This should log the contents of the NetworkTables, which should be good for now.
    DataLogManager.start();
    //This should log the joysticks as well.
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Currently disabled with the enableLimelight variable.
    if (enableLimelight) {
      //Periodically retrieve the results from the limelight and extract the pose.
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.kLimelightName);
      if (limelightMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {
    //Had a weird issue.
    //Moving the shooter arm or front intake during disabled (like stowing to practice auton routines) was causing the arm and intake to interfere once enabled.
    //Solution, briefly set them in a zero output mode when disabled, then allow Motion Magic to take control again in TeleOp.
    m_robotContainer.frontIntake.setFrontIntakeRotateZeroOutput();
    m_robotContainer.shooter.setShooterArmRotateZeroOutput();
    m_robotContainer.shooter.setShooterSpeed(0);
  }

  @Override
  public void disabledPeriodic() {
    //double m_DistanceBetweenAprilTagAndLimelight = Constants.kSpeakerAprilTagHeight - Constants.kLimelightHeight;
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getAngle());
    /*double m_VerticalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTY(Constants.kLimelightName));
    double m_HorizontalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTX(Constants.kLimelightName));
    double m_DistanceToAprilTag = m_DistanceBetweenAprilTagAndLimelight / (Math.tan(m_VerticalAngleToAprilTag));
    System.out.println(m_DistanceToAprilTag);*/

    //Check if the robot is in communication with the Driver Station.
    //If it is, attempt to filter the autonomous routines based on alliance color.
    if (!hasFilteredAutonRoutines && DriverStation.isDSAttached()) {
      switch (DriverStation.getAlliance().get()) {
        case Blue:
          m_robotContainer.autonSelect.filterSelections("Blue");
        break;
        case Red:
          m_robotContainer.autonSelect.filterSelections("Red");
        break;
      }

      //Sort the new list alphabetically.  Also sort the chirp file list.
      m_robotContainer.autonSelect.sortSelections();
      m_robotContainer.chirpSelect.sortSelections();

      //After filtering the auton list, set a flag so the filter is only applied once.
      hasFilteredAutonRoutines = true;

      };
  }
  
  @Override
  public void disabledExit() {}

  @Override

  public void autonomousInit() {
    //Add a 10ms wait before returning the selected autonomous routine.
    //Should help reduce the delay between the official beginning of autonomous and the start of the PathPlanner autonomous.
    m_autonomousCommand = new WaitCommand(0.010).andThen(m_robotContainer.getAutonomousCommand());
    m_robotContainer.shooterIntake.setShooterIntakeRotate(Constants.kShooterIntakeRotateHomePos);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.shooterIntake.setShooterIntakeRotate(Constants.kShooterIntakeRotateHomePos);
    m_notePositionCommand.schedule();
    
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      //System.out.println("Alliance Color is: " + DriverStation.getAlliance().get());
      m_robotContainer.resetPose = Constants.resetPoseBlue;
    } else {
      //System.out.println("Alliance Color is: " + DriverStation.getAlliance().get());
      m_robotContainer.resetPose = Constants.resetPoseRed;
    } 
    
  }
  
  public void teleopPeriodic() {
    //double m_DistanceBetweenAprilTagAndLimelight = Constants.kSpeakerAprilTagHeight - Constants.kLimelightHeight;
    SmartDashboard.putNumber("Pigeon", m_robotContainer.drivetrain.getPigeon2().getAngle());
    /*double m_VerticalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTY(Constants.kLimelightName));
    double m_HorizontalAngleToAprilTag = Math.toRadians(LimelightHelpers.getTX(Constants.kLimelightName));
    double m_DistanceToAprilTag = m_DistanceBetweenAprilTagAndLimelight / (Math.tan(m_VerticalAngleToAprilTag) * Math.cos(m_HorizontalAngleToAprilTag));
    System.out.println(m_DistanceToAprilTag);*/
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
