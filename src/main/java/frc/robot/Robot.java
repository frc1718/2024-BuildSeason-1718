// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.Results;
import frc.robot.commands.General.DisabledSafety;
import frc.robot.commands.General.NotePosition;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private NotePosition m_notePositionCommand;
  private RobotContainer m_robotContainer;
  private DisabledSafety m_disabledSafetyCommand;
  private Command m_autonLoading;
  
  //Use this to enable / disable reading data from the limelight.
  //The terminal gets clogged up if a limelight isn't actually connected.
  boolean enableLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_disabledSafetyCommand = new DisabledSafety(m_robotContainer.shooterIntake,m_robotContainer.frontIntake,m_robotContainer.shooter,m_robotContainer.climber);
    m_notePositionCommand = new NotePosition(m_robotContainer.shooterIntake,m_robotContainer.beamBreak);
    //Load a 'dummy' auton just to load the PathPlannerAuto class.
    //Hopefully, this makes the auton load faster.
    m_autonLoading = new PathPlannerAuto("Blue - Score 0").ignoringDisable(true);
    //Need to seed the field relative position at least once.
    //This may not be necessary with autonomous.  It might actually interfere!
    //m_robotContainer.drivetrain.seedFieldRelative(Constants.kDefaultPose);

    //CameraServer.startAutomaticCapture();

    //Setting up port forwarding for all limelight related ports.
    //Only setting the port-forwarding once in the code.
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*if (enableLimelight) {
      //Periodically retrieve the results from the limelight and extract the pose.
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.kLimelightName);
      if (limelightMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      }

      //Pose2d limelightPose = limelightResults.getBotPose2d();

      //This validity check will probably have more logic to it in the future, but for now, just check if the latest results are valid.
      //if (limelightResults.valid) {
        //botpose[6] is the combined targeting latency and capture latency.  Subtract from the current time to determine when the results were calculated.
        //m_robotContainer.drivetrain.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (limelightResults.botpose[6] / 1000));
      //}
      } */
  }

  @Override
  public void disabledInit() {
    //Had a weird issue.  Moving the shooter arm or front intake during disabled (like stowing to practice auton routines) was causing the arm and intake to interfere once enabled.
    //Solution, briefly set them in a zero output mode when disabled, then allow Motion Magic to take control again in TeleOp.
    m_robotContainer.frontIntake.setFrontIntakeRotateZeroOutput();
    m_robotContainer.shooter.setShooterArmRotateZeroOutput();
    m_autonLoading.schedule();
    
  }

  @Override
  public void disabledPeriodic() {
    //If we have been in disabled more than 5 seconds, stop everything and set desired positions to current
    //if (disabledTimer.get() > 5.0) {
    //  m_disabledSafetyCommand.schedule();
    //}
   }

  

  @Override
  public void disabledExit() {
    //Stop and disabled timer

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new WaitCommand(0.010).andThen(m_robotContainer.getAutonomousCommand());

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
    m_notePositionCommand.schedule();
    
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      //System.out.println("Alliance Color is: " + DriverStation.getAlliance().get());
      m_robotContainer.resetPose = m_robotContainer.resetPoseBlue;
    } else {
      //System.out.println("Alliance Color is: " + DriverStation.getAlliance().get());
      m_robotContainer.resetPose = m_robotContainer.resetPoseRed;
    }
  }
  
  public void teleopPeriodic() {}

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
