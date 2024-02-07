// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
 
  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog); 

  //Open Motors
  // TalonFX m_ShooterRotateLeft = new TalonFX(Constants.kShooterRotateLeftCanID, "Canivore");
  // TalonFX m_ShooterRotateRight = new TalonFX(Constants.kShooterRotateRightCanID, "Canivore");
  // TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");

  public ShooterSubsystem() {
  }

  public Command shoot() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {

        });
  }

  public Command extra() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

        });
  }
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean shooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  // Start of sensor related methods
  public boolean getNotePresentIntake() {  
    return (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kIntakeBeamBreakCrossover);
  }
   
  public boolean getNotePresentShooter() {  
    return (m_BeamBreakShooterAnalog.getVoltage() >= Constants.kShooterBeamBreakCrossover);
  }

  public double getIntakeVolt() {
    return m_BeamBreakIntakeAnalog.getVoltage();
  }
  
  public double getShooterVolt() {
    return m_BeamBreakShooterAnalog.getVoltage();
  }

  public void printGetNotePresentShooter() {
    double printStatement = m_BeamBreakShooterAnalog.getVoltage();
    System.out.println(printStatement);
  }

  // Start of motor control methods
  public void runShooterIntake(double speed) {

  }

  public void runRightShooter(double speed) {

  }

  public void runLeftShooter(double speed) {
    
  }

  public void shooterArmToPosition(int position, double power) {
    
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");
    builder.addBooleanProperty("Note Present in Intake?", this::getNotePresentIntake, null); 
    builder.addBooleanProperty("Note Present in Shooter?", this::getNotePresentShooter, null);
    builder.addDoubleProperty("Intake Beam Break Voltage", () -> {return m_BeamBreakIntakeAnalog.getVoltage();}, null);
    builder.addDoubleProperty("Shooter Beam Break Voltage", () -> {return m_BeamBreakShooterAnalog.getVoltage();}, null);
    builder.addStringProperty("Shooter Mode", () -> {return "null"; /*this::getShooterMode*/}, null);
    builder.addDoubleProperty("Shooter Arm Position", () -> {return 0; /*this::getShooterArmPosition*/}, null);
    builder.addDoubleProperty("Shooter Speed", () -> {return 0; /*this::getShooterSpeed*/}, null);
    builder.addBooleanProperty("Shooter Arm in Position?", () -> {return false; /*this::getShooterArmInPosition*/}, null);
    builder.addBooleanProperty("Shooter up to Speed?", () -> {return false; /*this::getShooterUpToSpeed*/}, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
