// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
 */
public class ShooterIntakeSubsystem extends SubsystemBase {
 

  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog);
  
  /*  
  //Open Servo
  //Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  //Open Motors
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");

  private final VelocityVoltage ShooterIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  */

  /**
   * Constructs an instance of the shooter intake subsystem.
   * The motor and sensor configuration is done here.
   */
  public ShooterIntakeSubsystem() {
  //=======================================shouldn't we be doing something with velocity voltage motor here to set it up?
  m_BeamBreakIntakeAnalog.setAverageBits(4);
  m_BeamBreakShooterAnalog.setAverageBits(4);
  
  }
  
  // Start of sensor related methods
  /**
   * Check if a note is in front of the intake beam break.
   * @return Whether the intake beam break detects a note.
   * True or false.
   */
  public boolean getNotePresentIntake() {  
    //This will print out constantly because it's on a trigger
    //System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
    //This needs to be both on a debounce AND to have hysterisis programmed in
    return (m_BeamBreakIntakeAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover);
    
  }
  
  /**
   * Check if a note is in front of the shooter beam break.
   * @return Whether the shooter beam break detects a note.
   * True or false.
   */
  public boolean getNotePresentShooter() {  
    //This will print out constantly because it's on a constant trigger
    //System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());   
    return (m_BeamBreakShooterAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover);
    
  }
  // End of sensor related methods

  /**
   * Check if a note is in front of either the intake beam break or the shooter beam break.
   * @return Whether either beam break detects a note.
   * True or false.
   */
  public boolean getNotePresent() {
    System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
    System.out.println("Subsystem: Shooter - getNotePresentIntake Voltage " + m_BeamBreakIntakeAnalog.getAverageVoltage());

    return (this.getNotePresentIntake() || this.getNotePresentShooter());
  }

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */
  public void setShooterIntakeSpeed(double speed) {
    System.out.println("ShooterIntakeSubsystem: setShooterIntakeSpeed");
    //m_ShooterIntakeSpin.setControl(ShooterIntakeVelocity.withVelocity(speed));
  }

  /**
   * Extends the intake hinge servo to allow the intake to pivot.
   */
  public void Release() {
    System.out.println("ShooterIntakeSubsystem: setShooterPivotPosition");
    //intakeHinge.set(desiredPosition);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ShooterIntakeSubsystem");
    builder.addBooleanProperty("Note Present in Intake?", this::getNotePresentIntake, null); 
    builder.addBooleanProperty("Note Present in Shooter?", this::getNotePresentShooter, null);
    builder.addDoubleProperty("Intake Beam Break Voltage", () -> {return m_BeamBreakIntakeAnalog.getVoltage();}, null);
    builder.addDoubleProperty("Shooter Beam Break Voltage", () -> {return m_BeamBreakShooterAnalog.getVoltage();}, null);
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
