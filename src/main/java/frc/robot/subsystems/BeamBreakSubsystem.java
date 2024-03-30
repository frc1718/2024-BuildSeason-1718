// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
public class BeamBreakSubsystem extends SubsystemBase {

  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog);

  Debouncer m_IntakeDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
  Debouncer m_ShooterDebounce = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

  //Open Servo
  //Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  public BeamBreakSubsystem() {
  m_BeamBreakIntakeAnalog.setAverageBits(6);
  m_BeamBreakShooterAnalog.setAverageBits(6);
  }
  
  // Start of sensor related methods
  /**
   * Check if a note is in front of the intake beam break.
   * @return Whether the intake beam break detects a note.
   * True or false.
   */
  public boolean getNotePresentIntake() {  
    if (Constants.kPrintSubsystemBeamBreak){System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());}
    
    return (m_IntakeDebounce.calculate(m_BeamBreakIntakeAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover));
  }
  
  /**
   * Check if a note is in front of the shooter beam break.
   * @return Whether the shooter beam break detects a note.
   * True or false.
   */
  public boolean getNotePresentShooter() {  
    if (Constants.kPrintSubsystemBeamBreak){
      System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
      System.out.println("Subsystem: Shooter - getNotePresentShooter :" + (m_BeamBreakShooterAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover));
    }
    return (m_ShooterDebounce.calculate(m_BeamBreakShooterAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover));
    
  }
  // End of sensor related methods

  /**
   * Check if a note is in front of either the intake beam break or the shooter beam break.
   * @return Whether either beam break detects a note.
   * True or false.
   */
  public boolean getNotePresent() {
    if (Constants.kPrintSubsystemBeamBreak){
      System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
      System.out.println("Subsystem: Shooter - getNotePresentIntake Voltage " + m_BeamBreakIntakeAnalog.getAverageVoltage());
    }
    return (this.getNotePresentIntake() || this.getNotePresentShooter());
  }

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */

  /**
   * Extends the intake hinge servo to allow the intake to pivot.
   */
  
  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("BeamBreakSubsystem");
    builder.addBooleanProperty("Note Present in Intake?", this::getNotePresentIntake, null); 
    builder.addBooleanProperty("Note Present in Shooter?", this::getNotePresentShooter, null);
    builder.addDoubleProperty("Intake Beam Break Voltage", () -> {return m_BeamBreakIntakeAnalog.getAverageVoltage();}, null);
    builder.addDoubleProperty("Shooter Beam Break Voltage", () -> {return m_BeamBreakShooterAnalog.getAverageVoltage();}, null);
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
