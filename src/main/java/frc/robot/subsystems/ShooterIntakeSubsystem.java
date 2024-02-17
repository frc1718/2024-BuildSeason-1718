// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  public ShooterIntakeSubsystem() {
  //=======================================shouldn't we be doing something with velocity voltage motor here to set it up?
  m_BeamBreakIntakeAnalog.setAverageBits(4);
  m_BeamBreakShooterAnalog.setAverageBits(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  // Start of sensor related methods
  public boolean getNotePresentIntake() {  
    //This will print out constantly because it's on a trigger
    //System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
    //This needs to be both on a debounce AND to have hysterisis programmed in
    return (m_BeamBreakIntakeAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover);
    
  }
  
  public boolean getNotePresentShooter() {  
    //This will print out constantly because it's on a constant trigger
    //System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());   
    return (m_BeamBreakShooterAnalog.getAverageVoltage() >= Constants.kShooterBeamBreakCrossover);
    
  }
  // End of sensor related methods

  public boolean getNotePresent() {
    //System.out.println("Subsystem: Shooter - getNotePresentShooter Voltage " + m_BeamBreakShooterAnalog.getAverageVoltage());
    //System.out.println("Subsystem: Shooter - getNotePresentIntake Voltage " + m_BeamBreakIntakeAnalog.getAverageVoltage());

    if ((m_BeamBreakIntakeAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover) || (m_BeamBreakShooterAnalog.getAverageVoltage() >= Constants.kIntakeBeamBreakCrossover)){
      return true;  
    } else {
      return false;
    }
  }

  public void setShooterIntakeSpeed(double speed) {
    System.out.println("ShooterIntakeSubsystem: setShooterIntakeSpeed");
    //m_ShooterIntakeSpin.setControl(ShooterIntakeVelocity.withVelocity(speed));
  }

  public void Release() {
    System.out.println("ShooterIntakeSubsystem: setShooterPivotPosition");
    //intakeHinge.set(desiredPosition);
  }
 
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
