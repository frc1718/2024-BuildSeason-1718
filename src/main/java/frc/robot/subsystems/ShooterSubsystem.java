// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements NTSendable{
 
  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog);
  
  //Open Servo
  Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  //Open Motors
  TalonFX m_ShooterRotateLeft = new TalonFX(Constants.kShooterRotateLeftCanID, "Canivore");
  TalonFX m_ShooterRotateRight = new TalonFX(Constants.kShooterRotateRightCanID, "Canivore");
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");

  public ShooterSubsystem() {}
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean shooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  // Start of sensor related methods
  public boolean getNotePresentIntake() {  
    return (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kIntakeBeamBreakCrossover);
  }
   
  public boolean getNotePresentShooter() {  
    return (m_BeamBreakShooterAnalog.getVoltage() >= Constants.kShooterBeamBreakCrossover);
  }
  // End of sensor related methods

  // Start of motor set methods
  public void setShooterIntakeSpeed(double speed) {

  }

  public void setShooterSpeed(double speed) {
    //Set both right and left shooter motor speeds here
    //Don't we only have to set one motor because one will be a follower?
  }

  public void setShooterArmPosition(int position) {
    
  }

  public void setShooterIntakePivotPosition(double desiredPosition) {

  }
  //End of motor set methods

  //Start of motor get methods
  public int getShooterSpeed() {
    int integer = 0;
    return integer;
  }

  public int getShooterArmPosition() {
    return 1;
  }

  public boolean getShooterUpToSpeed(int desiredSpeed) {
    if ((desiredSpeed - Constants.kShooterSpeedTolerance) >= getShooterSpeed() && getShooterSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getShooterArmInPosition(int desiredPosition) {
      if (m_ShooterRotateLeft.getPosition().getValue() > (desiredPosition-Constants.kShooterArmTolerancePos) && (m_ShooterRotateLeft.getPosition().getValue() < (desiredPosition+Constants.kShooterArmTolerancePos)))
      {
        return true; 
      } else
      {
        return false;
      }
      
    }
    //End of motor get methods

  @Override
  public void initSendable(NTSendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");
    builder.addDoubleProperty("Current Voltage", () -> {return m_BeamBreakShooterAnalog.getVoltage();}, null); 
    builder.addBooleanProperty("Beam Broken", () -> {return (m_BeamBreakShooterAnalog.getVoltage() > Constants.kShooterBeamBreakCrossover);}, null);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
