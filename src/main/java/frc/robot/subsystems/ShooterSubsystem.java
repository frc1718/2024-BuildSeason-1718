// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements NTSendable{
 
  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog);
  
  //Open Servo
  //Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  //Open Motors
  // TalonFX m_ShooterRotateLeft = new TalonFX(Constants.kShooterRotateLeftCanID, "Canivore");
  // TalonFX m_ShooterRotateRight = new TalonFX(Constants.kShooterRotateRightCanID, "Canivore");
  // TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");
  
  public String m_shooterMode = "";
  public boolean m_readyToShoot = false;

  public ShooterSubsystem() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void setShooterReadyToShoot(boolean readyToShoot) {
  m_readyToShoot = readyToShoot;
}

public boolean getShooterReadyToShoot() {
  return m_readyToShoot;
}

  // Start of sensor related methods
  public boolean getNotePresentIntake() {  
    //System.out.println("Subsystem: Shooter - getNotePresentIntake");
    return (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kIntakeBeamBreakCrossover);
    
  }
   
  public boolean getNotePresentShooter() {  
    //System.out.println("Subsystem: Shooter - getNotePresentShooter");
    return (m_BeamBreakShooterAnalog.getVoltage() >= Constants.kShooterBeamBreakCrossover);

  }
  // End of sensor related methods

  // Start of motor set methods

  public void setShooterMode(String shooterMode){
    m_shooterMode=shooterMode;
    System.out.println("Subsystem: Shooter - setShooterMode");
  }

  public String getShooterMode(){
    System.out.println("Subsystem: Shooter - getShooterMode");
    return m_shooterMode;
  }

  public void setShooterIntakeSpeed(double speed) {
    System.out.println("Subsystem: Shooter - setShooterIntakeSpeed");
  }

  public void setShooterSpeed(double speed) {
    //Set both right and left shooter motor speeds here
    //Don't we only have to set one motor because one will be a follower?
    System.out.println("Subsystem: Shooter - setShooterSpeed");
  }

  public void setShooterArmPosition(int position) {
    System.out.println("Subsystem: Shooter - setShooterArmPosition");
  }

  public void setShooterIntakePivotPosition(double desiredPosition) {
    System.out.println("Subsystem: Shooter - setShooterIntakePivotPosition");
  }
  //End of motor set methods

  //Start of motor get methods
  public int getShooterSpeed() {
    int integer = 0;
    System.out.println("Subsystem: Shooter - getShooterSpeed");
    return integer;
  }

  public int getShooterArmPosition() {
    System.out.println("Subsystem: Shooter - GetShooterArmPosition");
    return 1;
  }

  public boolean getShooterUpToSpeed(int desiredSpeed) {
    System.out.println("Subsystem: Shooter - getShooterUpToSpeed");
    if ((desiredSpeed - Constants.kShooterSpeedTolerance) >= getShooterSpeed() && getShooterSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getShooterArmInPosition(int desiredPosition) {
      System.out.println("Subsystem: Shooter - getShooterArmInPosition");
      /* if (m_ShooterRotateLeft.getPosition().getValue() > (desiredPosition-Constants.kShooterArmTolerancePos) && (m_ShooterRotateLeft.getPosition().getValue() < (desiredPosition+Constants.kShooterArmTolerancePos)))
      {
        return true; 
      } else
      {
        return false;
      } */
      return false;
      
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
