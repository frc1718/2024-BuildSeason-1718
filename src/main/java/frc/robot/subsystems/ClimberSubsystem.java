// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  boolean preClimbActuated = false;

  //Open hardware
  Servo m_IntakePivotRelease = new Servo(Constants.kShooterIntakePivotReleasePWM);
  
  //RightClimb Should be a follower of leftclimb
  TalonFX m_LeftClimb = new TalonFX(Constants.kLeftClimbCanID, "Canivore");
  TalonFX m_RightClimb = new TalonFX(Constants.kRightClimbCanID, "Canivore");

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setClimberDesiredPosition(int desiredPosition) {
    
  }

  public boolean preClimbActuated(boolean preClimb){
    if ((preClimbActuated!=true) && (preClimb=true )){
      preClimbActuated=true;
    }
  return preClimbActuated;
  }

  public int getClimberPosition() {   
    return 1;
  }

  public boolean getClimberInPosition (int desiredPosition) {
    if (m_LeftClimb.getPosition().getValue() > (desiredPosition-Constants.kClimberTolerancePos) && (m_LeftClimb.getPosition().getValue() < (desiredPosition+Constants.kClimberTolerancePos)))
    {
      return true; 
    } else
    {
      return false;
    }
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
