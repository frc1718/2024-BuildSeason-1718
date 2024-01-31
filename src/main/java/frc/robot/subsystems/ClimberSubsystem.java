// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  
  //Open hardware
  Servo m_IntakePivotRelease = new Servo(Constants.kShooterIntakePivotReleasePWM);
  TalonFX m_LeftClimb = new TalonFX(Constants.kLeftClimbCanID, "Canivore");
  TalonFX m_RightClimb = new TalonFX(Constants.kRightClimbCanID, "Canivore");
  
  int integerer = 1;

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setClimberDesiredPosition(int desiredPosition) {
    
  }

  public int getClimberPosition() {
    
    return 1;
  }

  public boolean climbComplete() {
    return false;
  }

  public boolean climberInPosition (int desiredPosition) {
    if (m_LeftClimb.getPosition().getValue() > (desiredPosition-Constants.kClimberTolerancePos) && (m_LeftClimb.getPosition().getValue() < (desiredPosition+Constants.kClimberTolerancePos)))
    {
      return true; 
    } else
    {
      return false;
    }
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean climberCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
