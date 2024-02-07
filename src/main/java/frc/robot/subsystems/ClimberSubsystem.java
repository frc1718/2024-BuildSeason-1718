// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  //Open hardware
  // Servo m_IntakePivotRelease = new Servo(Constants.kIntakePivotReleasePWM);
  // TalonFX m_LeftClimb = new TalonFX(Constants.kLeftClimbCanID, "Canivore");
  // TalonFX m_RightClimb = new TalonFX(Constants.kRightClimbCanID, "Canivore");
  
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setDesiredPosition(double DesiredPosition) {
    
  }

  public void setPower(double power) {
    
  }

  public void IntakePivotRelease() {
    
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
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Climber Position", () -> {return 0; /*this::getClimberPosition*/}, null);
    builder.addBooleanProperty("Climber Pre-Actuated?", () -> {return false; /*this::getPreClimbActuated*/}, null);
    builder.addBooleanProperty("Climber in Position?", () -> {return false; /*this::getClimberInPosition*/}, null);  
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
