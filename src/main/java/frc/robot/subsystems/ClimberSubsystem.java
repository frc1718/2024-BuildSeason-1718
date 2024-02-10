// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  boolean m_preClimbActuated = false;

  /*

  //RightClimb Should be a follower of leftclimb
  TalonFX m_LeftClimb = new TalonFX(Constants.kLeftClimbCanID, "Canivore");
  TalonFX m_RightClimb = new TalonFX(Constants.kRightClimbCanID, "Canivore");

  */

  private final MotionMagicVoltage climberMove = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {

    /*
    //Start Configuring Climbers
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    MotionMagicConfigs climberMotionMagic = climberConfig.MotionMagic;
    climberMotionMagic.MotionMagicCruiseVelocity = Constants.kClimberMotionMagicCruiseVelocity; // 5 rotations per second cruise if this is 5
    climberMotionMagic.MotionMagicAcceleration = Constants.kClimberMotionMagicAcceleration; // Take approximately 0.5 seconds to reach max vel if this is 10
    // Take approximately 0.2 seconds to reach max accel 
    climberMotionMagic.MotionMagicJerk = Constants.kClimberMotionMagicJerk;

    climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.kClimberSupplyCurrentLimit;
    climberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kClimberVoltageClosedLoopRampPeriod;

    Slot0Configs slot0 = climberConfig.Slot0;
    slot0.kP = Constants.kClimberProportional;
    slot0.kI = Constants.kClimberIntegral;
    slot0.kD = Constants.kClimberDerivative;
    slot0.kV = Constants.kClimberVelocityFeedFoward;
    slot0.kS = Constants.kClimberStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      climberStatus = m_LeftClimb.getConfigurator().apply(climberConfig);
      if (climberStatus.isOK()) break;
    }
    if (!climberStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + climberStatus.toString());
    }

    m_RightClimb.setControl(new Follower(Constants.kLeftClimbCanID, true));
    //End Configuration

    */

  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setClimberDesiredPosition(int desiredPosition) {
    System.out.println("Subsystem: Climber - setClimberDesiredPosition");
    // m_LeftClimb.setControl(climberMove.withPosition(desiredPosition));
  }

  public void setPreClimbActuated(boolean preClimbActuated){
    m_preClimbActuated=preClimbActuated;
  }

  public boolean getPreClimbActuated(){
    System.out.println("Subsystem: Climber - getPreClimbActuated");
    return m_preClimbActuated;
  }

  public double getClimberPosition() {   
    System.out.println("Subsystem: Climber - getClimberPosition");
    // return m_LeftClimb.getPosition().getValueAsDouble();
    //Remove the return below this when the above code is uncommented
    return 1;
  }

  public boolean getClimberInPosition (int desiredPosition) {
    System.out.println("Subsystem: Climber - getClimberInPosition");
    
    /*
    if (m_LeftClimb.getPosition().getValue() > (desiredPosition-Constants.kClimberTolerancePos) && (m_LeftClimb.getPosition().getValue() < (desiredPosition+Constants.kClimberTolerancePos)))
    {
      return true; 
    } else
    {
      return false;
    } 
    */

    //Remove the return below this when the above code is uncommented
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
