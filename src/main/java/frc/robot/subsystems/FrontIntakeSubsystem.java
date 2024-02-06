// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class FrontIntakeSubsystem extends SubsystemBase {

  TalonFX m_frontIntakeRotate = new TalonFX(Constants.kFrontIntakeRotateCanID, "Canivore");
  TalonFX m_FrontIntakeSpin = new TalonFX(Constants.kFrontIntakeSpinCanID, "Canivore");

  private final MotionMagicVoltage FrontIntakePosition = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);
  private final VelocityVoltage FrontIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  public FrontIntakeSubsystem() {
    TalonFXConfiguration FrontIntakeRotateConfig = new TalonFXConfiguration();
    MotionMagicConfigs FrontIntakeRotateMotionMagic = FrontIntakeRotateConfig.MotionMagic;
    FrontIntakeRotateMotionMagic.MotionMagicCruiseVelocity = Constants.kShooterArmRotateMotionMagicCruiseVelocity;
    FrontIntakeRotateMotionMagic.MotionMagicAcceleration = Constants.kShooterArmRotateMotionMagicAcceleration;
    // Take approximately 0.2 seconds to reach max accel 
    FrontIntakeRotateMotionMagic.MotionMagicJerk = Constants.kShooterArmRotateMotionMagicJerk;

    Slot0Configs slot0 = FrontIntakeRotateConfig.Slot0;
    slot0.kP = Constants.kFrontIntakeRotateProportional;
    slot0.kI = Constants.kFrontIntakeRotateIntegral;
    slot0.kD = Constants.kFrontIntakeRotateDerivative;
    slot0.kV = Constants.kFrontIntakeRotateVelocityFeedFoward;
    slot0.kS = Constants.kFrontIntakeRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    StatusCode FrontIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      FrontIntakeRotateStatus = m_frontIntakeRotate.getConfigurator().apply(FrontIntakeRotateConfig);
      if (FrontIntakeRotateStatus.isOK()) break;
    }
    if (!FrontIntakeRotateStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + FrontIntakeRotateStatus.toString());
    }


  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setFrontIntakePosition(int position) {
    /* set.intakeposition() */
    
    System.out.println("FrontIntakeSubsystem - setFrontIntakePosition");
    
  }

  public void setFrontIntakeSpeed(int speed) {
    /* set.intakeSpeed( ) */
    System.out.println("FrontIntakeSubsystem - setFrontIntakeSpeed");
  }
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  
  public Boolean getFrontIntakeInPosition(int desiredPosition) {
    //Check that the front intake is within the tolerance of the desired position.
    /* if (m_frontIntakeRotate.getPosition().getValue() > (desiredPosition-Constants.kFrontIntakeTolerancePos) && (m_frontIntakeRotate.getPosition().getValue() < (desiredPosition+Constants.kFrontIntakeTolerancePos)))
    {
      return true; 
    } else
    {
      return false;
    }
     */
    System.out.println("FrontIntakeSubsystem - getFrontIntakeInPosition");
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
