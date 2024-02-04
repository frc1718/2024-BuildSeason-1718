// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontIntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public FrontIntakeSubsystem() {}

  //TalonFX m_frontIntakeRotate = new TalonFX(Constants.kFrontIntakeRotateCanID, "Canivore");
  //TalonFX m_FrontIntakeSpin = new TalonFX(Constants.kFrontIntakeSpinCanID, "Canivore");

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setFrontIntakePosition(int position) {
    /* set.intakeposition() */
    
    System.out.println("Subsystem: FrontIntake - setFrontIntakePosition");
    
  }

  public void setFrontIntakeSpeed(int speed) {
    // set.intakeposition
    /* set.intakeSpeed(spit) */
    System.out.println("Subsystem: FrontIntake - setFrontIntakeSpeed");
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
    System.out.println("Subsystem: FrontIntake - getFrontIntakeInPosition");
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
