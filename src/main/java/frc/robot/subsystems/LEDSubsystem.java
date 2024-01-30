// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  // Create motor controllers here
  TalonSRX SignalLight = new TalonSRX(Constants.kSignalLightCanID);

  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {}

  public void SetLightIntensity(double intensity) {
    if (intensity < 0) {
      intensity = 0;
    } else if (intensity > 1) {
      intensity = 1;
    }

    SignalLight.set(ControlMode.PercentOutput, intensity);
  }

  public double GetLightIntensity() {
    return SignalLight.getMotorOutputPercent();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
