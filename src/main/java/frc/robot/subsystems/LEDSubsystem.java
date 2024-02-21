// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The LED subsystem controls the brightness of the LED string on the robot.
 * <p>It's an orange LED string.
 */
public class LEDSubsystem extends SubsystemBase {
  // Create motor controllers here
  TalonSRX SignalLight = new TalonSRX(Constants.kSignalLightCanID);

  /**
   * Constructs an instance of the LED subsystem.
   */
  public LEDSubsystem() {
    
  }

  /**
   * Sets the PWM intensity of the LED string.
   * The value is constrained so it cannot be less than 0, or greater than 1.
   * @param intensity The desired PWM duty cycle for the LED brightness.
   * From 0.0 to 1.0.
   */
  public void SetLightIntensity(double intensity) {
    if (intensity < 0) {
      intensity = 0;
    } else if (intensity > 1) {
      intensity = 1;
    }

    SignalLight.set(ControlMode.PercentOutput, intensity);
  }

  /**
   * Get the current PWM duty cycle of the LED string.
   * @return The current PWM duty cycle of the LED string.
   * Duty cycle is represented as a number from 0.0 to 1.0.
   */
  public double GetLightIntensity() {
    return SignalLight.getMotorOutputPercent();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LEDSubsystem");
    builder.addDoubleProperty("LED Intensity", this::GetLightIntensity, null); 
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
