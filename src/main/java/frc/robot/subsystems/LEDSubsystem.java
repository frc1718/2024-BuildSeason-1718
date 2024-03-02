// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/**
 * The LED subsystem controls the brightness of the LED string on the robot.
 * <p>It's an orange LED string.
 */
public class LEDSubsystem extends SubsystemBase {
  // Create motor controllers here

  PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  /**
   * Constructs an instance of the LED subsystem.
   */
  public LEDSubsystem() {
    
  }

  public void LEDON() {
    m_powerDistribution.setSwitchableChannel(true);
 
  }
  public void LEDOFF() {
    m_powerDistribution.setSwitchableChannel(false);
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {

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
