// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// The shooter intake subsystem is the portion of the shooter that is NOT responsible for ejecting notes at high velocities.
public class ShooterIntakeSubsystem extends SubsystemBase {

  //Open Servo
  //Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  //Open Motors
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");

  private final VelocityVoltage ShooterIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  // Constructs an instance of the shooter intake subsystem.

  public ShooterIntakeSubsystem() {
  //=======================================shouldn't we be doing something with velocity voltage motor here to set it up?
  this.configureShooterIntakeSpin(m_ShooterIntakeSpin);
  }
  
  // Start of sensor related methods
  /**
   * Check if a note is in front of the intake beam break.
   * @return Whether the intake beam break detects a note.
   * True or false.
   */
  
  /**
   * Check if a note is in front of the shooter beam break.
   * @return Whether the shooter beam break detects a note.
   * True or false.
   */
  // End of sensor related methods

  /**
   * Check if a note is in front of either the intake beam break or the shooter beam break.
   * @return Whether either beam break detects a note.
   * True or false.
   */

  /**
   * Sets the speed of the shooter intake motor.
   * @param speed The desired speed of the shooter intake, in rotations per second.
   */
  public void setShooterIntakeSpeed(double speed) {
    if (Constants.kMotorEnableShooterIntakeSpin == 1){
      System.out.println("ShooterIntakeSubsystem: setShooterIntakeSpeed");
      m_ShooterIntakeSpin.setControl(ShooterIntakeVelocity.withVelocity(speed));
    }
  }

  /**
   * Extends the intake hinge servo to allow the intake to pivot.
   */
  public void Release() {
    System.out.println("ShooterIntakeSubsystem: setShooterPivotPosition");
    //intakeHinge.set(desiredPosition);
  }

  public void configureShooterIntakeSpin(TalonFX shooterIntakeSpin) {
    TalonFXConfiguration shooterIntakeSpinVelocityConfig = new TalonFXConfiguration();
    shooterIntakeSpinVelocityConfig.Slot0.kP = Constants.kShooterIntakeSpinProportional; // An error of 1 rotation per second results in 2V output
    shooterIntakeSpinVelocityConfig.Slot0.kI = Constants.kShooterIntakeSpinIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    shooterIntakeSpinVelocityConfig.Slot0.kD = Constants.kShooterIntakeSpinDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    shooterIntakeSpinVelocityConfig.Slot0.kV = Constants.kShooterIntakeSpinVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second Peak output of 8 volts
    shooterIntakeSpinVelocityConfig.Voltage.PeakForwardVoltage = Constants.kShooterIntakeSpinMaxForwardVoltage;
    shooterIntakeSpinVelocityConfig.Voltage.PeakReverseVoltage = Constants.kShooterIntakeSpinMaxReverseVoltage;
    shooterIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterIntakeSpinSupplyCurrentLimit;
    shooterIntakeSpinVelocityConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooterIntakeSpinVoltageClosedLoopRampPeriod;
    shooterIntakeSpinVelocityConfig.MotorOutput.Inverted = Constants.kShooterIntakeSpinDirection;
    shooterIntakeSpinVelocityConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    StatusCode shooterIntakeSpinStatus = StatusCode.StatusCodeNotInitialized;

    for(int i = 0; i < 5; ++i) {
      shooterIntakeSpinStatus = shooterIntakeSpin.getConfigurator().apply(shooterIntakeSpinVelocityConfig);
      if (shooterIntakeSpinStatus.isOK()) break;
    }
    if (!shooterIntakeSpinStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterIntakeSpinStatus.toString());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder){
    
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
