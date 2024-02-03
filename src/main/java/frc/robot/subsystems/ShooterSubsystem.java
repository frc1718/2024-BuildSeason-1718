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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements NTSendable{
 
  //Open sensors
  AnalogInput m_BeamBreakIntakeAnalog = new AnalogInput(Constants.kBeamBreakIntakeAnalog);
  AnalogInput m_BeamBreakShooterAnalog = new AnalogInput(Constants.kBeamBreakShooterAnalog);
  
  //Open Servo
  Servo intakeHinge = new Servo(Constants.kShooterIntakePivotReleasePWM);

  //Open Motors
  TalonFX m_ShooterArmRotateLeft = new TalonFX(Constants.kShooterArmRotateLeftCanID, "Canivore");
  TalonFX m_ShooterArmRotateRight = new TalonFX(Constants.kShooterArmRotateRightCanID, "Canivore");
  TalonFX m_ShooterIntakeSpin = new TalonFX(Constants.kShooterIntakeSpinCanID, "Canivore");
  TalonFX m_SpinRightShooter = new TalonFX(Constants.kSpinRightShooterCanID, "Canivore");
  TalonFX m_SpinLeftShooter = new TalonFX(Constants.kSpinLeftShooterCanID, "Canivore"); 
  
  private final VelocityVoltage ShooterVelocity = new VelocityVoltage(0.0, 0.0, true, 0,0, false, false, false);
  private final MotionMagicVoltage ShooterArmPosition = new MotionMagicVoltage(0.0, true, 0, 0, false, false, false);
  private final VelocityVoltage ShooterIntakeVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private String m_shooterMode="";

  public ShooterSubsystem() {
    TalonFXConfiguration ShooterMotorsConfig = new TalonFXConfiguration();

    ShooterMotorsConfig.Slot0.kP = Constants.kShooterProportional; // An error of 1 rotation per second results in 2V output
    ShooterMotorsConfig.Slot0.kI = Constants.kShooterIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    ShooterMotorsConfig.Slot0.kD = Constants.kShooterDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    ShooterMotorsConfig.Slot0.kV = Constants.kShooterVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    ShooterMotorsConfig.Voltage.PeakForwardVoltage = Constants.kShooterMaxForwardVoltage;
    ShooterMotorsConfig.Voltage.PeakReverseVoltage = Constants.kShooterMaxReverseVoltage;

    StatusCode leftShooterStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftShooterStatus = m_SpinLeftShooter.getConfigurator().apply(ShooterMotorsConfig);
      if (leftShooterStatus.isOK()) break;
    }
    if (!leftShooterStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftShooterStatus.toString());
    }

    StatusCode rightShooterStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightShooterStatus = m_SpinRightShooter.getConfigurator().apply(ShooterMotorsConfig);
      if (rightShooterStatus.isOK()) break;
    }
    if (!rightShooterStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightShooterStatus.toString());
    }
  

    TalonFXConfiguration ShooterArmRotateConfigs = new TalonFXConfiguration();
    MotionMagicConfigs ShooterArmRotateMotionMagic = ShooterArmRotateConfigs.MotionMagic;
    ShooterArmRotateMotionMagic.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    ShooterArmRotateMotionMagic.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    ShooterArmRotateMotionMagic.MotionMagicJerk = 50;

    Slot0Configs slot0 = ShooterArmRotateConfigs.Slot0;
    slot0.kP = Constants.kShooterArmRotateProportional;
    slot0.kI = Constants.kShooterArmRotateIntegral;
    slot0.kD = Constants.kShooterArmRotateDerivative;
    slot0.kV = Constants.kShooterArmRotateVelocityFeedFoward;
    slot0.kS = Constants.kShooterArmRotateStaticFeedFoward; // The value of s is approximately the number of volts needed to get the mechanism moving

    StatusCode shooterArmStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      shooterArmStatus = m_ShooterArmRotateLeft.getConfigurator().apply(ShooterArmRotateConfigs);
      if (shooterArmStatus.isOK()) break;
    }
    if (!shooterArmStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterArmStatus.toString());
    }
    m_ShooterArmRotateRight.setControl(new Follower(Constants.kShooterArmRotateLeftCanID, true));
  }
  
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
  
  // Start of sensor related methods
  public boolean getNotePresentIntake() {  
    return (m_BeamBreakIntakeAnalog.getVoltage() >= Constants.kIntakeBeamBreakCrossover);
  }
   
  public boolean getNotePresentShooter() {  
    return (m_BeamBreakShooterAnalog.getVoltage() >= Constants.kShooterBeamBreakCrossover);
  }
  // End of sensor related methods

  // Start of motor set methods

  public void setShooterMode(String shooterMode){
    m_shooterMode=shooterMode;
  }

  public String getShooterMode(){
  return m_shooterMode;
  }

  public void setShooterIntakeSpeed(double speed) {
    m_ShooterIntakeSpin.setControl(ShooterIntakeVelocity.withVelocity(speed));
  }

  public void setShooterSpeed(double shootSpeed) {
    m_SpinLeftShooter.setControl(ShooterVelocity.withVelocity(shootSpeed));
    m_SpinRightShooter.setControl(ShooterVelocity.withVelocity((shootSpeed*0.9)));
  }

  public void setShooterArmPosition(int position) {
    m_ShooterArmRotateLeft.setControl(ShooterArmPosition.withPosition(position));
  }

  public void setShooterIntakePivotPosition(double desiredPosition) {
    intakeHinge.set(desiredPosition);
  }
  //End of motor set methods

  //Start of motor get methods
  public double getShooterSpeed() {
    return m_SpinLeftShooter.getPosition().getValue();
  }

  public double getShooterArmPosition() {
    return m_ShooterArmRotateLeft.getPosition().getValue();
  }

  public boolean getShooterUpToSpeed(int desiredSpeed) {
    if ((desiredSpeed - Constants.kShooterSpeedTolerance) >= getShooterSpeed() && getShooterSpeed() <= (desiredSpeed + Constants.kShooterSpeedTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getShooterArmInPosition(int desiredPosition) {
      if (m_ShooterArmRotateLeft.getPosition().getValue() > (desiredPosition-Constants.kShooterArmTolerancePos) && (m_ShooterArmRotateLeft.getPosition().getValue() < (desiredPosition+Constants.kShooterArmTolerancePos)))
      {
        return true; 
      } else
      {
        return false;
      }
      
    }
    //End of motor get methods

  @Override
  public void initSendable(NTSendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");
    builder.addDoubleProperty("Current Voltage", () -> {return m_BeamBreakShooterAnalog.getVoltage();}, null); 
    builder.addBooleanProperty("Beam Broken", () -> {return (m_BeamBreakShooterAnalog.getVoltage() > Constants.kShooterBeamBreakCrossover);}, null);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
