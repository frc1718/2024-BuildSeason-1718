// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The shooter subsystem takes a note that was picked up off the ground, and puts it somewhere that isn't the ground.
 * <p><i>Maybe.</i>
 */
public class CornerRollerSubsystem extends SubsystemBase {
 
  //Make variables
  public String m_shooterMode = "DoNothing";
  public boolean m_readyToShoot = false;
  public double m_desiredPosition = 0;
  public double m_desiredSpeed = 0;

  
  TalonFX m_SpinRightRoller = new TalonFX(Constants.kSpinRightRollerCanID, "Canivore");
  TalonFX m_SpinLeftRoller = new TalonFX(Constants.kSpinLeftRollerCanID, "Canivore"); 

  //Open CANcoder 
  private final VelocityVoltage LeftRollerVelocityRequest = new VelocityVoltage(0.0, 0.0, true, 0,0, false, false, false);
  private final VelocityVoltage RightRollerVelocityRequest = new VelocityVoltage(0.0, 0.0, true, 0,0, false, false, false);
  
  /**
   * Constructs an instance of the shooter subsystem.
   * The motor and sensor configuration is done here.
   */
  public CornerRollerSubsystem() {
    //Configure Motors in seperate methods for clarity
    this.configureSpinRightRoller(m_SpinRightRoller);
    this.configureSpinLeftRoller(m_SpinLeftRoller);
  }

  // Start of motor get methods
  public double getRollerSpeed() {
    if (Constants.kPrintSubsystemCornerRoller){System.out.println("CornerRollerSubsystem: getSpinSpeed: "+m_SpinLeftRoller.getVelocity().getValueAsDouble());}
    return m_SpinLeftRoller.getVelocity().getValueAsDouble();
  }

  // Start of motor set methods

  /**
   * Sets the speed of the shooter motors.
   * The left shooter motor is set to the input speed.
   * The right shooter motor is set to 90% of the input speed.
   * @param spinSpeed The desired speed of the shooter motors, in rotations per second.
   */
  public void setSpinSpeed(double spinSpeed) {
    if (Constants.kPrintSubsystemCornerRoller){System.out.println("CornerRollerSubsystem: setSpinSpeed");}
    if (Constants.kMotorEnableLeftRollerSpin == 1){
      m_SpinLeftRoller.setControl(RightRollerVelocityRequest.withVelocity(spinSpeed));
    }
    if (Constants.kMotorEnableRightRollerSpin == 1){
      m_SpinRightRoller.setControl(LeftRollerVelocityRequest.withVelocity((spinSpeed)));
    }

    m_desiredSpeed = spinSpeed;
  }


    /**
   * Open Motors
   * 
   */

  public void configureSpinRightRoller(TalonFX spinRightRoller){
    TalonFXConfiguration RightRollerMotorsConfig = new TalonFXConfiguration();

    RightRollerMotorsConfig.Slot0.kP = Constants.kRightRollerProportional; // An error of 1 rotation per second results in 2V output
    RightRollerMotorsConfig.Slot0.kI = Constants.kRightRollerIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    RightRollerMotorsConfig.Slot0.kD = Constants.kRightRollerDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    RightRollerMotorsConfig.Slot0.kV = Constants.kRightRollerVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  
    RightRollerMotorsConfig.MotorOutput.Inverted = Constants.kRightShooterDirection;

    // Peak output of 8 volts
    RightRollerMotorsConfig.Voltage.PeakForwardVoltage = Constants.kRightShooterMaxForwardVoltage;
    RightRollerMotorsConfig.Voltage.PeakReverseVoltage = Constants.kRightShooterMaxReverseVoltage;
    RightRollerMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    RightRollerMotorsConfig.CurrentLimits.SupplyCurrentLimit = Constants.kRightShooterSupplyCurrentLimit;
    RightRollerMotorsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kRightShooterVoltageClosedLoopRampPeriod;
    
    //Setting the config option that allows playing music on the motor during disabled.
    RightRollerMotorsConfig.Audio.AllowMusicDurDisable = true;

    StatusCode rightRollerStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      rightRollerStatus = m_SpinRightRoller.getConfigurator().apply(RightRollerMotorsConfig);
      if (rightRollerStatus.isOK()) break;
    }
    if (!rightRollerStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + rightRollerStatus.toString());
    }
  }

  public void configureSpinLeftRoller(TalonFX spinLeftRoller){
    TalonFXConfiguration LeftRollerMotorsConfig = new TalonFXConfiguration();

    LeftRollerMotorsConfig.Slot0.kP = Constants.kLeftRollerProportional; // An error of 1 rotation per second results in 2V output
    LeftRollerMotorsConfig.Slot0.kI = Constants.kLeftRollerIntegral; // An error of 1 rotation per second increases output by 0.5V every second
    LeftRollerMotorsConfig.Slot0.kD = Constants.kLeftRollerDerivative; // A change of 1 rotation per second squared results in 0.01 volts output
    LeftRollerMotorsConfig.Slot0.kV = Constants.kLeftRollerVelocityFeedFoward; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
  
    LeftRollerMotorsConfig.MotorOutput.Inverted = Constants.kLeftShooterDirection;

    // Peak output of 8 volts
    LeftRollerMotorsConfig.Voltage.PeakForwardVoltage = Constants.kLeftShooterMaxForwardVoltage;
    LeftRollerMotorsConfig.Voltage.PeakReverseVoltage = Constants.kLeftShooterMaxReverseVoltage;
    LeftRollerMotorsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    LeftRollerMotorsConfig.CurrentLimits.SupplyCurrentLimit = Constants.kLeftShooterSupplyCurrentLimit;
    LeftRollerMotorsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kLeftShooterVoltageClosedLoopRampPeriod;

    //Setting the config option that allows playing music on the motor during disabled.
    LeftRollerMotorsConfig.Audio.AllowMusicDurDisable = true;    

    StatusCode leftRollerStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      leftRollerStatus = m_SpinLeftRoller.getConfigurator().apply(LeftRollerMotorsConfig);
      if (leftRollerStatus.isOK()) break;
    }
    if (!leftRollerStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + leftRollerStatus.toString());
    }
  }

  /**
   * Add all of the motors in the shooter subsystem to the Orchestra.
   * I want the robot to sing.
   * @param robotOrchestra The Orchestra to add the motors as instruments to.
   */
  public void addToOrchestra(Orchestra robotOrchestra) {
    robotOrchestra.addInstrument(m_SpinLeftRoller);
    robotOrchestra.addInstrument(m_SpinRightRoller);
  }


  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("CornerRollerSubsystem");
    builder.addDoubleProperty("Roller Speed", this::getRollerSpeed, null);
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
