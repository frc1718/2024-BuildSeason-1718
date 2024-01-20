// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NoteSensors;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public ShooterSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shoot() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          /* one-time action goes here */
          /* Psuedo CODE */
          // Verify front intake is clear
          // this.setArmPosition(Constants.kArmHighPos);
          // this.setShooterSpeed(Constants.shooterHighSpeed);
          // if (this.getWheelSpeed().and(this.getKickUpSpeed()) > atSpeed){
          //  this.fire();
          //}
        });
  }

  public Command home() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          /* Psuedo CODE */
          // Verify front intake is clear
          // this.setArmPosition(home);
          // this.setShooterSpeed(idle);
        });
  }

  public Command debug(NoteSensors noteSense){
    return runOnce(() -> {System.out.print(noteSense.getNotePresentIntakeString());});
  };
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
