// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  public LazyTalonFX climberMotor1;
  public LazyTalonFX climberMotor2;
  public PIDController pidController;
  public DigitalInput upperRightLimitSwitch;
  public DigitalInput upperLeftLimitSwitch;
  public DigitalInput lowerRightLimitSwitch;
  public DigitalInput lowerLeftLimitSwitch;

  public ClimberSubsystem() {
    climberMotor1 = new LazyTalonFX(Constants.CLIMBER_MOTOR1.id, Constants.CLIMBER_MOTOR1.busName);
    climberMotor2 = new LazyTalonFX(Constants.CLIMBER_MOTOR2.id, Constants.CLIMBER_MOTOR2.busName);
  }

  // version 2.0
  public void climb1(double climberSpeed) {
    if(!lowerLeftLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMotor1.set(newClimberSpeed); // ignore warning, it'll last till next year
    }
  }

  public void climb2(double climberSpeed) {
    if(!lowerRightLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMotor1.set(newClimberSpeed); // ignore warning, it'll last till next year
    }
  }

  public void descend1(double climberSpeed) {
    if(!upperLeftLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMotor1.set(newClimberSpeed); // ignore warning, it'll last till next year
    }
  }

  public void descend2(double climberSpeed) {
    if(!upperRightLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMotor2.set(newClimberSpeed); // ignore warning, it'll last till next year
    }
  }

  // extend, doesn't need one and two so fetch from 1.0

  // version 1.0
  public void climberOff() {
    climberMotor1.set(0); // ignore warning, it'll last till next year
    climberMotor2.set(0);
  }

  public void extend() {
    climberOn(1);  // fast speed
  }

  public void climb() {
    climberOn(-0.9); // slow-er speed
  }

  public void descend() {
    climberOn(0.1); // slow speed
  }

  public void climberOn(double climberSpeed) {
    if(!upperRightLimitSwitch.get() || !upperLeftLimitSwitch.get() || !lowerRightLimitSwitch.get() || !lowerLeftLimitSwitch.get()){
      double newClimberSpeed = climberSpeed < 0 ? 0 : climberSpeed;
      climberMotor1.set(newClimberSpeed); // ignore warning, it'll last till next year
      climberMotor2.set(newClimberSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
