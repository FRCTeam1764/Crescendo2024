// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.Constants;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new BlinkinSubsystem. */
  Spark blinkin;
  double LEDColor;
  public BlinkinSubsystem() {
  }

  public void setLEDs(double LEDColor){
    this.LEDColor = LEDColor;
    //getLEDColor());
  }
  public double getLEDColor(){
    return LEDColor;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
