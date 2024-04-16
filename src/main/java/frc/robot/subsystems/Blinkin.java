// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin. */
  Double color;
  //preset colors:
  //-0.41  - blue ocean wave
  //-.99 - rainbow!!
  //.87 - blue 
  //-.09
  

double[] colors = { //https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  -0.41,
  -0.95,
  -0.75,
  -0.65,
  -0.29,
  -0.15,
  -0.97,
  -0.77,
  -0.67,
  -0.53,
  -0.43
};

int index = 0;



  Spark blinkin  = new Spark(Constants.BLINKIN_SPARKPORT);
  public Blinkin() {
    color = -0.41;
  }

  public void setColor(double color){
    this.color = color;
  }


public void setColorLoop(){
  color = colors[index];
  index = index+1;

  if(index >= colors.length-1){
    index = 0;
  }
}

  @Override
  public void periodic() {
    blinkin.set(color);
    // This method will be called once per scheduler run
  }
}
