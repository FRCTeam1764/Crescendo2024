// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  /** Creates a new Music. */
  Orchestra theMUSIC = new Orchestra();
  public Music() {

  }


  public void AddDevice(TalonFX talon){
theMUSIC.addInstrument(talon);
  }
  


  public void PlayMusic(String song){
theMUSIC.loadMusic(song);
theMUSIC.play();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
