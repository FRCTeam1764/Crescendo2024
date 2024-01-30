// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// :)))))))
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
   public LazyTalonFX flyWheel1;
   public LazyTalonFX flyWheel2;
   public LazyTalonFX holderRoller;
   private DigitalInput breakBeamHolder;

  public Shooter() {
   
    flyWheel1 = new LazyTalonFX(Constants.FLYWHEEL_MOTOR1.id, Constants.FLYWHEEL_MOTOR1.busName);
    flyWheel2 = new LazyTalonFX(Constants.FLYWHEEL_MOTOR2.id, Constants.FLYWHEEL_MOTOR2.busName);
    holderRoller = new LazyTalonFX(Constants.HOLDER_MOTOR.id, Constants.HOLDER_MOTOR.busName);
    // breakBeamHolder = new DigitalInput(Constants.HOLDER_BREAK_BEAM);

  }
  public void shooterOn() {
    flyWheel1.set(1);
    flyWheel2.set(1);
  }
  public void shooterOff() {
    flyWheel1.set(0);
    flyWheel2.set(0);
  }
  public void roller(double speed){
    double newspeed = speed;
        if (!breakBeamHolder.get()) {
          newspeed =0;
        }
    holderRoller.set(newspeed);
  }
  public void rollerOff(){
    holderRoller.set(0);
  }
  /* public void roller() {
    if (!breakBeamHolder.get()) {
    holderRoller.set(0);
    } else {
      holderRoller.set(0.5);
    }
  }
  **/

  public boolean isInputBroken(){
    return !breakBeamHolder.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
