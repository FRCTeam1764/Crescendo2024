// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// :)))))))
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
   public LazyTalonFX flyWheel;
   public LazyTalonFX flyWheel2;
   public LazyTalonFX holderRoller;
   private DigitalInput breakBeamHolder;

  public Shooter() {
   
    flyWheel = new LazyTalonFX(Constants.FLYWHEEL_MOTOR1.id, Constants.FLYWHEEL_MOTOR1.busName);
    flyWheel2 = new LazyTalonFX(Constants.FLYWHEEL_MOTOR2.id, Constants.FLYWHEEL_MOTOR2.busName);
    holderRoller = new LazyTalonFX(Constants.HOLDER_MOTOR.id, Constants.HOLDER_MOTOR.busName);

  //flyWheel.setInverted(true);
    breakBeamHolder = new DigitalInput(Constants.HOLDER_BREAK_BEAM);

  }
  public void shooterOn() {
    flyWheel.set(-1);//.18 prev
    flyWheel2.set(1);
  }
  public void shooterOff() {
    flyWheel.set(0);
    flyWheel2.set(0);
  }
  public void roller(double speed){
    holderRoller.set(speed);
  }
  /* public void roller() {
    if (!breakBeamHolder.get()) {
    holderRoller.set(0);
    } else {
      holderRoller.set(0.5);
    }
  }
  **/
  @Override
  public void periodic() {
  SmartDashboard.putBoolean("ShooterBreakbeam",breakBeamHolder.get());
    // This method will be called once per scheduler run
  }
}
