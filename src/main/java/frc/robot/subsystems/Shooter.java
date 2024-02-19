// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// :)))))))
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    setupMotors();

    flyWheel1.setInverted(true);
    flyWheel1.setNeutralMode(NeutralModeValue.Coast);
    flyWheel2.setNeutralMode(NeutralModeValue.Coast);

    holderRoller.setInverted(true);
    holderRoller.setNeutralMode(NeutralModeValue.Coast);

    breakBeamHolder = new DigitalInput(Constants.HOLDER_BREAK_BEAM);

  }

  public void setupMotors() {
    TalonFXConfiguration config1 = new TalonFXConfiguration();
    TalonFXConfiguration config2 = new TalonFXConfiguration();
    TalonFXConfiguration configRoller = new TalonFXConfiguration();

    flyWheel1.getConfigurator().apply(config1);
    flyWheel2.getConfigurator().apply(config2);

    holderRoller.getConfigurator().apply(configRoller);
  }

  public void shooterOn() {
    flyWheel1.set(1);
    flyWheel2.set(1);
  }

  public void shooterAmp() {
    flyWheel1.set(0.5);
    flyWheel1.set(0.5);
  }

  public void shooterOff() {
    flyWheel1.set(0);
    flyWheel2.set(0);
  }

  public void roller(double speed) {
    int negative = 1;
    if (speed > 0) {
      negative = 1;
    } else {
      negative = -1;
    }
    double newspeed = speed;

    // if (!breakBeamHolder.get()) {
    //   newspeed = negative * .05; // stall
    // }
    holderRoller.set(newspeed);
  }

  public void rollerOff() {
    holderRoller.set(0);
  }

  public boolean RollerBreakBeamBroken() {
    return !breakBeamHolder.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("RollerBreakBeam", RollerBreakBeamBroken());
    // This method will be called once per scheduler run
  }
}
