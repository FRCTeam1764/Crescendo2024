// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public CANSparkMax intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
  }

  public void intakeOn() {
    intakeMotor.set(1); // edit speed?
  }

  public void intakeOff() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
