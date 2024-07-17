// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax frontIntakeSparkMax = new CANSparkMax(MechanismConstants.kFrontIntakePort, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {}

  public void moveIntake(double speed) {
    frontIntakeSparkMax.set(-speed); 
  }
 
  public void reverseIntake(double speed) {
    frontIntakeSparkMax.set(speed); 
  }

  public void motorOff() {
    frontIntakeSparkMax.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
