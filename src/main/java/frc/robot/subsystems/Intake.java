// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public double intakeSpeed = 0.7;
  private CANSparkMax frontIntakeSparkMax = new CANSparkMax(Constants.kIntakePort, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }
  
  public void ringGrab(double speed) {
    frontIntakeSparkMax.set(-speed); 
  }
 
    public void ringRelease(double speed) {
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