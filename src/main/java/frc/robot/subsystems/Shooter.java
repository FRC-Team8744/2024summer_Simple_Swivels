// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
    private SparkPIDController m_pidController;
  // private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /** Creates a new Shooter. */
  public Shooter() {
    // PID coefficients
    // kP = 6e-5; 
    // kI = 0;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    // m_pidController = topShooterSparkMax.getPIDController();

    // Encoder object created to display position values
    // m_encoder = m_motor.getEncoder();

    // set PID coefficients
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    bottomShooterSparkMax.follow(topShooterSparkMax);
  }

  public void shootShooter(double speed){
    topShooterSparkMax.set(speed); 
    // m_pidController.setReference(speed*maxRPM, CANSparkMax.ControlType.kVelocity);
    // bottomShooterSparkMax.set(speed);
  }
  public void motorOff() {
    topShooterSparkMax.stopMotor();
    // bottomShooterSparkMax.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
