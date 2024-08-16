// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax rightShooterSparkMax = new CANSparkMax(MechanismConstants.kRightShooterPort, MotorType.kBrushless);
  private CANSparkMax leftShooterSparkMax = new CANSparkMax(MechanismConstants.kLeftShooterPort, MotorType.kBrushless);

  private RelativeEncoder rightShooterEncoder = rightShooterSparkMax.getEncoder(); 
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

    // m_pidController = rightShooterSparkMax.getPIDController();

    // Encoder object created to display position values
    // m_encoder = m_motor.getEncoder();

    // set PID coefficients
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // leftShooterSparkMax.setInverted(false);

    leftShooterSparkMax.follow(rightShooterSparkMax, true);

    rightShooterSparkMax.setSmartCurrentLimit(40);
    leftShooterSparkMax.setSmartCurrentLimit(40);
  }

  public void shootShooter(double speed){
    rightShooterSparkMax.set(speed); 
    // leftShooterSparkMax.set(speed / 2);
    // m_pidController.setReference(speed*maxRPM, CANSparkMax.ControlType.kVelocity);
    // leftShooterSparkMax.set(speed);
  }

  public void ampShootShooter(double speed) {
    rightShooterSparkMax.set(speed);
    // leftShooterSparkMax.set(speed / 2);
  }

  public void reverseShooter(double speed) {
    rightShooterSparkMax.set(-speed);
    // leftShooterSparkMax.set(-speed);
  }

  public void motorOff() {
    rightShooterSparkMax.stopMotor();
    // leftShooterSparkMax.stopMotor();
  }

  public boolean isAtSpeed() {
    if (rightShooterEncoder.getVelocity() >= 5000 * Constants.MechanismConstants.shooterSpeed) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isAtAmpSpeed() {
    if (rightShooterEncoder.getVelocity() >= 5000 * Constants.MechanismConstants.ampShooterSpeed) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Shooter RPM", rightShooterEncoder.getVelocity());
  }
}
