// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import java.lang.constant.Constable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public double shootingSpeed = 2430;

  public PowerDistribution PDH = new PowerDistribution(14, ModuleType.kRev);

  private CANSparkMax topShooterSparkMax = new CANSparkMax(Constants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(Constants.kBottomShooterPort, MotorType.kBrushless);
  
  private final RelativeEncoder topShooterEnc = topShooterSparkMax.getEncoder();
  private final RelativeEncoder bottomShooterEnc = bottomShooterSparkMax.getEncoder();

  private final SparkPIDController topShooterPID = topShooterSparkMax.getPIDController();
  private final SparkPIDController bottomShooterPID = bottomShooterSparkMax.getPIDController();

  public Shooter() {
    topShooterSparkMax.setSmartCurrentLimit(40);
    bottomShooterSparkMax.setSmartCurrentLimit(40);

    bottomShooterSparkMax.setInverted(true);

    bottomShooterPID.setP(0.0001);
    bottomShooterPID.setI(0);
    bottomShooterPID.setD(0.001);
    bottomShooterPID.setFF(0.0002);

    topShooterPID.setP(0.0001);
    topShooterPID.setI(0);
    topShooterPID.setD(0.001);
    topShooterPID.setFF(0.0002);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel top RPM", topShooterEnc.getVelocity());
    SmartDashboard.putNumber("Flywheel bottom RPM", bottomShooterEnc.getVelocity());
  }

  public void testShoot(double speed) {
    bottomShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    topShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    topShooterSparkMax.stopMotor();
    bottomShooterSparkMax.stopMotor();
  }

  public boolean atSpeed() {
    if ((topShooterEnc.getVelocity() + bottomShooterEnc.getVelocity()) / 2 >= shootingSpeed * .95)
    {
      return true;
    }
    else {
      return false;
    }
  }
}