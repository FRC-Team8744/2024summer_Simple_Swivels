// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax indexerSparkMax = new CANSparkMax(MechanismConstants.kIndexShooterPort, MotorType.kBrushless);
  public Indexer() {}

  public void moveIndexer(double speed){
    indexerSparkMax.set(speed);
  }
  public void reverseIndexer(double speed){
    indexerSparkMax.set(-speed);
  }

  public void motorOff(){
    indexerSparkMax.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
