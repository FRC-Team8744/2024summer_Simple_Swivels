// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IMUConstants;

// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

public class Multi_IMU extends SubsystemBase {

  // The imu sensors
  private PigeonIMU m_imu_pigeon1;
  private AHRS m_imu_navX2_micro;
  private Pigeon2 m_imu_pigeon2;

  // Robot type
  private String m_whoami;
  private int m_imuSelected;
  private boolean m_pigeon1_Enable = false;
  private boolean m_pigeon2_Enable = false;
  private boolean m_navx2_Enable = false;

  public Multi_IMU() {
    m_whoami = Preferences.getString("RobotName", "Undefined");
    switch (m_whoami) {
      case "Swivels":
          m_imuSelected = IMUConstants.PIGEON2;
          m_pigeon1_Enable = true;
        break;
    
      case "NoNo":
          m_imuSelected = IMUConstants.PIGEON2;
          m_pigeon2_Enable = true;
        break;
    
      default:
          m_imuSelected = IMUConstants.PIGEON2;
          m_pigeon1_Enable = true;
        break;
    }

    // Attempt to communicate with new sensor (may not exist)
    if (m_pigeon1_Enable) m_imu_pigeon1 = new PigeonIMU(IMUConstants.PIGEON1_kIMU_CAN_ID);

    if (m_navx2_Enable) {
      try {
        m_imu_navX2_micro = new AHRS(SerialPort.Port.kUSB);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      }
    }

    if (m_pigeon2_Enable) {
      m_imu_pigeon2 = new Pigeon2(IMUConstants.PIGEON2_kIMU_CAN_ID, "rio");

      // Configure Pigeon2
      var toApply = new Pigeon2Configuration();  // User can change the configs if they want, or leave it empty for factory-default
      // Set configuration with "toApply." here
      /* For instance:
      toApply.MountPose.MountPoseYaw = 0;
      toApply.MountPose.MountPosePitch = 0;
      toApply.MountPose.MountPoseRoll = 90;
      */
      m_imu_pigeon2.getConfigurator().apply(toApply);

      // Speed up signals to an appropriate rate
      m_imu_pigeon2.getYaw().setUpdateFrequency(100);
      m_imu_pigeon2.getGravityVectorZ().setUpdateFrequency(100);
    }
  }

  @Override
  public void periodic() {
    if (m_pigeon1_Enable && (IMUConstants.DEBUG_IMU >= IMUConstants.DEBUG_ALL)) {
      SmartDashboard.putNumber("Pigeon1 GyroZ", m_imu_pigeon1.getYaw());
      SmartDashboard.putNumber("Pigeon1 Absolute GyroZ", m_imu_pigeon1.getAbsoluteCompassHeading());
      SmartDashboard.putNumber("Pigeon1 Fused GyroZ", m_imu_pigeon1.getFusedHeading());
    }

    if (m_navx2_Enable && (IMUConstants.DEBUG_IMU >= IMUConstants.DEBUG_ALL)) {
      SmartDashboard.putNumber("NavX GyroZ", m_imu_navX2_micro.getYaw());
    }

    if (m_pigeon2_Enable && (IMUConstants.DEBUG_IMU >= IMUConstants.DEBUG_ALL)) {
      SmartDashboard.putNumber("Pigeon2 GyroZ", m_imu_pigeon2.getYaw().getValueAsDouble());
    }
  }

  /**
   * Zeros the heading of the robot.
   */
  public void zeroHeading() {
    if (m_pigeon1_Enable) {
      m_imu_pigeon1.setYaw(0);
    }

    if (IMUConstants.NAVX2_MICRO_ENABLE) {
      m_imu_navX2_micro.zeroYaw();
      m_imu_navX2_micro.reset();
    }

    if (m_pigeon2_Enable) {
      m_imu_pigeon2.setYaw(0);
    }
  }

  /**
   * Returns the heading of the robot as Rotation2D
   *
   * @return the robot's heading as Rotation2D
   */
  public Rotation2d getHeading() {
    switch (m_imuSelected){
      case IMUConstants.PIGEON1:
        if (m_pigeon1_Enable) {
          return Rotation2d.fromDegrees(m_imu_pigeon1.getYaw());
        } else return Rotation2d.fromDegrees(0.0);
      case IMUConstants.NAVX2_MICRO:
        if (m_navx2_Enable) {
          return Rotation2d.fromDegrees(m_imu_navX2_micro.getYaw());
        } else return Rotation2d.fromDegrees(0.0);
      case IMUConstants.PIGEON2:
        if (m_pigeon2_Enable) {
          return Rotation2d.fromDegrees(m_imu_pigeon2.getYaw().getValueAsDouble());
        } else return Rotation2d.fromDegrees(0.0);
      default:
        return Rotation2d.fromDegrees(0.0);
    }
  }

  /**
   * Returns human readable heading of the robot.
   *
   * @return The robot's current heading in degrees.
   */
  public double getHeadingDegrees() {
    switch (m_imuSelected){
      case IMUConstants.PIGEON1:
        if (m_pigeon1_Enable) {
          return m_imu_pigeon1.getYaw();
        } else return 9999999.9;
      case IMUConstants.NAVX2_MICRO:
        if (m_navx2_Enable) {
          return m_imu_navX2_micro.getYaw();
        } else return 9999999.9;
      case IMUConstants.PIGEON2:
        if (m_pigeon2_Enable) {
          return m_imu_pigeon2.getYaw().getValueAsDouble();
        } else return 9999999.9;
      default:
        return 9999999.9;
    }
  }

}
