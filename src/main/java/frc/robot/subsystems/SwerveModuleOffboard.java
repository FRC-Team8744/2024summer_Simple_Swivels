// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController; 
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleOffboard {
  // Drive motor
  private final CANSparkMax m_driveMotor;
  private final RelativeEncoder m_driveEncoder;
  private final SparkPIDController m_drivePID;

  // Turning motor
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_turningEncoder;
  private final SparkPIDController m_turningPID;
  
  // Swerve module absolute encoder (wheel angle)
  private final CANcoder m_canCoder;
  private final double m_canCoderOffsetDegrees;
  private final boolean m_invertDriveMotor;
  //private double lastAngle;

  SwerveModuleState state;
  private int DisplayCount = 0;

  /**
   * SwerveModuleOffboard - A SparkMax-based swerve module with canCoder wheel angle measurement
   *
   * @param driveMotorID The CAN ID of the drive motor.
   * @param turningMotorID The CAN ID of the turning motor.
   * @param magEncoderID The CAN ID of the magnetic encoder.
   * @param magEncoderOffsetDegrees The absolute offset of the magnetic encoder.
   */
  public SwerveModuleOffboard(int driveMotorID, int turningMotorID, int magEncoderID, boolean invertDrive,
      double magEncoderOffsetDegrees) {
    // Create drive motor objects
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePID = m_driveMotor.getPIDController();
   

    // Create turning motor objects
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningPID = m_turningMotor.getPIDController();

    // Create steering encoder objects (high resolution encoder)
    m_canCoder = new CANcoder(magEncoderID, "rio");
    m_canCoderOffsetDegrees = magEncoderOffsetDegrees;
    m_invertDriveMotor = invertDrive;

    configureDevices();
    //lastAngle = getState().angle.getRadians();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (SwerveConstants.DISABLE_ANGLE_OPTIMIZER) {
      // Only enable when correcting wheel offsets!
      state = desiredState;
      if (DisplayCount++ > 40) {
        System.out.println("CAUTION: Steering optimizer is disabled!");
        DisplayCount = 0;
      }
    } else {
      // Optimize the reference state to avoid spinning further than 90 degrees
      state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));
    }

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(new Rotation2d(m_turningEncoder.getPosition())).getCos();

    // Set the PID reference states
    m_drivePID.setReference(state.speedMetersPerSecond, (ConstantsOffboard.DRIVE_MOTOR_PROFILED_MODE) ? CANSparkMax.ControlType.kSmartVelocity : CANSparkMax.ControlType.kVelocity);
    m_turningPID.setReference(state.angle.getRadians(), (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) ? CANSparkMax.ControlType.kSmartMotion : CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    double velocity = m_driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(m_turningEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getCanCoder() {
    var posVal = m_canCoder.getPosition();
    if(posVal.getStatus().isOK()) {
        double val = posVal.getValue();
        return val * 360.0;
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0.0;
    }
  }

  /**
   * Returns the SparkMax internal encoder's measured turn angle in degrees.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(m_turningEncoder.getPosition());
  }

  /**
   * Returns the SparkMax internal encoder's measured position in meters.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(m_turningEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }
  
  public double getCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

   public double getTurnCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  private void configureDevices() {
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    toApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    m_canCoder.getPosition().setUpdateFrequency(100);
    m_canCoder.getVelocity().setUpdateFrequency(100);

    // Drive motor configuration.
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setSmartCurrentLimit(ConstantsOffboard.DRIVE_CURRENT_LIMIT);
     m_driveMotor.setInverted(m_invertDriveMotor);
 
    if (ConstantsOffboard.DRIVE_MOTOR_PROFILED_MODE) {
      m_drivePID.setP(ConstantsOffboard.DRIVE_KP_PROFILED);
      m_drivePID.setI(ConstantsOffboard.DRIVE_KI_PROFILED);
      m_drivePID.setD(ConstantsOffboard.DRIVE_KD_PROFILED);
      m_drivePID.setFF(ConstantsOffboard.DRIVE_KF_PROFILED);
    } else {
      m_drivePID.setP(ConstantsOffboard.DRIVE_KP);
      m_drivePID.setI(ConstantsOffboard.DRIVE_KI);
      m_drivePID.setD(ConstantsOffboard.DRIVE_KD);
      m_drivePID.setFF(ConstantsOffboard.DRIVE_KF);
    }

    m_drivePID.setSmartMotionMaxVelocity(ConstantsOffboard.DRIVE_MAX_VEL_PROFILED, 0);
    m_drivePID.setSmartMotionMinOutputVelocity(0.0, 0);
    m_drivePID.setSmartMotionMaxAccel(ConstantsOffboard.DRIVE_MAX_ACC_PROFILED, 0);
    m_drivePID.setSmartMotionAllowedClosedLoopError(ConstantsOffboard.DRIVE_MAX_ERR_PROFILED, 0);
    
    m_driveEncoder.setPositionConversionFactor(ConstantsOffboard.DRIVE_ROTATIONS_TO_METERS);
    m_driveEncoder.setVelocityConversionFactor(ConstantsOffboard.DRIVE_RPM_TO_METERS_PER_SECOND);
    m_driveEncoder.setPosition(0);

    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setInverted(ConstantsOffboard.ANGLE_MOTOR_INVERSION);
    m_turningMotor.setSmartCurrentLimit(ConstantsOffboard.ANGLE_CURRENT_LIMIT);

    if (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) {
      m_turningPID.setP(ConstantsOffboard.ANGLE_KP_PROFILED);
      m_turningPID.setI(ConstantsOffboard.ANGLE_KI_PROFILED);
      m_turningPID.setD(ConstantsOffboard.ANGLE_KD_PROFILED);
      m_turningPID.setFF(ConstantsOffboard.ANGLE_KF_PROFILED);
    } else {
      m_turningPID.setP(ConstantsOffboard.ANGLE_KP);
      m_turningPID.setI(ConstantsOffboard.ANGLE_KI);
      m_turningPID.setD(ConstantsOffboard.ANGLE_KD);
      m_turningPID.setFF(ConstantsOffboard.ANGLE_KF);
    }

    m_turningPID.setPositionPIDWrappingEnabled(true);
    m_turningPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    m_turningPID.setPositionPIDWrappingMinInput(0);

    m_turningPID.setSmartMotionMaxVelocity(ConstantsOffboard.ANGLE_MAX_VEL_PROFILED, 0);
    m_turningPID.setSmartMotionMinOutputVelocity(0.0, 0);
    m_turningPID.setSmartMotionMaxAccel(ConstantsOffboard.ANGLE_MAX_ACC_PROFILED, 0);
    m_turningPID.setSmartMotionAllowedClosedLoopError(ConstantsOffboard.ANGLE_MAX_ERR_PROFILED, 0);
    
    m_turningEncoder.setPositionConversionFactor(ConstantsOffboard.ANGLE_ROTATIONS_TO_RADIANS);
    m_turningEncoder.setVelocityConversionFactor(ConstantsOffboard.ANGLE_RPM_TO_RADIANS_PER_SECOND);

    // According to this:
    // https://www.chiefdelphi.com/t/ctre-phoenix-pro-to-phoenix-6-looking-back-and-looking-ahead/437313/27
    // When using Phoenix6, the CanCoder should not have problems on startup as long as we wait for an update and check for errors.
    var posVal = m_canCoder.getAbsolutePosition().waitForUpdate(0.1); // This actaully waits that long! Don't call after init!
    if(posVal.getStatus().isOK()) {
        /* Perform seeding */
        double val = posVal.getValue();
        m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0 - m_canCoderOffsetDegrees));
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
    }

    // Math check!  Is reading the SparkMax position good enough?
    // CANcoder resolution in degrees: 360/4096 = 0.088 degrees
    // SparkMax encoder resolution: 360/ ( 48 positions * ANGLE_GEAR_RATIO (150/7) ) = 0.35 degrees
    // So is the CANcoder slightly better? Yes, but the CANcoder hardware spec:
    // https://store.ctr-electronics.com/content/user-manual/CANCoder%20User's%20Guide.pdf
    // says that the absolute position can be off by 1.44 degrees if there is rotation!
    // It may be possible to servo to m_canCoder.getPosition(), but then then CAN bus utilization will go way up.
    // Future project: Servo on the CANcoder position and see if CAN bus utilization is a problem.
  }
}