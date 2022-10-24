// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.ConMath;
import frc.robot.Constants.ConSparkMax;
import frc.robot.OI.ConShuffleboard;

public class DriveTrain extends SubsystemBase {

  public static class ConDriveTrain {
    // Autonomous Constants
    public static double AUTONOMOUS_DISTANCE = 84;  // 84.75 inches Needed to exit the launchpad 
    public static double AUTONOMOUS_DRIVE_DELAY = 5.0; // Seconds to delay between launch & drive
    public static final int AUTONOMOUS_MODE_2_BALL = 1;
    public static final int AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE = 2;
    public static final int AUTONOMOUS_MODE_JUST_MOVE = 3;
    public static final int AUTONOMOUS_MODE_5_BALL = 5;

    // Motors
    public static int RIGHT_MOTOR_A_ID = 2;
    public static int RIGHT_MOTOR_B_ID = 4;
    public static int LEFT_MOTOR_A_ID = 3;
    public static int LEFT_MOTOR_B_ID = 5;
    //constexpr double ROTATION_FACTOR = 1/1.3;

    //Spark Max Settings
    public static double RAMP_RATE = 0.100; //seconds
    public static boolean INVERTED = true; //
    public static boolean NONINVERTED = false; //
    
    // Neo Motor & Gearbox
    public static double ENCODER_TICK_RESOLUTION = 42.0; // IS IT REALLY 42? or 48? or maybe 24?  
    public static double GEAR_RATIO = 10.71; // Neo rotates 10.71 times for one rotation of the output
    public static double WHEEL_DIAMETER = 6.0;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * ConMath.PI; // Abt 18.85 in.

    public static double TICKS_PER_WHEEL_REVOLUTION = ENCODER_TICK_RESOLUTION * GEAR_RATIO; // Abt 450 ticks

    //Conversions
    public static double TICKS_PER_INCH = TICKS_PER_WHEEL_REVOLUTION / (double)WHEEL_CIRCUMFERENCE; // Abt 24 ticks per inch
    public static double INCHES_PER_TICK = (double)WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_REVOLUTION; // Abt 1/24 (.042)

    // degrees to in
    public static double ANGLE_2_IN = 25.5 * ConMath.PI/360; // FIXME: What is this fudge factor? 25.5?
    public static double IN_2_ANGLE= 1/ANGLE_2_IN;

    // Experimental Drive Characterization/SysID for Trajectory Following
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    //constexpr auto ks = 0.14251_V;
    //constexpr auto kv = 1.98 * 1_V * 1_s / 1_in;
    //constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_in;

    // Example value only - as above, this must be tuned for your drive!
    public static double kPDriveVel = 0.14047;

    //constexpr auto kTrackwidth = 22.8125_in;
    //extern const frc::DifferentialDriveKinematics kDriveKinematics;
    //constexpr auto kMaxSpeed = 3_mps;
    //constexpr auto kMaxAcceleration = 3_mps_sq;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static double kRamseteB = 39.3701 * 2.0; // Inches. Was: 2;
    public static double kRamseteZeta = 0.7;
  }

  ShuffleboardTab m_sbt_DriveTrain;
  public NetworkTableEntry m_nte_DriveSpeedFilter;
  public NetworkTableEntry m_nte_DriveRotationFilter;
  public NetworkTableEntry m_nte_InputExponent;

  // Encoder outputs
  NetworkTableEntry m_nte_LeftEncoder;
  NetworkTableEntry m_nte_RightEncoder;
  NetworkTableEntry m_nte_IMU_ZAngle;

  public NetworkTableEntry m_nte_Testing;

  // Autonomous Variables
  NetworkTableEntry m_nte_a_DriveDelay;
  NetworkTableEntry m_nte_b_DriveDistance;
  NetworkTableEntry m_nte_c_DriveTurnAngle;
  NetworkTableEntry m_nte_autoDriveMode;
    
  // Autonomous drive parameters
  public double m_autoDistance = ConDriveTrain.AUTONOMOUS_DISTANCE;
  public double m_autoDriveMode = ConDriveTrain.AUTONOMOUS_MODE_2_BALL;
  public double m_autoDriveDelay = ConDriveTrain.AUTONOMOUS_DRIVE_DELAY;

  double m_maxOutput = 1.0;

  // #ifdef ENABLE_DRIVETRAIN
  AHRS gyro;

  // Neo motor controllers
  CANSparkMax m_rightMotorA = new CANSparkMax(ConDriveTrain.RIGHT_MOTOR_A_ID, CANSparkMax.MotorType.kBrushless);
  CANSparkMax m_rightMotorB = new CANSparkMax(ConDriveTrain.RIGHT_MOTOR_B_ID, CANSparkMax.MotorType.kBrushless);
  CANSparkMax m_leftMotorA = new CANSparkMax(ConDriveTrain.LEFT_MOTOR_A_ID, CANSparkMax.MotorType.kBrushless);
  CANSparkMax m_leftMotorB = new CANSparkMax(ConDriveTrain.LEFT_MOTOR_B_ID, CANSparkMax.MotorType.kBrushless);

  // Drive Train Smart Motion PID set-up below 
  SparkMaxPIDController left_pidController = m_leftMotorA.getPIDController();
  SparkMaxPIDController right_pidController = m_rightMotorA.getPIDController();

  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  final double MaxRPM = 5700;

  // Drive encoders
  RelativeEncoder m_rightEncoderA = m_rightMotorA.getEncoder();
  RelativeEncoder m_rightEncoderB = m_rightMotorB.getEncoder();
  RelativeEncoder m_leftEncoderA = m_leftMotorA.getEncoder();
  RelativeEncoder m_leftEncoderB = m_leftMotorB.getEncoder();

  // Robot Drive
  DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftMotorA, m_rightMotorA);
  //#endif // ENABLE_DRIVETRAIN

  public DriveTrain() {
    //#ifdef ENABLE_DRIVETRAIN
    // Settings for Spark Max motor controllers should be done here, in code
    // and not in the Spark Max Client Software
    m_rightMotorA.setOpenLoopRampRate(ConDriveTrain.RAMP_RATE);
    m_rightMotorB.setOpenLoopRampRate(ConDriveTrain.RAMP_RATE);
    m_leftMotorA.setOpenLoopRampRate(ConDriveTrain.RAMP_RATE);
    m_leftMotorB.setOpenLoopRampRate(ConDriveTrain.RAMP_RATE);
    
    //  To swap front & back of robot, swap the INVERTED/NONINVERTED below and add or remove the minus sign 
    //  in RobotContainer.cpp on the line which contains "ConXBOXControl::LEFT_JOYSTICK_X/4" for the turning axis
    
    // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
    m_rightMotorA.setInverted(ConDriveTrain.NONINVERTED);
    m_rightMotorB.setInverted(ConDriveTrain.NONINVERTED);
    // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
    m_leftMotorA.setInverted(ConDriveTrain.INVERTED);
    m_leftMotorB.setInverted(ConDriveTrain.INVERTED);

    // set PID coefficients
    // left pid controller
    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(kIz);
    left_pidController.setFF(kFF);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // right pid controller
    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(kIz);
    right_pidController.setFF(kFF);
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController  object
     * 
     * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - SetSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
    */
    // left pid controller
    left_pidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    left_pidController.setSmartMotionMinOutputVelocity(kMinVel, 0);
    left_pidController.setSmartMotionMaxAccel(kMaxAcc, 0);
    left_pidController.setSmartMotionAllowedClosedLoopError(kAllErr, 0);

    //right pid controller
    right_pidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    right_pidController.setSmartMotionMinOutputVelocity(kMinVel, 0);
    right_pidController.setSmartMotionMaxAccel(kMaxAcc, 0);
    right_pidController.setSmartMotionAllowedClosedLoopError(kAllErr, 0);

    // Set additional motor controllers on drive train to follow
    m_rightMotorB.follow(m_rightMotorA, ConDriveTrain.NONINVERTED);
    m_leftMotorB.follow(m_leftMotorA, ConDriveTrain.NONINVERTED);

    // NavX gyro
    gyro = new AHRS(SPI.Port.kMXP);

    // We should always be writing the desired SparkMax settings if they're not the default
    m_leftEncoderA.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR);
    m_rightEncoderA.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR);
    m_leftEncoderB.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR);
    m_rightEncoderB.setPositionConversionFactor(ConSparkMax.POSITION_CONVERSION_FACTOR);
    
    /**
       FIXME: This may be a better way to set the distance conversion: Right on the SparkMax!
        Native Tick counts * Gear Ratio divided by Wheel circumference (42 * 10.71)/(6 * pi) = ticks per inch
        We can use the SetPositionConversionFactor() to use this as our tick reference.
        // m_leftEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
        // m_rightEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
        // m_leftEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
        // m_rightEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
      BUUUT: IF YOU DO THIS, CHANGE THE GetLeftDistanceInches() and GetRightDistanceInches() methods!!!
      */
    
    //#endif // ENABLE_DRIVETRAIN

    // Create and get reference to SB tab
    m_sbt_DriveTrain = Shuffleboard.getTab(ConShuffleboard.DriveTrainTab);

    // Create widgets for digital filter lengths
    m_nte_DriveSpeedFilter = m_sbt_DriveTrain.addPersistent("Drive Speed Filter", 10.0)
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();
    m_nte_DriveRotationFilter = m_sbt_DriveTrain.addPersistent("Drive Rotation Filter", 5.0)
      .withSize(2, 1)
      .withPosition(0, 1)
      .getEntry();

    // Create widget for non-linear input
    m_nte_InputExponent = m_sbt_DriveTrain.addPersistent("Input Exponent", 1.0)
      .withSize(1, 1)
      .withPosition(0, 2)
      .getEntry();

    // Create widgets for AutoDrive
    m_nte_a_DriveDelay = m_sbt_DriveTrain.addPersistent("a Launch Delay", ConDriveTrain.AUTONOMOUS_DRIVE_DELAY)
      .withSize(1, 1)
      .withPosition(3, 0)
      .getEntry();
    m_nte_b_DriveDistance = m_sbt_DriveTrain.addPersistent("b Drive Distance", ConDriveTrain.AUTONOMOUS_DISTANCE)
      .withSize(1, 1)
      .withPosition(3, 1)
      .getEntry();
    m_nte_c_DriveTurnAngle = m_sbt_DriveTrain.addPersistent("c Turn Angle", 0.0)
      .withSize(1, 1)
      .withPosition(3, 2)
      .getEntry();
    m_nte_autoDriveMode = m_sbt_DriveTrain.addPersistent("AutoDrive Mode", ConDriveTrain.AUTONOMOUS_MODE_2_BALL)
      .withSize(1, 1)
      .withPosition(3, 3)
      .getEntry();
    //  m_nte_Testing     = m_sbt_DriveTrain->AddPersistent("Testing", 0.0)       .WithSize(1, 1).WithPosition(3, 3).GetEntry();

    // Display current encoder values
    m_nte_LeftEncoder = m_sbt_DriveTrain.addPersistent("Left Side Encoder", 0.0)
      .withSize(2,1)
      .withPosition(4,0)
      .getEntry();
    m_nte_RightEncoder = m_sbt_DriveTrain.addPersistent("Right Side Encoder", 0.0)
      .withSize(2,1)
      .withPosition(4,1)
      .getEntry();
    m_nte_IMU_ZAngle = m_sbt_DriveTrain.addPersistent("IMU Z-Axis Angle", 0.0)
      .withSize(2,1)
      .withPosition(4,2)
      .getEntry();

    // End of DriveTrain Constructor
    System.out.println("DriveTrain() Constructor returning...");
  }

  // #ifdef ENABLE_DRIVETRAIN
  // This method will be called once per scheduler run
  public void periodic() {
    m_nte_LeftEncoder.setDouble(getAverageLeftEncoders());
    m_nte_RightEncoder.setDouble(getAverageRightEncoders());
    m_nte_IMU_ZAngle.setDouble(getGyroAngle());
  }

  // Used by TeleOpDrive
  public void arcadeDrive(double speed, double rotation) {
    m_driveTrain.arcadeDrive(speed, OI.deadZone(rotation));
  }

  // Used by AlignToPlayerStationPID
  public void tankDrive(double left, double right){
    m_driveTrain.tankDrive(left, right);
  }

  // Used by TeleOpSlowDrive
  public double getMaxOutput() {
    return m_maxOutput;
  }

  public void setMaxOutput(double maxOutput) {
    m_maxOutput = maxOutput;
    // m_drivetain is the frc::DifferentialDrive class/object
    m_driveTrain.setMaxOutput(maxOutput);
  }

  // Used by AutoDriveDistance
  public void resetEncoders() {
    m_rightEncoderA.setPosition(0.0);
    m_rightEncoderB.setPosition(0.0);
    m_leftEncoderA.setPosition(0.0);
    m_leftEncoderB.setPosition(0.0);
  }

  // Account for two encoders per side
  public double getRightDistanceInches() {
    return (getAverageRightEncoders() * ConDriveTrain.INCHES_PER_TICK);
  }

  public double getLeftDistanceInches() {
    return (getAverageLeftEncoders() * ConDriveTrain.INCHES_PER_TICK);
  }

  // Used by AutoDriveDistance
  public double getAverageDistanceInches() {
    // FIXME: Should't these be added, or is one negative? I think we just REVERSE the encoder. CRE 2022-01-25
    return ((getLeftDistanceInches() + getRightDistanceInches()) / 2.0);
  }

  public double getAverageLeftEncoders() {
    return (m_leftEncoderA.getPosition() + m_leftEncoderB.getPosition() ) / 2.0;
  }

  public double getAverageRightEncoders() {
    return (m_rightEncoderA.getPosition() + m_rightEncoderB.getPosition() ) / 2.0;
  }

  public void goToAngle(double angle) {
    angle = angle * ConDriveTrain.ANGLE_2_IN;
    left_pidController.setReference(angle, CANSparkMax.ControlType.kSmartMotion);
    right_pidController.setReference(angle, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  // Used by AutoTurn 																	   
  public void resetGyro() {
    gyro.reset();
  }
  //#endif // ENABLE_DRIVETRAIN

  // Call SetAutonomousParameters() inside the AutonomousInit() method to read values from Shuffleboard
  public void setAutonomousParameters() {
    double d;
    d = m_nte_autoDriveMode.getDouble(ConDriveTrain.AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE);
    if (d != m_autoDriveMode) {
      m_autoDriveMode = d;
      System.out.println("m_autoDriveMode set to " + m_autoDriveMode);
    }
    d = m_nte_b_DriveDistance.getDouble(ConDriveTrain.AUTONOMOUS_DISTANCE);
    if (d != m_autoDistance) {
      m_autoDistance = d;
      System.out.println("m_autoDistance set to " + m_autoDistance);
    }
    d = m_nte_a_DriveDelay.getDouble(ConDriveTrain.AUTONOMOUS_DRIVE_DELAY);
    if (d != m_autoDriveDelay) {
      m_autoDriveDelay = d;
      System.out.println("m_autoDriveDelay set to " + m_autoDriveDelay);
    }
  }

  public void burnFlash() {
    System.out.println("BurnFlash for DriveTrain");
    // #ifdef ENABLE_DRIVETRAIN
    // Save all SparkMax firmware parameters to flash memory
    m_leftMotorA.burnFlash();
    m_leftMotorB.burnFlash();
    m_rightMotorA.burnFlash();
    m_rightMotorB.burnFlash();
    //#endif // ENABLE_DRIVETRAIN
  }

  // void DriveTrain::SetSafety(bool safety) { SetSafetyEnabled(safety);}
}

/** Original H

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <rev/CANSparkMax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <AHRS.h>

#include "OI.h"
#include "Constants.h"

namespace ConDriveTrain {
    // Autonomous Constants
    constexpr double AUTONOMOUS_DISTANCE = 84;  // 84.75 inches Needed to exit the launchpad 
    constexpr double AUTONOMOUS_DRIVE_DELAY = 5.0; // Seconds to delay between launch & drive
    constexpr int AUTONOMOUS_MODE_2_BALL = 1;
    constexpr int AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE = 2;
    constexpr int AUTONOMOUS_MODE_JUST_MOVE = 3;
    constexpr int AUTONOMOUS_MODE_5_BALL = 5;

    // Motors
    constexpr int RIGHT_MOTOR_A_ID = 2;
    constexpr int RIGHT_MOTOR_B_ID = 4;
    constexpr int LEFT_MOTOR_A_ID = 3;
    constexpr int LEFT_MOTOR_B_ID = 5;
    //constexpr double ROTATION_FACTOR = 1/1.3;

    //Spark Max Settings
    constexpr int RAMP_RATE = 0.100; //seconds
    constexpr bool INVERTED = true; //
    constexpr bool NONINVERTED = false; //
    
    // Neo Motor & Gearbox
    constexpr double ENCODER_TICK_RESOLUTION = 42.0; // IS IT REALLY 42? or 48? or maybe 24?  
    constexpr double GEAR_RATIO = 10.71; // Neo rotates 10.71 times for one rotation of the output
    constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;
    constexpr units::length::inch_t WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI; // Abt 18.85 in.

    constexpr double TICKS_PER_WHEEL_REVOLUTION = ENCODER_TICK_RESOLUTION * GEAR_RATIO; // Abt 450 ticks

    //Conversions
    constexpr double TICKS_PER_INCH = TICKS_PER_WHEEL_REVOLUTION / (double)WHEEL_CIRCUMFERENCE; // Abt 24 ticks per inch
    constexpr double INCHES_PER_TICK = (double)WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_REVOLUTION; // Abt 1/24 (.042)

    // degrees to in
    constexpr double ANGLE_2_IN = 25.5*ConMath::PI/360; // FIXME: What is this fudge factor? 25.5?
    constexpr double IN_2_ANGLE= 1/ANGLE_2_IN;

    // Experimental Drive Characterization/SysID for Trajectory Following
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    constexpr auto ks = 0.14251_V;
    constexpr auto kv = 1.98 * 1_V * 1_s / 1_in;
    constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_in;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 0.14047;

    constexpr auto kTrackwidth = 22.8125_in;
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    constexpr double kRamseteB = 39.3701 * 2.0; // Inches. Was: 2;
    constexpr double kRamseteZeta = 0.7;
}

class DriveTrain : public frc2::SubsystemBase {
 public:
 
  DriveTrain();
  frc::ShuffleboardTab *m_sbt_DriveTrain;
  nt::NetworkTableEntry m_nte_DriveSpeedFilter;
  nt::NetworkTableEntry m_nte_DriveRotationFilter;
  nt::NetworkTableEntry m_nte_InputExponent;

  // Encoder outputs
  nt::NetworkTableEntry m_nte_LeftEncoder;
  nt::NetworkTableEntry m_nte_RightEncoder;
  nt::NetworkTableEntry m_nte_IMU_ZAngle;

  nt::NetworkTableEntry m_nte_Testing;

  // Autonomous Variables
  nt::NetworkTableEntry m_nte_a_DriveDelay;
  nt::NetworkTableEntry m_nte_b_DriveDistance;
  nt::NetworkTableEntry m_nte_c_DriveTurnAngle;
  nt::NetworkTableEntry m_nte_autoDriveMode;
  
#ifdef ENABLE_DRIVETRAIN
  
   * Will be called periodically whenever the CommandScheduler runs.
   
  void Periodic();

  
   * Drives the robot using arcade controls.
   *
   * @param speed the commanded forward movement
   * @param rotation the commanded rotation
   
  void ArcadeDrive(double speed, double rotation);

  void TankDrive(double left, double right);

  double GetMaxOutput();

  void SetMaxOutput(double maxOutput);

  double GetRightDistanceInches();
  double GetLeftDistanceInches();
  double GetAverageDistanceInches();
  
  double GetAverageRightEncoders();
  double GetAverageLeftEncoders();

  double GetGyroAngle();
  void ResetEncoders();

  void GoToAngle(double angle);
  void ResetGyro();
#endif // ENABLE_DRIVETRAIN

  // Retrieve from dashboard, set member variables
  void SetAutonomousParameters();

  void BurnFlash();
  //void SetSafety(bool safety);
  
  // Autonomous drive parameters
  double m_autoDistance = ConDriveTrain::AUTONOMOUS_DISTANCE;
  double m_autoDriveMode = ConDriveTrain::AUTONOMOUS_MODE_2_BALL;
  double m_autoDriveDelay = ConDriveTrain::AUTONOMOUS_DRIVE_DELAY;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double m_maxOutput = 1.0;

#ifdef ENABLE_DRIVETRAIN
  AHRS *gyro;

  // Neo motor controllers
  rev::CANSparkMax m_rightMotorA{ConDriveTrain::RIGHT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotorB{ConDriveTrain::RIGHT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorA{ConDriveTrain::LEFT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorB{ConDriveTrain::LEFT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};

  // Drive Train Smart Motion PID set-up below 
  rev::SparkMaxPIDController  left_pidController = m_leftMotorA.GetPIDController();
  rev::SparkMaxPIDController  right_pidController = m_rightMotorA.GetPIDController();

  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  const double MaxRPM = 5700;

  // Drive encoders
  rev::SparkMaxRelativeEncoder m_rightEncoderA = m_rightMotorA.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightEncoderB = m_rightMotorB.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftEncoderA = m_leftMotorA.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftEncoderB = m_leftMotorB.GetEncoder();

  // Robot Drive
  frc::DifferentialDrive m_driveTrain{m_leftMotorA, m_rightMotorA};
#endif // ENABLE_DRIVETRAIN
};

 */

/** Original CPP

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

#ifdef ENABLE_DRIVETRAIN
  // Settings for Spark Max motor controllers should be done here, in code
  // and not in the Spark Max Client Software
  m_rightMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_rightMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);

  
  //  To swap front & back of robot, swap the INVERTED/NONINVERTED below and add or remove the minus sign 
  //  in RobotContainer.cpp on the line which contains "ConXBOXControl::LEFT_JOYSTICK_X/4" for the turning axis
  
  // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
  m_rightMotorA.SetInverted(ConDriveTrain::NONINVERTED);
  m_rightMotorB.SetInverted(ConDriveTrain::NONINVERTED);
  // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
  m_leftMotorA.SetInverted(ConDriveTrain::INVERTED);
  m_leftMotorB.SetInverted(ConDriveTrain::INVERTED);

  // set PID coefficients
  // left pid controller
  left_pidController.SetP(kP);
  left_pidController.SetI(kI);
  left_pidController.SetD(kD);
  left_pidController.SetIZone(kIz);
  left_pidController.SetFF(kFF);
  left_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  // right pid controller
  right_pidController.SetP(kP);
  right_pidController.SetI(kI);
  right_pidController.SetD(kD);
  right_pidController.SetIZone(kIz);
  right_pidController.SetFF(kFF);
  right_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  
   * Smart Motion coefficients are set on a SparkMaxPIDController  object
   * 
   * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
   * the pid controller in Smart Motion mode
   * - SetSmartMotionMinOutputVelocity() will put a lower bound in
   * RPM of the pid controller in Smart Motion mode
   * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
   * of the pid controller in Smart Motion mode
   * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
   * error for the pid controller in Smart Motion mode
  
  // left pid controller
  left_pidController.SetSmartMotionMaxVelocity(kMaxVel);
  left_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
  left_pidController.SetSmartMotionMaxAccel(kMaxAcc);
  left_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  //right pid controller
  right_pidController.SetSmartMotionMaxVelocity(kMaxVel);
  right_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
  right_pidController.SetSmartMotionMaxAccel(kMaxAcc);
  right_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  // Set additional motor controllers on drive train to follow
  m_rightMotorB.Follow(m_rightMotorA, ConDriveTrain::NONINVERTED);
  m_leftMotorB.Follow(m_leftMotorA, ConDriveTrain::NONINVERTED);

  // NavX gyro
  gyro = new AHRS(frc::SPI::Port::kMXP);

  // We should always be writing the desired SparkMax settings if they're not the default
  m_leftEncoderA.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR);
  m_rightEncoderA.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR);
  m_leftEncoderB.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR);
  m_rightEncoderB.SetPositionConversionFactor(ConSparkMax::POSITION_CONVERSION_FACTOR);
  
     FIXME: This may be a better way to set the distance conversion: Right on the SparkMax!
     Native Tick counts * Gear Ratio divided by Wheel circumference (42 * 10.71)/(6 * pi) = ticks per inch
     We can use the SetPositionConversionFactor() to use this as our tick reference.
     // m_leftEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_rightEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_leftEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_rightEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
    BUUUT: IF YOU DO THIS, CHANGE THE GetLeftDistanceInches() and GetRightDistanceInches() methods!!!

  
#endif // ENABLE_DRIVETRAIN

  // Create and get reference to SB tab
  m_sbt_DriveTrain = &frc::Shuffleboard::GetTab(ConShuffleboard::DriveTrainTab);

  // Create widgets for digital filter lengths
  m_nte_DriveSpeedFilter    = m_sbt_DriveTrain->AddPersistent("Drive Speed Filter", 10.0)   .WithSize(2, 1).WithPosition(0, 0).GetEntry();
  m_nte_DriveRotationFilter = m_sbt_DriveTrain->AddPersistent("Drive Rotation Filter", 5.0) .WithSize(2, 1).WithPosition(0, 1).GetEntry();

  // Create widget for non-linear input
  m_nte_InputExponent       = m_sbt_DriveTrain->AddPersistent("Input Exponent", 1.0)        .WithSize(1, 1).WithPosition(0, 2).GetEntry();

  // Create widgets for AutoDrive
  m_nte_a_DriveDelay     = m_sbt_DriveTrain->AddPersistent("a Launch Delay", ConDriveTrain::AUTONOMOUS_DRIVE_DELAY).WithSize(1, 1).WithPosition(3, 0).GetEntry();
  m_nte_b_DriveDistance  = m_sbt_DriveTrain->AddPersistent("b Drive Distance", ConDriveTrain::AUTONOMOUS_DISTANCE).WithSize(1, 1).WithPosition(3, 1).GetEntry();
  m_nte_c_DriveTurnAngle = m_sbt_DriveTrain->AddPersistent("c Turn Angle", 0.0)       .WithSize(1, 1).WithPosition(3, 2).GetEntry();
  m_nte_autoDriveMode    = m_sbt_DriveTrain->AddPersistent("AutoDrive Mode", ConDriveTrain::AUTONOMOUS_MODE_2_BALL).WithSize(1, 1).WithPosition(3, 3).GetEntry();
  //  m_nte_Testing     = m_sbt_DriveTrain->AddPersistent("Testing", 0.0)       .WithSize(1, 1).WithPosition(3, 3).GetEntry();

  // Display current encoder values
  m_nte_LeftEncoder = m_sbt_DriveTrain->AddPersistent("Left Side Encoder", 0.0)             .WithSize(2,1).WithPosition(4,0).GetEntry();
  m_nte_RightEncoder = m_sbt_DriveTrain->AddPersistent("Right Side Encoder", 0.0)            .WithSize(2,1).WithPosition(4,1).GetEntry();
  m_nte_IMU_ZAngle = m_sbt_DriveTrain->AddPersistent("IMU Z-Axis Angle", 0.0)               .WithSize(2,1).WithPosition(4,2).GetEntry();

  // End of DriveTrain Constructor
  printf("DriveTrain() Constructor returning...\n");
}

#ifdef ENABLE_DRIVETRAIN
// This method will be called once per scheduler run
void DriveTrain::Periodic() {
  m_nte_LeftEncoder.SetDouble(GetAverageLeftEncoders());
  m_nte_RightEncoder.SetDouble(GetAverageRightEncoders());
  m_nte_IMU_ZAngle.SetDouble(GetGyroAngle());
}

// Used by TeleOpDrive
void DriveTrain::ArcadeDrive(double speed, double rotation) {
  m_driveTrain.ArcadeDrive(speed, DeadZone(rotation));
}

// Used by AlignToPlayerStationPID
void DriveTrain::TankDrive(double left, double right){
  m_driveTrain.TankDrive(left, right);
}

// Used by TeleOpSlowDrive
double DriveTrain::GetMaxOutput() {
    return m_maxOutput;
}

void DriveTrain::SetMaxOutput(double maxOutput) {
  m_maxOutput = maxOutput;
  // m_drivetain is the frc::DifferentialDrive class/object
  m_driveTrain.SetMaxOutput(maxOutput);
}

// Used by AutoDriveDistance
void DriveTrain::ResetEncoders() {
  m_rightEncoderA.SetPosition(0.0);
  m_rightEncoderB.SetPosition(0.0);
  m_leftEncoderA.SetPosition(0.0);
  m_leftEncoderB.SetPosition(0.0);
}

// Account for two encoders per side
double DriveTrain::GetRightDistanceInches() {
  return (GetAverageRightEncoders() * ConDriveTrain::INCHES_PER_TICK);
}

double DriveTrain::GetLeftDistanceInches() {
  return (GetAverageLeftEncoders() * ConDriveTrain::INCHES_PER_TICK);
}

// Used by AutoDriveDistance
double DriveTrain::GetAverageDistanceInches() {
  // FIXME: Should't these be added, or is one negative? I think we just REVERSE the encoder. CRE 2022-01-25
  return ((GetLeftDistanceInches() + GetRightDistanceInches()) / 2.0);
}

double DriveTrain::GetAverageLeftEncoders() {
  return (m_leftEncoderA.GetPosition() + m_leftEncoderB.GetPosition() ) / 2.0;
}

double DriveTrain::GetAverageRightEncoders() {
  return (m_rightEncoderA.GetPosition() + m_rightEncoderB.GetPosition() ) / 2.0;
}
void DriveTrain::GoToAngle(double angle) {
  angle *= ConDriveTrain::ANGLE_2_IN;
  left_pidController.SetReference(angle, rev::CANSparkMax::ControlType::kSmartMotion);
  right_pidController.SetReference(angle, rev::CANSparkMax::ControlType::kSmartMotion);
}
double DriveTrain::GetGyroAngle() {return gyro->GetAngle();}

// Used by AutoTurn 																	   
void DriveTrain::ResetGyro() {
  gyro->Reset();
}																				 
#endif // ENABLE_DRIVETRAIN

// Call SetAutonomousParameters() inside the AutonomousInit() method to read values from Shuffleboard
void DriveTrain::SetAutonomousParameters() {
  double d;
  d = m_nte_autoDriveMode.GetDouble(ConDriveTrain::AUTONOMOUS_MODE_LAUNCH_DELAY_MOVE);
  if (d != m_autoDriveMode) {
    m_autoDriveMode = d;
    printf("m_autoDriveMode set to %f\n", m_autoDriveMode);
  }
  d = m_nte_b_DriveDistance.GetDouble(ConDriveTrain::AUTONOMOUS_DISTANCE);
  if (d != m_autoDistance) {
    m_autoDistance = d;
    printf("m_autoDistance set to %f\n", m_autoDistance);
  }
  d = m_nte_a_DriveDelay.GetDouble(ConDriveTrain::AUTONOMOUS_DRIVE_DELAY);
  if (d != m_autoDriveDelay) {
    m_autoDriveDelay = d;
    printf("m_autoDriveDelay set to %f\n", m_autoDriveDelay);
  }
}

void DriveTrain::BurnFlash() {
  printf("BurnFlash for DriveTrain\n");
#ifdef ENABLE_DRIVETRAIN
 // Save all SparkMax firmware parameters to flash memory
  m_leftMotorA.BurnFlash();
  m_leftMotorB.BurnFlash();
  m_rightMotorA.BurnFlash();
  m_rightMotorB.BurnFlash();
#endif // ENABLE_DRIVETRAIN
}

// void DriveTrain::SetSafety(bool safety) { SetSafetyEnabled(safety);}

 */