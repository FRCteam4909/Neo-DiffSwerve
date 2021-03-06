/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  SwerveModule modOne;

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();
    final int driveMotorChannelA = 1;
    final int driveMotorChannelB = 2;
    final int dioEncoderChanA    = 0;
    final int dioEncoderChanB    = 1;
    modOne = new SwerveModule(driveMotorChannelA, driveMotorChannelB, dioEncoderChanA, dioEncoderChanB);

    // /**
    //  * In order to use PID functionality for a controller, a CANPIDController object
    //  * is constructed by calling the getPIDController() method on an existing
    //  * CANSparkMax object
    //  */
    // m_pidController = m_motor.getPIDController();

    // // Encoder object created to display position values
    // m_encoder = m_motor.getEncoder();

    // // PID coefficients
    // kP = 6e-5; 
    // kI = 0;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000175; 
    // kMaxOutput = .3; 
    // kMinOutput = -.3;
    // maxRPM = 5700;

    // // set PID coefficients
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Desired Velocity", 0.0);

    // SmartDashboard.putBoolean("1500", false);
    // SmartDashboard.putBoolean("1000", false);
    // SmartDashboard.putBoolean("500", false);
    // SmartDashboard.putBoolean("0", false);

    SmartDashboard.putNumber("RequestedSpeed", 0);
    SmartDashboard.putNumber("RequestedAngle", 0);
  }

  @Override
  public void teleopPeriodic() {



    double speedMetersPerSecond = SmartDashboard.getNumber("RequestedSpeed", 0);
    double angle = SmartDashboard.getNumber("RequestedAngle", 0);
    Rotation2d angleRot2d = Rotation2d.fromDegrees(angle);
    SwerveModuleState desiredState = new SwerveModuleState(speedMetersPerSecond, angleRot2d);
    
    modOne.setDesiredState(desiredState);

    SmartDashboard.putNumber("ActualAngle", modOne.calcTurn());
    SmartDashboard.putData("TurnEncoder", modOne.m_turningEncoder);
    SmartDashboard.putNumber("ActualVelocity", modOne.calcVelocity());





    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { m_pidController.setP(p); kP = p; }
    // if((i != kI)) { m_pidController.setI(i); kI = i; }
    // if((d != kD)) { m_pidController.setD(d); kD = d; }
    // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   m_pidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }

    // /**
    //  * PIDController objects are commanded to a set point using the 
    //  * SetReference() method.
    //  * 
    //  * The first parameter is the value of the set point, whose units vary
    //  * depending on the control type set in the second parameter.
    //  * 
    //  * The second parameter is the control type can be set to one of four 
    //  * parameters:
    //  *  com.revrobotics.ControlType.kDutyCycle
    //  *  com.revrobotics.ControlType.kPosition
    //  *  com.revrobotics.ControlType.kVelocity
    //  *  com.revrobotics.ControlType.kVoltage
    //  */



    // if (SmartDashboard.getBoolean("1500", false)) {
    //   SmartDashboard.putNumber("Desired Velocity", 1500.0);
    //   SmartDashboard.putBoolean("1500", false);
    // }
    // if (SmartDashboard.getBoolean("1000", false)) {
    //   SmartDashboard.putNumber("Desired Velocity", 1000.0);
    //   SmartDashboard.putBoolean("1000", false);
    // }
    // if (SmartDashboard.getBoolean("500", false)) {
    //   SmartDashboard.putNumber("Desired Velocity", 500.0);
    //   SmartDashboard.putBoolean("500", false);
    // }
    // if (SmartDashboard.getBoolean("0", false)) {
    //   SmartDashboard.putNumber("Desired Velocity", 0.0);
    //   SmartDashboard.putBoolean("0", false);
    // }
    // // double setPoint = m_stick.getY()*maxRPM;
    // double setPoint = SmartDashboard.getNumber("Desired Velocity", 0.0);
    // m_pidController.setReference(setPoint, ControlType.kVelocity);
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
}