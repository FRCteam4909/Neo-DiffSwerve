// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Math.PI; // 1/2 rotation per second;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotorA;
  private final CANSparkMax m_driveMotorB;

  private final CANEncoder m_driveEncoderA;
  private final CANEncoder m_driveEncoderB;
  private final Encoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   */
  public SwerveModule(int driveMotorChannelA, int driveMotorChannelB, int dioEncoderChanA, int dioEncoderChanB) {
    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();
    m_turningEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB);

    // // Set the distance per pulse for the drive encoder. We can simply use the
    // // distance traveled for one rotation of the wheel divided by the encoder
    // // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // // Set the distance (in this case, angle) per pulse for the turning encoder.
    // // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // // divided by the encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // // Limit the PID Controller's input range between -pi and pi and set the input
    // // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   */
  public SwerveModuleState getState() {
    double a = m_driveEncoderA.getVelocity();
    double b = m_driveEncoderB.getVelocity();

    // see https://github.com/ameliorater/ftc-diff-swerve/wiki/Motor-Power-Vectors
    // We can build two vectors, one for a and b whose magnitude is the velosity of a and be respectivly
    // the vectors shall start at the origin
    // vector for motor a is in quadrant 1 or 3
    // vector for motor b is in quadrant 2 or 4
    // the addition of these two vectors creates a new vector that explains module state
    // on the coordinate plane:
    // Y+ is CW rotation
    // Y- is CCW Rotation
    // X+ is Forward
    // X- is Reverse
    // By breaking the resulting vecor down into its component parts we can determine
    // how much rotate vs how much translate

    Vector vecA = new Vector(Math.PI/2, a);
    Vector vecB = new Vector(Math.PI*3/4, b);
    vecA.sum(vecB);

    double speed = vecA.getXChange();
    // double turn = vecA.getYChange();

    // A&B going the same direction => module rotation
    // another way to say this is the signes are the same.

    // A&B going opposite directions => wheel rotation (robot translation)
    
    return new SwerveModuleState(speed, new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(-999999, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotorA.setVoltage(driveOutput + driveFeedforward);
    m_driveMotorB.setVoltage(turnOutput + turnFeedforward);
  }
}
