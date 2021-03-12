// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Math.PI; // 1/2 rotation per second;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // https://github.com/ameliorater/ftc-diff-swerve/blob/master/DriveModule.java#L43
  //this variable is set to 0.7 because when in RUN_USING_ENCODERS mode, powers about ~0.7 are the same
  //setting to 1 may increase robot top speed, but may decrease accuracy
  public double MAX_MOTOR_POWER = 0.7;

  private final CANSparkMax m_driveMotorA;
  private final CANSparkMax m_driveMotorB;

  private final CANEncoder m_driveEncoderA;
  private final CANEncoder m_driveEncoderB;
  public final Encoder m_turningEncoder;

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
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("TurningPID", m_drivePIDController);
    SmartDashboard.putData("DrivePID", m_drivePIDController);
  }

  public double calcVelocity()
  {
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

    return speed;
  }

  public double calcTurn()
  {
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

    // double speed = vecA.getXChange();
    double turn = vecA.getYChange();

    // A&B going the same direction => module rotation
    // another way to say this is the signes are the same.

    // A&B going opposite directions => wheel rotation (robot translation)

    return turn;
  }

  /**
   * Returns the current state of the module.
   */
  public SwerveModuleState getState() {
    
    
    return new SwerveModuleState(calcVelocity(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());
    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    final double turnMag = turnOutput + turnFeedforward;

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(calcVelocity(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    final double driveMag = driveOutput + driveFeedforward;


    Vector driveVec = new Vector(0, driveMag);
    Vector turnVec = new Vector(Math.PI/2, turnMag);

    driveVec.sum(turnVec); // now driveVec is the resultant vector (target)

    // we need the projection method from the Vec2d class... this could be written better
    Vec2d powerVector = new Vec2d(driveVec.getXChange(), driveVec.getYChange());
    // Vector powerVector = new Vector(driveVec);

    // src: https://github.com/ameliorater/ftc-diff-swerve/blob/master/DriveModule.java
    //this is one way to convert desired ratio of module translation and module rotation to motor powers
    //vectors are not strictly necessary for this, but made it easier to visualize
    //more documentation on this visualization method coming soon
    final Vec2d MOTOR_1_VECTOR = new Vec2d(1/Math.sqrt(2), 1/Math.sqrt(2));
    final Vec2d MOTOR_2_VECTOR = new Vec2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

    Vec2d motor1Unscaled = powerVector.projection(MOTOR_1_VECTOR);
    Vec2d motor2Unscaled = powerVector.projection(MOTOR_2_VECTOR);

    //makes sure no vector magnitudes exceed the maximum motor power
    Vec2d[] motorPowersScaled = Vec2d.batchNormalize(MAX_MOTOR_POWER, motor1Unscaled, motor2Unscaled);
    double motor1power = motorPowersScaled[0].getMagnitude();
    double motor2power = motorPowersScaled[1].getMagnitude();

    //@todo We may need to do some sign manipulation

    SmartDashboard.putNumber("MotorAOutput", motor1power);
    SmartDashboard.putNumber("MotorBOutput", motor2power);

    m_driveMotorA.setVoltage(motor1power);
    m_driveMotorB.setVoltage(motor2power);
  }

 
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   builder.setSmartDashboardType("SwerveModule");
  //   builder.add
  //   builder.addDoubleProperty("p", this::getP, this::setP);
  //   builder.addDoubleProperty("i", this::getI, this::setI);
  //   builder.addDoubleProperty("d", this::getD, this::setD);
  //   builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
  // }
}
