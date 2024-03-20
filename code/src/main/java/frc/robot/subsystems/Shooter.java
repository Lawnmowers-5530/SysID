// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  // The motor on the shooter wheel .
  private final CANSparkMax m_angleMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

  // The shooter wheel rev absolute encoder
  private final SparkAbsoluteEncoder m_angleEncoder = m_angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_angleMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-angle")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_angleMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(m_angleEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_angleEncoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
  // calculated by SysId
  private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0, 0);
  // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
  // those calculated by SysId
  private final ArmFeedforward m_shooterFeedforward =
      new ArmFeedforward(
          ShooterConstants.kSVolts,
          ShooterConstants.kGVolts,
          ShooterConstants.kVVoltSecondsPerRotation,
          ShooterConstants.kAVoltSecondsSquaredPerRotation);

  /** Creates a new Shooter subsystem. */
  public Shooter() {
    m_angleEncoder.setZeroOffset(Constants.ShooterConstants.kAngleOffset);
  }

  /**
   * Returns a command that runs the shooter at a specifc velocity.
   *
   * @param shooterSpeed The commanded shooter wheel speed in rotations per second
   */
  public Command runShooter(DoubleSupplier shooterPosition) {
    // Run shooter to the desired angle using a PID controller and feedforward.
    return run(() -> {
          m_angleMotor.setVoltage(
              m_shooterFeedback.calculate(m_angleEncoder.getPosition(), shooterPosition.getAsDouble())
                  + m_shooterFeedforward.calculate(shooterPosition.getAsDouble(), 0)); //TODO idk if the velocity setpoint should be 0
        })
        .finallyDo(
            () -> {
              m_angleMotor.stopMotor();
            })
        .withName("runShooter");
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
