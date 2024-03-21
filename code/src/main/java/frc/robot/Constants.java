// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ShooterConstants {
    public static final boolean kEncoderReversed = false;
    public static final int kShooterMotorPort = 22;

    // These are not real PID gains, and will have to be tuned for your specific robot.
    public static final double kP = 0.01;

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation = 0.01;
    public static final double kGVolts = 0.01;
    public static final double kAVoltSecondsSquaredPerRotation = 0;

    public static final double kAngleOffset = 0.32; //TODO i think this is the right zero offset, 115 deg from what i remember which is 0.32 rotations
  }
}
