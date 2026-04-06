// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        // Send gyro heading to Limelight so MegaTag2 can use it
        double gyroYaw = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(
            Constants.AutoAlign.kLimelightName,
            gyroYaw, 0, 0, 0, 0, 0
        );

        // Fuse MegaTag2 vision into drivetrain pose estimator.
        // MegaTag2 uses the gyro heading we just sent, so its pose is much more stable.
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.AutoAlign.kLimelightName
        );
        if (mt2 != null && mt2.tagCount > 0 && mt2.avgTagDist < 4.0) {
            // Scale trust based on distance — farther tags = less trust
            double xyStdDev = 0.3 + (0.3 * mt2.avgTagDist);
            // Single-tag detections are much noisier
            if (mt2.tagCount == 1) {
                xyStdDev *= 2.0;
            }
            m_robotContainer.drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, 999.0)
            );
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

@Override
public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}