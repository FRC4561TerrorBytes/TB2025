// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.VisionRecorder;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private VideoSource limelightCamera1;
  private VideoSource limelightCamera2;
  private VisionRecorder visionRecorder1;
  private VisionRecorder visionRecorder2;

  private Thread visionRecordingThread;
  private volatile boolean visionRecordingActive = false;

  private boolean lastEnabledAuto = false;

  public Robot() {
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();

    // Start an MJPEG stream using the camera feeds on the limelights
    limelightCamera1 =
        new HttpCamera(
            "limelight1",
            "http://10.45.61:18:5800/stream.mjpg"); // FRONT LEFT (IDK IF THIS IP IS RIGHT)
    limelightCamera2 =
        new HttpCamera(
            "limelight2",
            "http://10.45.61.19:5800/stream.mjpg"); // BACK RIGHT (THIS HAS THE RIGHT IP)

    // OPTIONAL
    // Restreams out the limelight feed on port 1182 and 1183
    MjpegServer server1 = new MjpegServer("serve_ll1", 1182);
    MjpegServer server2 = new MjpegServer("serve_ll2", 1183);
    server1.setSource(limelightCamera1);
    server2.setSource(limelightCamera2);

    // adding the camera feeds to the vision recorder
    visionRecorder1 = new VisionRecorder(limelightCamera1, 5, 1);
    visionRecorder2 = new VisionRecorder(limelightCamera2, 5, 2);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    VirtualSubsystem.periodicAll();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // if we are in a match then dont stop recording in the 2 second period where disabled
    if (DriverStation.getMatchNumber() > 0) {
      if (visionRecordingThread != null && !lastEnabledAuto) {
        try {
          // waiting for the thread to shutdown after setting it to inactive
          visionRecordingActive = false;

          visionRecordingThread.join();

          visionRecorder1.close();
          visionRecorder2.close();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
    }
    // if we are just practicing then stop the recording
    else {
      if (visionRecordingThread != null) {
        try {
          // waiting for the thread to shutdown after setting it to inactive
          visionRecordingActive = false;

          visionRecordingThread.join();

          visionRecorder1.close();
          visionRecorder2.close();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    // start the vision recording
    visionRecordingActive = true;
    // creating a new thread so the command schduler is ideally not affected by this running
    visionRecordingThread =
        new Thread(
            () -> {
              while (visionRecordingActive) {
                visionRecorder1.update();
                visionRecorder2.update();
                try {
                  // stopping thread for 10 milliseconds so it does not run as fast as it possibly
                  // can
                  Thread.sleep(10);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
              }
            });
    // start running the recording thread
    visionRecordingThread.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    lastEnabledAuto = true;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    if (visionRecordingThread == null) {
      visionRecordingActive = true;
      visionRecordingThread =
          new Thread(
              () -> {
                while (visionRecordingActive) {
                  visionRecorder1.update();
                  visionRecorder2.update();
                  try {
                    // stopping thread for 10 milliseconds so it does not run as fast as it possibly
                    // can
                    Thread.sleep(10);
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                  }
                }
                visionRecorder1.close();
                visionRecorder2.close();
              });
      // start running the recording thread
      visionRecordingThread.start();
    }

    lastEnabledAuto = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
