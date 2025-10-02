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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean tagUpdate = true;

  public static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.defaultAprilTagType
              .getLayout()
              .getTagPose(i)
              .map(Pose3d::toPose2d)
              .orElse(new Pose2d()));
    }
  }

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                || !tagUpdate;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public boolean seenTagId(int id, int camera) {
    for (var tagId : inputs[camera].tagIds) {
      if (tagId == id) return true;
    }
    return false;
  }

  public void blockTagUpdate() {
    tagUpdate = false;
  }

  public void unblockTagUpdate() {
    tagUpdate = true;
  }

  @AutoLogOutput(key = "Vision/Distance To Tag")
  public double getDistanceToTag(int camera) {
    return inputs[camera].latestFiducialsObservations.distToCamera();
  }

  @AutoLogOutput(key = "Vision/Score Ready")
  public boolean scoreReady() {
    return false;
  }

  // @AutoLogOutput(key = "Vision/RobotPoseWithTag")
  public Pose2d getFieldPoseUsingTag(int camera, Rotation2d rotation) {
    double xAngle =
        inputs[camera].latestTargetObservation.tx().getDegrees()
            - VisionConstants.cameraAngleOffsetsYaw[camera];
    double yAngle =
        inputs[camera].latestTargetObservation.ty().getDegrees()
            - VisionConstants.cameraAngleOffsetsPitch[camera];
    double distance2d = getDistanceToTag(camera) * Math.cos(Units.degreesToRadians(yAngle));

    double yDist = Math.sin(Units.degreesToRadians(xAngle)) * distance2d;
    double xDist = Math.cos(Units.degreesToRadians(xAngle)) * distance2d;

    Logger.recordOutput("AutoLineup/distanceToTag", distance2d);
    Logger.recordOutput("AutoLineup/xDist", xDist);
    Logger.recordOutput("AutoLineup/yDist", yDist);

    Pose2d robotLocalPose =
        new Pose2d(xDist, yDist, rotation)
            .transformBy(new Transform2d(-0.280509, 0.257131, new Rotation2d()));
    Pose2d aprilTagFieldPose = tagPoses2d.get(inputs[camera].latestFiducialsObservations.id());

    return aprilTagFieldPose.transformBy(
        new Transform2d(
            robotLocalPose.getTranslation(),
            robotLocalPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
  }

  public Pose2d getFieldPoseUsingTag2(int camera, Rotation2d rot) {
    double tx = inputs[camera].latestTargetObservation.tx().getRadians();
    double ty = inputs[camera].latestTargetObservation.ty().getRadians();
    Pose2d tagPose2d = tagPoses2d.get(inputs[camera].latestFiducialsObservations.id());
    if (tagPose2d == null) return new Pose2d();
    Pose3d cameraPose = VisionConstants.cameraOffsets[camera];

    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, tx))
            .transformBy(
                new Transform3d(
                    new Translation3d(getDistanceToTag(camera), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        rot.plus(cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
            .getTranslation();

    Logger.recordOutput(
        "AutoLineup/camToTagTranslation",
        new Pose2d(camToTagTranslation.getX(), camToTagTranslation.getY(), rot));
    Logger.recordOutput(
        "AutoLineup/fieldToCameraTranslation",
        new Pose2d(fieldToCameraTranslation.getX(), fieldToCameraTranslation.getY(), rot));

    Pose2d robotPose =
        new Pose2d(fieldToCameraTranslation, rot.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

    robotPose = new Pose2d(robotPose.getTranslation(), rot);

    return robotPose;
  }
}
