package frc.robot.subsystems.implementations.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Vision;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase implements Vision {
  public static class Constants {
    public static double visionDistanceOffsetInMeters =
        0; // Average difference between vision-calculated distance vs actual
  }

  private class VisionCameraImpl {
    private static int numCameras = 0;
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator poseEstimator;
    private final int index;
    private PhotonCameraSim simCamera;
    private PhotonPipelineResult result;
    private Optional<EstimatedRobotPose> estimatedRobotPose;

    private VisionCameraImpl(Camera camera, AprilTagFieldLayout fieldLayout) {
      this.camera = new PhotonCamera(camera.getName());
      this.robotToCamera = camera.getRobotToCamera();
      this.poseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.robotToCamera);
      this.index = numCameras++;
      update();
    }

    private void update() {
      for (PhotonPipelineResult photonPipelineResult : camera.getAllUnreadResults()) {
        estimatedRobotPose = poseEstimator.update(photonPipelineResult);
        result = photonPipelineResult;
      }
    }

    private double getLatestTimestamp() {
      return result.getTimestampSeconds();
    }

    private boolean hasTargets() {
      return result.hasTargets();
    }

    private List<PhotonTrackedTarget> getTargets() {
      return result.targets;
    }

    private PhotonTrackedTarget getBestTarget() {
      return result.getBestTarget();
    }

    private double getDistanceToBestTarget() {
      // ASSUME this camera can see a target
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      // if (bestTarget == null) return -1.0;

      return PhotonUtils.calculateDistanceToTargetMeters(
              robotToCamera.getZ(),
              fieldLayout.getTagPose(bestTarget.getFiducialId()).get().getZ(),
              robotToCamera.getRotation().getY(),
              Units.degreesToRadians(bestTarget.getPitch()))
          + Constants.visionDistanceOffsetInMeters;
    }

    private Optional<EstimatedRobotPose> getEstimatedRobotPose() {
      return estimatedRobotPose;
    }

    private int getIndex() {
      return index;
    }

    private String getName() {
      return camera.getName();
    }

    private Transform3d getRobotToCamera() {
      return robotToCamera;
    }
  }

  private final List<VisionCameraImpl> cameras = new ArrayList<VisionCameraImpl>();
  private VisionCameraImpl primaryCamera = null;
  private final AprilTagFieldLayout fieldLayout;
  private VisionMeasurementConsumer visionMeasurementConsumer = null;
  List<Camera> visionCameras = new ArrayList<Camera>();

  /* Debug Info */
  @AutoLogOutput private int debugTargetsVisible;

  @AutoLogOutput private double debugTargetDistance = 0;
  @AutoLogOutput private double debugTargetYaw = 0;

  /* Simulation Support*/
  private boolean simEnabled = false;
  private VisionSystemSim simVision;
  private Supplier<Pose2d> simPoseSupplier;

  public VisionSubsystem(AprilTagFieldLayout fieldLayout) {
    this.fieldLayout = fieldLayout;
  }

  @Override
  public void addCamera(Camera camera) {
    cameras.add(new VisionCameraImpl(camera, fieldLayout));
    visionCameras.add(camera);

    if (1 == cameras.size()) {
      primaryCamera = this.cameras.get(0);
    }
  }

  @Override
  public List<Camera> getCameras() {
    return visionCameras;
  }

  @Override
  public void enableSimulation(Supplier<Pose2d> poseSupplier, boolean enableWireFrame) {
    simVision = new VisionSystemSim("main");
    simVision.addAprilTags(this.fieldLayout);

    for (VisionCameraImpl camera : cameras) {
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(70));
      // cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(120);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      camera.simCamera = new PhotonCameraSim(camera.camera, cameraProp);
      simVision.addCamera(camera.simCamera, camera.robotToCamera);
      camera.simCamera.enableDrawWireframe(enableWireFrame);
    }

    this.simPoseSupplier = poseSupplier;
    simEnabled = true;
  }

  public void updatePoseEstimation() {
    // Initialize new list of robot poses (for debugging)
    List<Pose2d> debugRobotPoses = new LinkedList<>();

    if (simEnabled) {
      simVision.update(simPoseSupplier.get());
    }

    for (VisionCameraImpl camera : cameras) {
      camera.update();

      Optional<EstimatedRobotPose> currentEstimatedRobotPose = camera.getEstimatedRobotPose();
      if (currentEstimatedRobotPose.isPresent()) {
        double distanceToBestTarget = camera.getDistanceToBestTarget();

        // Add vision measurement to the consumer.
        if (visionMeasurementConsumer != null && distanceToBestTarget > 5) {
          visionMeasurementConsumer.add(
              currentEstimatedRobotPose.get().estimatedPose.toPose2d(),
              currentEstimatedRobotPose.get().timestampSeconds,
              VecBuilder.fill(
                  distanceToBestTarget / 2, distanceToBestTarget / 2, distanceToBestTarget / 2));
        }
        /* NOTE standard deviation format:
         * (x position in meters, y position in meters, and heading in radians)
         * Increase these numbers to trust the vision pose measurement less.
         */

        // Log estimated robot pose for debugging
        debugRobotPoses.add(currentEstimatedRobotPose.get().estimatedPose.toPose2d());
      } else {
        // Display the robot pose "out of the arena" (indicating no pose found)
        debugRobotPoses.add(new Pose2d(-10.0, -10.0, new Rotation2d(0)));
      }
    }

    // Record estimated robot pose to AdvantageKit networktables
    if (debugRobotPoses.size() > 0) {
      Logger.recordOutput(
          "VisionSubsystem/DEBUGestimatedCameraPoses",
          debugRobotPoses.toArray(new Pose2d[debugRobotPoses.size()]));
    }
  }

  @Override
  public void periodic() {
    updatePoseEstimation();
  }

  private PhotonTrackedTarget findAprilTag(int id) {
    if (primaryCamera != null) {
      for (PhotonTrackedTarget target : primaryCamera.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }
    return null;
  }

  @Override
  public Optional<Double> getDistanceToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if ((target != null) && (primaryCamera != null)) {
      return Optional.of(
          PhotonUtils.calculateDistanceToTargetMeters(
                  primaryCamera.getRobotToCamera().getZ(),
                  fieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
                  -primaryCamera.getRobotToCamera().getRotation().getY(),
                  Units.degreesToRadians(target.getPitch()))
              + Constants.visionDistanceOffsetInMeters);
    }
    return Optional.empty();
  }

  @Override
  public Optional<Integer> getBestTargetId() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return Optional.of(primaryCamera.getBestTarget().getFiducialId());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToBestTarget() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return Optional.of(primaryCamera.getBestTarget().getYaw());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getDistanceToBestTarget() {
    if ((primaryCamera != null) && (primaryCamera.hasTargets())) {
      return getDistanceToAprilTag(primaryCamera.getBestTarget().getFiducialId());
    }
    return Optional.empty();
  }

  @Override
  public Optional<Double> getYawToAprilTag(int id) {
    PhotonTrackedTarget target = findAprilTag(id);

    if (target != null) {
      return Optional.of(target.getYaw());
    }

    return Optional.empty();
  }

  @Override
  public boolean setPrimaryCamera(String name) {
    boolean foundCamera = false;

    for (VisionCameraImpl camera : cameras) {
      if (camera.getName().equals(name)) {
        primaryCamera = camera;
        foundCamera = true;
        break;
      }
    }
    return foundCamera;
  }

  @Override
  public void setVisionMeasurementConsumer(VisionMeasurementConsumer func) {
    visionMeasurementConsumer = func;
  }
}
