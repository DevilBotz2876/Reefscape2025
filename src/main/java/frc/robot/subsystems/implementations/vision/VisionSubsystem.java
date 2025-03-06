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
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
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
    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

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
      // This methods is invoked "periodically"
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
              -robotToCamera.getRotation().getY(),
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
  }

  private final List<VisionCameraImpl> cameras = new ArrayList<VisionCameraImpl>();
  private final AprilTagFieldLayout fieldLayout;
  private VisionMeasurementConsumer visionMeasurementConsumer = null;
  List<Camera> visionCameras = new ArrayList<Camera>();

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

    if (simEnabled) {
      simVision.update(simPoseSupplier.get());
    }

    for (VisionCameraImpl camera : cameras) {
      camera.update();

      // Default values: in case no targets are seen
      double distanceToTarget = -1;
      Pose2d estimatedRobotPoseFromCamera = new Pose2d(-10.0, -10.0, new Rotation2d(0));

      Optional<EstimatedRobotPose> currentEstimatedRobotPose = camera.getEstimatedRobotPose();
      if (currentEstimatedRobotPose.isPresent()) {

        estimatedRobotPoseFromCamera = currentEstimatedRobotPose.get().estimatedPose.toPose2d();
        distanceToTarget = camera.getDistanceToBestTarget();

        // Add vision measurement to the consumer.
        if (visionMeasurementConsumer != null && distanceToTarget < 2) {
          visionMeasurementConsumer.add(
              estimatedRobotPoseFromCamera,
              currentEstimatedRobotPose.get().timestampSeconds,
              VecBuilder.fill(distanceToTarget / 2, distanceToTarget / 2, distanceToTarget / 2));
        }
        /* NOTE standard deviation format:
         * (x position in meters, y position in meters, and heading in radians)
         * Increase these numbers to trust the vision pose measurement less.
         */
      }

      // Log information for debugging
      Logger.recordOutput(
          "VisionSubsystem/Distances To Target/" + camera.getName(), distanceToTarget);
      Logger.recordOutput(
          "VisionSubsystem/Estimated Robot Poses/" + camera.getName(),
          estimatedRobotPoseFromCamera);
    }
  }

  @Override
  public void periodic() {
    updatePoseEstimation();
  }

  @Override
  public void setVisionMeasurementConsumer(VisionMeasurementConsumer func) {
    visionMeasurementConsumer = func;
  }
}
