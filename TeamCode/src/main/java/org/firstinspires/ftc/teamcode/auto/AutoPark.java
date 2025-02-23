package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@Autonomous(name = "AutoPark", group = "drive")
public class AutoPark extends AutoOp {
    Trajectory signalOne;
    Trajectory signalTwo;
    Trajectory signalThree;

    @Override
    protected void setupTrajectories() {
        signalOne = drive.trajectoryBuilder(new Pose2d(0.00, -0.00, Math.toRadians(1.97)))
                .splineTo(new Vector2d(30.98, 8.74), Math.toRadians(89.08))
                .splineTo(new Vector2d(38.49, 21.74), Math.toRadians(-6.71))
                .build();
    }

    public enum AutoState {
        PARK_CV,
        PARK,
    }
    AutoState autoState = AutoState.PARK_CV;

    public Pose2d startPose;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startPose = new Pose2d(0, 0, Math.toRadians(-90));
//        startPose = new Pose2d(-60, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
//        doCV();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 500);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive() && timer.time() < 30) {
            if(isStopRequested()) return;
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            int detection = 0;
            switch (autoState) {
                case PARK_CV:
                    // If there's been a new frame...
                    if (detections != null) {
                        telemetry.addData("FPS", camera.getFps());
                        telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                        telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                        // If we don't see any tags
                        if (detections.size() == 0) {
                            numFramesWithoutDetection++;
                            if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                                aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        } else {
                            numFramesWithoutDetection = 0;
                            // If the target is within 1 meter, turn on high decimation to
                            // increase the frame rate
                            if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                            for (AprilTagDetection detection2 : detections) {
                                detection = detection2.id;
                                autoState = AutoState.PARK;
                                telemetry.addData("Detection2", detection2.id);
                                switch (detection) {
                                    case 10:
                                        telemetry.addData("Signal", "1");
                                        drive.followTrajectory(signalOne);
                                        autoState = AutoState.PARK;
                                        break;
                                    case 11:
                                        telemetry.addData("Signal", "2");

                                        signalTwo = drive.trajectoryBuilder(new Pose2d(-0.07, 0.22, Math.toRadians(-0.50)))
                                                .splineTo(new Vector2d(40.00, -0.00), Math.toRadians(6.01))
                                                .build();

                                        drive.followTrajectory(signalTwo);
                                        autoState = AutoState.PARK;
                                        break;
                                    case 21:
                                        telemetry.addData("Signal", "3");

                                        signalThree = drive.trajectoryBuilder(new Pose2d(-0.07, 0.36, Math.toRadians(0.87)))
                                                .splineTo(new Vector2d(25.78, -0.07), Math.toRadians(2.86))
                                                .splineTo(new Vector2d(31.41, -16.97), Math.toRadians(270.00))
                                                .splineTo(new Vector2d(43.55, -21.88), Math.toRadians(8.62))
                                                .build();


                                        drive.followTrajectory(signalThree);
                                        autoState = AutoState.PARK;
                                        break;
                                }
                            }
                        }
                        sleep(20);
                    }
                    break;
                case PARK:
                    break;
            }
            telemetry.addData("detection", detection);


//            drive.update();
            telemetry.addData("etime", timer.time());
            telemetry.update();
        }
    }
}