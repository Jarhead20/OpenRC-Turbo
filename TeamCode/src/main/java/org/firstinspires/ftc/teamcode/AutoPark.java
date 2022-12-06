package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(group = "drive")
public class AutoPark extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public SampleMecanumDrive drive;
    public Drive d;

    Trajectory signalOne;
    Trajectory signalTwo;
    Trajectory signalThree;

    public enum AutoState {
        PARK_CV,
        PARK,
    }
    AutoState autoState = AutoState.PARK_CV;

    public Pose2d startPose;

    ElapsedTime timer = new ElapsedTime();

    int cameraMonitorViewId;

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
//        signalOne = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(-90.00)))
//                .splineTo(new Vector2d(-13.38, 34.38), Math.toRadians(171.87))
//                .build();
//        signalTwo = drive.trajectoryBuilder(new Pose2d(-0.23, 0, Math.toRadians(-90.00)))
//                .splineTo(new Vector2d(0.69, 34.62), Math.toRadians(88.99))
//                .build();
//        signalThree = drive.trajectoryBuilder(new Pose2d(0.23, 0, Math.toRadians(-90.00)))
//                .splineTo(new Vector2d(11.31, 35.08), Math.toRadians(-1.85))
//                .build();

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
//                                        signalOne = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(-90.00)))
////                                                .splineTo(new Vector2d(34.38,-24), Math.toRadians(90))
////                                                .build();
//                                                .strafeLeft(30)
////                                                .turn(Math.toRadians(-90))
//                                                .back(24)
//                                                .turn(Math.toRadians(90))
//                                                .forward(10)
//                                                .build();
                                        signalOne = drive.trajectoryBuilder(new Pose2d(0.00, -0.00, Math.toRadians(1.97)))
                                                .splineTo(new Vector2d(30.98, 8.74), Math.toRadians(89.08))
                                                .splineTo(new Vector2d(38.49, 21.74), Math.toRadians(-6.71))
                                                .build();
                                        drive.followTrajectory(signalOne);
                                        autoState = AutoState.PARK;
                                        break;
                                    case 11:
                                        telemetry.addData("Signal", "2");
//                                        signalTwo = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(-90.00)))
////                                                .splineTo(new Vector2d(34.62, 0), Math.toRadians(0))
//                                                .strafeLeft(30)
////                                                .turn(Math.toRadians(-90))
//                                                .turn(Math.toRadians(90))
//                                                .forward(10)
//                                                .build();

                                        signalTwo = drive.trajectoryBuilder(new Pose2d(-0.07, 0.22, Math.toRadians(-0.50)))
                                                .splineTo(new Vector2d(40.00, -0.00), Math.toRadians(6.01))
                                                .build();

                                        drive.followTrajectory(signalTwo);
                                        autoState = AutoState.PARK;
                                        break;
                                    case 21:
                                        telemetry.addData("Signal", "3");
//                                        signalThree = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(-90.00)))
////                                                .splineTo(new Vector2d(35.08,24), Math.toRadians(-90))
////                                                .turn(Math.toRadians(90))
//                                                .strafeLeft(30)
////                                                .turn(Math.toRadians(-90))
//                                                .forward(24)
//                                                .turn(Math.toRadians(90))
//                                                .forward(10)
//                                                .build();

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