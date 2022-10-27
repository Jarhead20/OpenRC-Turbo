package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@Autonomous(group = "drive")
public class TestAuto extends LinearOpMode {

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
    public int TSEPos = 3;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public SampleMecanumDrive drive;


    public enum State {
        START,
        EXTEND,
        RETRACT
    }


    public Pose2d startPose;

    State state = State.START;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode(){
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        startPose = new Pose2d(-60, -60, Math.toRadians(180));
//        startPose = new Pose2d(-60, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        doCV();
        while(!isStarted() && !isStarted()){
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                if (detections.size() == 0) {
                    numFramesWithoutDetection++;
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION){
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        TSEPos = 3;
                    }
                } else {
                    numFramesWithoutDetection = 0;
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    if (detections.size() > 0) {


                        AprilTagDetection detection = detections.get(0);

                        if (detection.pose.x < 0) TSEPos = 1;
                        else if (detection.pose.x >= 0) TSEPos = 2;
                        telemetry.addData("test",detection.pose.x);
                    }
                }
            }
            telemetry.addData("TSE", TSEPos);
            telemetry.update();
        }

        waitForStart();



        telemetry.addData("Realtime analysis", TSEPos);





        if(isStopRequested()) return;




        while(!isStopRequested()){
            Trajectory untitled0 = drive.trajectoryBuilder(new Pose2d(-18.37, -40.00, Math.toRadians(29.05)))
                    .splineTo(new Vector2d(33.19, 3.70), Math.toRadians(129.98))
                    .splineTo(new Vector2d(-13.78, 42.22), Math.toRadians(227.28))
                    .splineTo(new Vector2d(-46.37, -8.30), Math.toRadians(237.17))
                    .splineTo(new Vector2d(-22.81, -21.63), Math.toRadians(5.66))
                    .build();

        }

    }

    public void doCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN); FtcDashboard.getInstance().startCameraStream(camera, 500); }
            @Override
            public void onError(int errorCode) {}
        });


    }
}