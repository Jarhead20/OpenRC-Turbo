package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
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

    public int TSEPos = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public SampleMecanumDrive drive;
    public Drive d;

    Trajectory deposit;
    TrajectorySequence ducks;



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        drive = new SampleMecanumDrive(hardwareMap);
        d = new Drive(hardwareMap, telemetry);
        d.setup();



//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
//                FtcDashboard.getInstance().startCameraStream(camera, 500);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });


            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.



//
//        deposit = drive.trajectoryBuilder(new Pose2d(), true)
//                .addDisplacementMarker(() -> d.position = TSEPos)
//                .splineTo(new Vector2d(0,-40),Math.toRadians(120))
//                .addDisplacementMarker(() -> {
//                    d.position = 1;
//                    drive.followTrajectoryAsync(ducks);
//                })
//                .build();
        Pose2d pose = new Pose2d(-60, -60, Math.toRadians(180));
        drive.setPoseEstimate(pose);


        waitForStart();

        if (isStopRequested()) return;
        //while (!isStopRequested()) {
        ducks = drive.trajectorySequenceBuilder(pose)
                .forward(10)
                .strafeLeft(4)

                //.splineTo(new Vector2d(-70, -60), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    try {
                        d.getDuck().setPower(-0.2);
                        Thread.sleep(6000);
                        d.getDuck().setPower(0);
                    } catch (InterruptedException e) { e.printStackTrace(); }
                })
                .back(10)
                .strafeLeft(4)
                .back(100)
                //.splineToConstantHeading(new Vector2d(55, -65), Math.toRadians(180))
                .build();
//            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(pose)
//                    .forward(10)
//                    .build();
            drive.followTrajectorySequence(ducks);
        //}
    }


//        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//
//        // If there's been a new frame...
//        if(detections != null)
//        {
//            telemetry.addData("FPS", camera.getFps());
//            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//            // If we don't see any tags
//            if(detections.size() == 0)
//            {
//                numFramesWithoutDetection++;
//                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//            }
//            else
//            {
//                numFramesWithoutDetection = 0;
//                // If the target is within 1 meter, turn on high decimation to
//                // increase the frame rate
//                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                for(AprilTagDetection detection : detections)
//                {
//                    if(detection.pose.x < 0) TSEPos = 1;
//                    else if(detection.pose.x > 1 && detection.pose.x > 1) TSEPos = 3;
//                    else if(detection.pose.x < 1 && detection.pose.x > 0) TSEPos = 2;
//                    if(TSEPos != 0) break;
//                }
//            }
//            telemetry.update();
//        }
//        telemetry.addData("TSE", TSEPos);
//        drive.update();
//        d.slide(d.position);

}

