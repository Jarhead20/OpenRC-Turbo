package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class TestAuto extends OpMode {

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
    public Drive d;

    Trajectory deposit;
    TrajectorySequence duck1;
    Trajectory duck2;

    public enum State {
        START,
        EXTEND,
        RETRACT
    }

    public Pose2d startPose;

    State state = State.START;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        d = new Drive(hardwareMap, telemetry);
        d.setup();

        startPose = new Pose2d(-60, -60, Math.toRadians(180));
//        startPose = new Pose2d(-60, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        doCV();

        telemetry.addData("Realtime analysis", TSEPos);
//        Pose2d test = new Pose2d(-53.2,41.6, Math.toRadians(-79)), Math.toRadians(-79);
        deposit = drive.trajectoryBuilder(startPose, true)

                .splineTo(new Vector2d(-10, -40), Math.toRadians(90))
                .addDisplacementMarker(() -> state = State.EXTEND)
                .build();

        duck1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .strafeLeft(4)

                //.splineTo(new Vector2d(-70, -60), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                })

                .back(10)
                .strafeLeft(4)
                .back(100)
                //.splineToConstantHeading(new Vector2d(55, -65), Math.toRadians(180))
                .build();
    }

    @Override
    public void loop() {
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

        drive.followTrajectorySequence(duck1);
        //drive.followTrajectory(deposit);

    }

            //drive.followTrajectorySequence(duck1);

//            switch (state){
//                case START:
//
//
//                    drive.followTrajectory(deposit);
//                    break;
//                case EXTEND:
//                    switch(TSEPos){
//                        case 1:
//                            d.slideDrive.setTargetPosition(-3500);
//                            d.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            d.slideDrive.setPower(1);
//                            if(d.slideDrive.getCurrentPosition() >= -3400)  {
//                                d.ramp.setPosition(0.4);
//                                d.servo.setPosition(0);
//                            }
//                            else {
//                                d.ramp.setPosition(0.5);
//                                d.servo.setPosition(0.5);
//                            }
//                            break;
//                        case 2:
//                            d.slideDrive.setTargetPosition(-3500);
//                            d.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            d.slideDrive.setPower(1);
//                            if(d.slideDrive.getCurrentPosition() >= -3400)  {
//                                d.ramp.setPosition(0.2);
//                                d.servo.setPosition(0);
//                            }
//                            else {
//                                d.ramp.setPosition(0.5);
//                                d.servo.setPosition(0.5);
//                            }
//                            break;
//                        case 3:
//
//                            d.slideDrive.setTargetPosition(-4400);
//                            d.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            d.slideDrive.setPower(1);
//                            if(d.slideDrive.getCurrentPosition() <= -4300) {
//                                d.ramp.setPosition(0.5);
//                                d.servo.setPosition(0);
//                            }
//                            else if(d.slideDrive.getCurrentPosition() >= -400) d.servo.setPosition(0.5);
//                            else {
//                                d.ramp.setPosition(0.5);
//                                d.servo.setPosition(0.5);
//                            }
//                            break;
//                        default:
//                            break;
//                    }
//
//                    if(d.slideDrive.getCurrentPosition() >  d.slideDrive.getTargetPosition()-50 && d.slideDrive.getCurrentPosition() < d.slideDrive.getTargetPosition() + 50) {
//                        timer.reset();
//                        state = State.RETRACT;
//                    }
//                    break;
//
//                case RETRACT:
//                    d.slideDrive.setTargetPosition(-1000);
//    //                d.position = 4;
//    //                if(d.slideDrive.getCurrentPosition() >  d.slideDrive.getTargetPosition()-50 && d.slideDrive.getCurrentPosition() < d.slideDrive.getTargetPosition() + 50)
//    //                    drive.followTrajectorySequence(duck1);
//                    break;
//                default:
//                    break;
//            }


        //d.slide(d.position);

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