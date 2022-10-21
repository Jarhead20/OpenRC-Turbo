package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class AutoRedLeft extends OpMode {

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

    TrajectorySequence duck1;// [object][num][Alliance side relative to field][Alliance]
    //<editor-fold desc="Trajectories for going from starting position to signal zones">
    Trajectory signalOneLeftRed = drive.trajectoryBuilder(new Pose2d(-35.67, -60.33, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-59.50, -37.25), Math.toRadians(135.92))
            .build();
    Trajectory signalTwoLeftRed = drive.trajectoryBuilder(new Pose2d(-35.67, -62.00, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-51.17, -54.00), Math.toRadians(152.70))
            .splineTo(new Vector2d(-58.83, -26.83), Math.toRadians(105.76))
            .splineTo(new Vector2d(-59.00, -15.33), Math.toRadians(90.83))
            .splineTo(new Vector2d(-36.83, -10.83), Math.toRadians(11.48))
            .build();
    Trajectory signalThreeLeftRed = drive.trajectoryBuilder(new Pose2d(-35.67, -60.83, Math.toRadians(90.00)))
            .splineTo(new Vector2d(-16.33, -61.50), Math.toRadians(-1.97))
            .splineTo(new Vector2d(-12.67, -36.33), Math.toRadians(81.05))
            .build();

    Trajectory signalOneRightRed = drive.trajectoryBuilder(new Pose2d(36.00, -60.50, Math.toRadians(90.00)))
            .splineTo(new Vector2d(23.25, -60.25), Math.toRadians(178.88))
            .splineTo(new Vector2d(12.50, -36.75), Math.toRadians(95.84))
            .build();
    Trajectory signalTwoRightRed = drive.trajectoryBuilder(new Pose2d(36.00, -60.25, Math.toRadians(90.00)))
            .splineTo(new Vector2d(23.25, -60.25), Math.toRadians(180.00))
            .splineTo(new Vector2d(10.25, -18.25), Math.toRadians(107.20))
            .splineTo(new Vector2d(34.50, -13.00), Math.toRadians(12.22))
            .build();
    Trajectory signalThreeRightRed = drive.trajectoryBuilder(new Pose2d(36.00, -60.25, Math.toRadians(90.00)))
            .splineTo(new Vector2d(59.25, -37.25), Math.toRadians(44.69))
            .build();

    Trajectory signalOneLeftBlue = drive.trajectoryBuilder(new Pose2d(-35.67, 60.83, Math.toRadians(270.00)))
            .splineTo(new Vector2d(-16.33, 61.50), Math.toRadians(361.97))
            .splineTo(new Vector2d(-12.67, 36.33), Math.toRadians(278.95))
            .build();
    Trajectory signalTwoLeftBlue = drive.trajectoryBuilder(new Pose2d(-35.67, 62.00, Math.toRadians(270.00)))
            .splineTo(new Vector2d(-51.17, 54.00), Math.toRadians(207.30))
            .splineTo(new Vector2d(-58.83, 26.83), Math.toRadians(254.24))
            .splineTo(new Vector2d(-59.00, 15.33), Math.toRadians(269.17))
            .splineTo(new Vector2d(-36.83, 10.83), Math.toRadians(348.52))
            .build();
    Trajectory signalThreeLeftBlue = drive.trajectoryBuilder(new Pose2d(-35.67, 60.33, Math.toRadians(270.00)))
            .splineTo(new Vector2d(-59.50, 37.25), Math.toRadians(224.08))
            .build();

    Trajectory signalOneRightBlue = drive.trajectoryBuilder(new Pose2d(36.00, 60.25, Math.toRadians(270.00)))
            .splineTo(new Vector2d(57.50, 39.25), Math.toRadians(315.67))
            .build();
    Trajectory signalTwoRightBlue = drive.trajectoryBuilder(new Pose2d(36.00, 60.25, Math.toRadians(270.00)))
            .splineTo(new Vector2d(23.25, 60.25), Math.toRadians(180.00))
            .splineTo(new Vector2d(10.25, 18.25), Math.toRadians(252.80))
            .splineTo(new Vector2d(34.50, 13.00), Math.toRadians(347.78))
            .build();
    Trajectory signalThreeRightBlue = drive.trajectoryBuilder(new Pose2d(36.00, 60.50, Math.toRadians(270.00)))
            .splineTo(new Vector2d(23.25, 60.25), Math.toRadians(181.12))
            .splineTo(new Vector2d(12.50, 36.75), Math.toRadians(264.16))
            .build();
    //</editor-fold>

    public enum AutoState {
        PARK_CV,
        PARK,
    }
    AutoState autoState = AutoState.PARK;

    public Pose2d startPose;

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
    }

    @Override
    public void loop() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        switch (autoState) {
            case PARK_CV:
                if (detections.size() <= 0) {
                    numFramesWithoutDetection++;
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION){
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        TSEPos = 3;
                    }
                } else {
                    numFramesWithoutDetection = 0;
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    switch(detections.get(0).id){
                        case 10:
                            drive.followTrajectory(signalOneLeftRed);
                            break;
                        case 11:
                            drive.followTrajectory(signalTwoLeftRed);
                            break;
                        case 21:
                            drive.followTrajectory(signalThreeLeftRed);
                            break;
                    }
                }
                autoState = AutoState.PARK;
            case PARK:
                break;
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