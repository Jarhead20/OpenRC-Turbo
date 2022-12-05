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
public class AutoBlueRight extends LinearOpMode {

    public SampleMecanumDrive drive;
    public Drive d;

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

    Trajectory traj;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    public enum AutoState {
        START,
        PARK1,
        ARMUP,
        DEPOSIT,
        PICK1,
        DEPOSIT2,
        PICK2,
        DEPOSIT3,
        PICK3,
        DEPOSIT4,
        PICK4,
        DEPOSIT5,
        PARK2,
        STOP,
    }
    AutoState autoState = AutoState.START;
    int cameraMonitorViewId;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    Arm arm;
    int offset = -50;
    int pickHeight = 180;
    Vector2 pickup1 = new Vector2(-400+offset, pickHeight);
    Vector2 pickupGrab = new Vector2(-600+offset, pickHeight);
    Vector2 pickupUp = new Vector2(-500+offset, 400);
    Vector2 depositLoc = new Vector2(170, 690);

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap, telemetry, timer);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        traj = drive.trajectoryBuilder(new Pose2d(-37.42, 66.46, Math.toRadians(180.00)))
                .splineTo(new Vector2d(-36.29, 50.21), Math.toRadians(-83.09))
                .splineToSplineHeading(new Pose2d(-35.00, 4, Math.toRadians(168)), Math.toRadians(-84.38))
                .build();
        park3 = drive.trajectoryBuilder(traj.end())
                .splineToConstantHeading(new Vector2d(-37.58, 35.77), Math.toRadians(195.95))
                .splineTo(new Vector2d(-59.75, 25.07), Math.toRadians(-74.74))
                .build();
        park2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-36.29, 38.35), Math.toRadians(99.46))
                .build();
        park1 = drive.trajectoryBuilder(traj.end())
                .splineToConstantHeading(new Vector2d(-28.94, 35.65), Math.toRadians(18.29))
                .splineTo(new Vector2d(-10.64, 23.79), Math.toRadians(-74.74))
                .build();
        arm.closeGripper();
        waitForStart();
        timer.reset();

        telemetry.setMsTransmissionInterval(50);

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
        int detection = 0;
        while (opModeIsActive() && timer.time() < 30) {
            if(isStopRequested()) return;
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            telemetry.addData("detection", detection);
            switch (autoState) {
                case START:
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
                            }
                        }
                    }
                    if(detection != 0)
                        autoState = AutoState.ARMUP;
                    break;
                case ARMUP:
                    arm.setPower(0.3, 0.6);
                    arm.moveTo(new Vector2(-20, 830));
                    if(arm.atTarget(new Vector2(-20, 830))) {
                        autoState = AutoState.PARK1;
                    }
                    break;
                case PARK1:
                    drive.setPoseEstimate(traj.start());
                    drive.followTrajectory(traj);
//                    arm.moveTo(236, 768);
                    //up 236, 768
                    //down -620, 300
                    if(!drive.isBusy()){
                        arm.moveTo(new Vector2(depositLoc.x-30, 800));
                        autoState = AutoState.DEPOSIT;
                        timer2.reset();
                    }
                    break;
                case DEPOSIT:
                    if(arm.atTarget(depositLoc)){
                        arm.openGripper();
                        arm.moveTo(pickup1);
                        autoState = AutoState.PICK1;
                        break;
                    }

                    if(arm.atTarget(new Vector2(depositLoc.x-30, 800)))
                        if(timer2.seconds() > 2)
                            arm.moveTo(depositLoc);

                    break;
                case PICK1:
                    if(arm.atTarget(pickupGrab)){
                        arm.closeGripper();
                        autoState = AutoState.DEPOSIT2;
                        timer2.reset();
                        break;
                    }
                    if(arm.atTarget(pickup1)) {
                        arm.moveTo(pickupGrab);
                    }
                    break;
                case DEPOSIT2:
                    if(arm.atTarget(depositLoc)){
                        arm.openGripper();
                        arm.moveTo(pickup1.lower(20));
                        autoState = AutoState.PICK2;
                        break;
                    }
                    if(arm.atTarget(pickupUp))
                        arm.moveTo(depositLoc);
                    if(timer2.seconds() > 0.5 && timer2.seconds() < 0.7)
                        arm.moveTo(pickupUp);
                    break;
                case PICK2:
                    if(arm.atTarget(pickupGrab.lower(20))){
                        arm.closeGripper();
                        timer2.reset();
                        autoState = AutoState.DEPOSIT3;
                        break;
                    }
                    if(arm.atTarget(pickup1.lower(20))) {
                        arm.moveTo(pickupGrab.lower(20));
                    }
                    break;
                case DEPOSIT3:
                    if(arm.atTarget(depositLoc)){
                        arm.openGripper();
                        arm.moveTo(pickup1.lower(50));
                        autoState = AutoState.PICK3;
                        break;
                    }

                    if(arm.atTarget(pickupUp))
                        arm.moveTo(depositLoc);
                    if(timer2.seconds() > 0.1 && timer2.seconds() < 0.2)
                        arm.moveTo(pickupUp);

                case PICK3:
                    if(arm.atTarget(pickupGrab.lower(50))){
                        arm.closeGripper();
                        autoState = AutoState.DEPOSIT4;
                        timer2.reset();
                        break;
                    }
                    if(arm.atTarget(pickup1.lower(50))) {
                        arm.moveTo(pickupGrab.lower(50));
                    }
                    break;
                case DEPOSIT4:
                    if(arm.atTarget(depositLoc)){
                        arm.openGripper();
                        autoState = AutoState.PICK4;
                        break;
                    }
                    if(arm.atTarget(pickupUp))
                        arm.moveTo(depositLoc);
                    else if(timer2.seconds() > 0.1 && timer2.seconds() < 0.2)
                        arm.moveTo(pickupUp);


                    break;
                case PICK4:
                    if(arm.atTarget(pickupGrab.lower(80))){
                        arm.closeGripper();
                        autoState = AutoState.DEPOSIT5;
                        timer2.reset();
                        break;
                    }
                    if(arm.atTarget(pickup1.lower(80))) {
                        arm.moveTo(pickupGrab.lower(80));
                    }
                    break;
                case DEPOSIT5:
                    if(arm.atTarget(depositLoc)){
                        arm.openGripper();
                        autoState = AutoState.PARK2;
                        break;
                    }
                    if(arm.atTarget(pickupUp))
                        arm.moveTo(depositLoc);
                    else if(timer2.seconds() > 0.1 && timer2.seconds() < 0.2)
                        arm.moveTo(pickupUp);


                    break;
                case PARK2:
                    switch (detection){
                        case 10:
                            drive.followTrajectory(park1);
                            break;
                        case 11:
                            drive.followTrajectory(park2);
                            break;
                        case 21:
                            drive.followTrajectory(park3);
                            break;
                    }
                    autoState = AutoState.STOP;

            }


//            drive.update();
            arm.reportCurrentPosition();
            telemetry.addData("etime", timer.time());
            telemetry.update();
        }
    }
}