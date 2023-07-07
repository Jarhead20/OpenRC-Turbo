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
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Vector2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")
public class AutoLeft extends AutoOpMoving {
    Trajectory traj;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    public enum AutoState {
        START,
        PARK1,
        ARMUP,
        PRELOAD,
        CYCLE1,
        CYCLE2,
        CYCLE3,
        CYCLE4,
        PARK2,
        STOP,
    }
    AutoState autoState = AutoState.START;

    public AutoLeft() {
        super(-110, 200);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap, telemetry, timer);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        traj = drive.trajectoryBuilder(new Pose2d(35.29, 66.46, Math.toRadians(180.00)))
                .splineToSplineHeading(new Pose2d(35.13, 46.60, Math.toRadians(185.71)), Math.toRadians(-86.42))
                .splineToSplineHeading(new Pose2d(38.0, 5.00, Math.toRadians(16.00)), Math.toRadians(266.69))
                .build();
        park3 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(33.07, 11.41), Math.toRadians(169.09))
                .splineTo(new Vector2d(12.57, 24.04), Math.toRadians(90.00))
                .build();
        park2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(36.93, 12.18), Math.toRadians(94.86))
                .splineTo(new Vector2d(36.16, 24.82), Math.toRadians(100.01))
                .build();
        park1 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(38.09, 12.57), Math.toRadians(10.92))
                .splineTo(new Vector2d(61.43, 23.27), Math.toRadians(86.76))
                .build();
        arm.closeGripper();

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
        timer.reset();

        telemetry.setMsTransmissionInterval(50);


        int detection = 0;
        while (opModeIsActive() && timer.time() <= 30) {
            if(isStopRequested()) return;
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
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
                        arm.moveTo(new Vector2(depositLoc.x-50, 800));
                        autoState = AutoState.PRELOAD;
                        timer3.reset();

                    }
                    break;
                case PRELOAD:
                    if(arm.atTarget(new Vector2(depositLoc.x-50, 800), 7)){
                        if(timer3.seconds() > 1){
                            arm.moveTo(depositLoc);
                            autoState = AutoState.CYCLE1;
                        }
                    }
                    break;

                case CYCLE1:
                    if(cycle(0))
                        autoState = AutoState.CYCLE2;
                    break;
                case CYCLE2:
                    if(cycle(downAmount))
                        autoState = AutoState.CYCLE3;
                    break;
                case CYCLE3:
                    if(cycle(downAmount*2))
                        autoState = AutoState.CYCLE4;
                    break;
                case CYCLE4:
                    if(cycle(downAmount*3))
                        autoState = AutoState.PARK2;
                    break;
                case PARK2:

                    arm.openGripper();
                    arm.shoulderMotor.setTargetPosition(0);
                    arm.elbowMotor.setTargetPosition(0);
                    arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.shoulderMotor.setPower(0.5);
                    arm.elbowMotor.setPower(0.5);
                    if(!arm.elbowMotor.isBusy() && !arm.shoulderMotor.isBusy()){
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
                                case 9:
                                    break;
                            }
                        autoState = AutoState.STOP;
                    }
                    break;
                case STOP:
                    telemetry.addData("parking", "parking");
                    break;

            }


//            drive.update();
            arm.reportCurrentPosition();
            telemetry.addData("etime", timer.time());
            telemetry.update();
        }
    }

    public enum Cycles {
        DEPOSIT,
        DOWN,
        GRAB,
        UP,
    }

    public Cycles cycleState = Cycles.DEPOSIT;

    boolean oneTime = false;

    public boolean cycle(double lower){
        switch (cycleState){
            case DEPOSIT:
                if(arm.atTarget(depositLoc) && !oneTime){
                    timer2.reset();
                    oneTime = true;
                }

                if(timer2.seconds() > 0.2 && oneTime){
                    arm.openGripper();
                    if(timer2.seconds() > 0.5){
                        arm.moveTo(pickup1.lower(lower));
                        cycleState = Cycles.DOWN;
                    }
                }
                break;
            case DOWN:
                if(arm.atTarget(pickup1.lower(lower), 7)){
                    arm.moveTo(pickupGrab.lower(lower));
                    oneTime = false;
                    cycleState = Cycles.GRAB;
                }
                break;
            case GRAB:
                if(arm.atTarget(pickupGrab.lower(lower), 7) && !oneTime){
                    oneTime = true;
                    timer2.reset();
                }

                if(timer2.seconds() > 0.2 && oneTime){
                    arm.closeGripper();
                    if(timer2.seconds() > 0.5){
                        arm.moveTo(pickupUp);
                        oneTime = false;
                        cycleState = Cycles.UP;
                    }
                }
                break;
            case UP:
                if(arm.atTarget(pickupUp, 50)){
                    arm.moveTo(depositLoc);
                }
                if(arm.atTarget(depositLoc)) {
                    cycleState = Cycles.DEPOSIT;
                    return true;
                }
                break;
        }
        return false;
    }

    private boolean inRange(ElapsedTime timer, double time){
        double low = time-0.1;
        double high = time+0.1;
        return (timer.seconds() >= low && timer.seconds() <= high);
    }
}