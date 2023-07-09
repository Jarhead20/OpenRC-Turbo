package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Vector2;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@Autonomous(name = "AutoRight", group = "drive")
public class AutoRight extends AutoOpMoving {
    @Override
    protected void setupTrajectories() {
        traj = drive.trajectoryBuilder(new Pose2d(-29.33, 65.30, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-36.16, 38.61), Math.toRadians(267.81))
                .splineToSplineHeading(new Pose2d(-34.36, 3.29, Math.toRadians(173.00)), Math.toRadians(-84.38))
                .build();
        park3 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-36.68, 12.96), Math.toRadians(166.66))
                .splineTo(new Vector2d(-61.04, 20.18), Math.toRadians(91.91))
                .build();
        park2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-35.39, 12.05), Math.toRadians(90.00))
                .splineTo(new Vector2d(-35.52, 27.27), Math.toRadians(90.00))
                .build();
        park1 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-31.65, 11.92), Math.toRadians(2.16))
                .splineTo(new Vector2d(-11.92, 19.79), Math.toRadians(88.73))
                .build();
    }

    public AutoRight() {
        super(-50, 215);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initComponents();

        waitForStart();
        timer.reset();
        telemetry.setMsTransmissionInterval(50);
        drive.setPoseEstimate(traj.start());

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
                    arm.setPower(0.3, 0.5);
                    arm.moveTo(new Vector2(-20, 830));
                    if(arm.atTarget(new Vector2(-20, 830))) {
                        autoState = AutoState.PARK1;
                    }
                    break;
                case PARK1:

                    drive.followTrajectory(traj);
//                    arm.moveTo(236, 768);
                    //up 236, 768
                    //down -620, 300
                    if(!drive.isBusy()){
                        arm.moveTo(new Vector2(depositLoc.x-30, 800));
                        autoState = AutoState.PRELOAD;
                        timer3.reset();

                    }
                    break;
                case PRELOAD:
                    if(arm.atTarget(new Vector2(depositLoc.x-30, 800), 7)){
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
                    arm.shoulderMotor.setPower(0.5);
                    arm.elbowMotor.setPower(0.5);
                    arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(!arm.elbowMotor.isBusy() && !arm.shoulderMotor.isBusy()) {
                        switch (detection) {
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
}