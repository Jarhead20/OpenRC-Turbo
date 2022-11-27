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

    Trajectory traj;

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


//        doCV();


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
            switch (autoState) {
                case PARK_CV:
                    startPose = new Pose2d(-31, 66.18, Math.toRadians(180));
                    drive.setPoseEstimate(startPose);
                    traj = drive.trajectoryBuilder(startPose)
                            .splineTo(new Vector2d(-33.89, 35.10), Math.toRadians(-83.09))
                            .splineToSplineHeading(new Pose2d(-37.10, 4.41, Math.toRadians(164.00)), Math.toRadians(161.24))
                            .build();
                    drive.followTrajectory(traj);
                    autoState = AutoState.PARK;
                    break;
                case PARK:
                    break;
            }


//            drive.update();
            telemetry.addData("etime", timer.time());
            telemetry.update();
        }
    }
}