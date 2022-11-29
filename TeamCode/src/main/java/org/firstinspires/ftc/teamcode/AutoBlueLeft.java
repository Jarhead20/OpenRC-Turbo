package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Disabled
@Autonomous(group = "drive")
public class AutoBlueLeft extends LinearOpMode {


    public SampleMecanumDrive drive;
    public Drive d;

    public enum AutoState {
        DRIVE,
        PARK,
    }
    AutoState autoState = AutoState.DRIVE;


    public Pose2d startPose;

    ElapsedTime timer = new ElapsedTime();
    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap, telemetry, timer);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive() && timer.time() < 30) {
            if(isStopRequested()) return;
            arm.moveTo(-130, 600);
            switch (autoState) {
                case DRIVE:
                    startPose = new Pose2d(-30.55, 65.07, Math.toRadians(180));
                    drive.setPoseEstimate(startPose);
                    TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-36.04, 38.78, Math.toRadians(236.31)), Math.toRadians(260.54))
                            .splineTo(new Vector2d(-33.29, 2.53), Math.toRadians(-12.85))
                            .build();
                    drive.followTrajectorySequence(untitled0);
                        autoState = AutoState.PARK;
                    break;
                case PARK:
                    break;
            }

            telemetry.addData("etime", timer.time());
            telemetry.update();
        }
    }
}