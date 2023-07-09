package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class RRPathGenTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(35.29, 66.46, Math.toRadians(180.00)))
                .splineToSplineHeading(new Pose2d(35.13, 46.60, Math.toRadians(185.71)), Math.toRadians(-86.42))
                .splineToSplineHeading(new Pose2d(38.0, 5.00, Math.toRadians(16.00)), Math.toRadians(266.69))
                .build();
        drive.setPoseEstimate(traj.start());

        drive.followTrajectorySequence(traj);
    }
}
