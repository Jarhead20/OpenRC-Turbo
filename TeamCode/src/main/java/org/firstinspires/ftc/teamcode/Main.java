package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative OpMode")
public class Main extends OpMode
{
    private final ElapsedTime runtime = new ElapsedTime();
    private Drive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
//        drive = new Drive(hardwareMap,telemetry);
        drive = new Drive(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) drive.setMultiplier(1);
        else drive.setMultiplier(0.5);

        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}

}