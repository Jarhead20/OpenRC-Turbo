package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative OpMode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime;
//    private Drive drive;
    private SampleMecanumDrive drive;
    private StandardTrackingWheelLocalizer localizer;
    private Arm arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
//        drive = new Drive(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap, telemetry, runtime);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    @Override
    public void init_loop() {
        try{
            arm.initLoop();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //use this on competition day
        //if(runtime.seconds() >= 120) return;
        arm.inputGamepad(gamepad2);
        if (gamepad1.right_bumper) drive.setMotorMultiplier(1);
        else drive.setMotorMultiplier(0.5);

        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();


    }

    @Override
    public void stop() {}

}