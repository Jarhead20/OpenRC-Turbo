package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Arm;

import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;

//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative OpMode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime;
    private Drive drive;
    private Arm arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        drive = new Drive(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry, runtime);
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
        drive.setMultiplier(0.5);
        if (gamepad1.right_bumper)
            drive.setMultiplier(1);
        drive.mecanum(gamepad1);

        arm.move(gamepad2);
    }

    @Override
    public void stop() {}

}