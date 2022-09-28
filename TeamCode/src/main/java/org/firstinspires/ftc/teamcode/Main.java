package org.firstinspires.ftc.teamcode;

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

import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;

//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative OpMode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Arm arm;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        drive = new Drive(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //use this on competition day
        //if(runtime.seconds() >= 120) return;
        if(!drive.imu.isGyroCalibrated()) return;

        arm.move(gamepad2);
        arm.update();
        drive.setMultiplier(0.5);
        if (gamepad1.right_bumper) drive.setMultiplier(1);
        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}
}