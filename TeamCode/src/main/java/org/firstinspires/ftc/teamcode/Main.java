package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative Opmode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private DcMotor arm = null;
    private int total = 0;
    private double speed = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        arm = hardwareMap.get(DcMotor.class, "arm");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        if(!drive.imu.isGyroCalibrated()) return;
        total += gamepad2.left_stick_y*2;

        arm.setPower(gamepad2.left_stick_y);
        drive.setMultiplier(0.5);
        if(gamepad1.right_bumper)
            drive.setMultiplier(1);
        telemetry.addData(arm.getTargetPosition() + " ",arm.getCurrentPosition());
        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}
}