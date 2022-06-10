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

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();

//        drive.slideDrive.setTargetPosition(-10000);
//        drive.slideDrive.setPower(0.5);
//        drive.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            System.out.println(drive.slideDrive.getCurrentPosition());
//            telemetry.addData("Status", "Waiting for the motor to reach its target");
//            telemetry.update();
//            if(drive.slideDrive.getCurrentPosition() <= -9900) {
//                drive.servo.setPosition(0);
//            }
//            else if(drive.slideDrive.getCurrentPosition() >= -1600) drive.servo.setPosition(0.5);
//
//            else drive.servo.setPosition(0.3);

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

        drive.setMultiplier(1-gamepad1.right_trigger);
        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}
}