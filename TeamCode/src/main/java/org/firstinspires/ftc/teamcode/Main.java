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
    private int total = 0;
    private int total2 = 0;
    private double speed = 0;

    /* code to move the arm (once an object is collected) to its desired position (height of the
    highest pole), then let go of the object and reset to Collection state
    allow the driver to press button to stop machine and arm movement
    simultaneously control the drive train
    */

    public enum ArmState {
        START,
        MOVE_MIDDLE, // when the desiredX is on the opposite side to currentX, moves to (0, 0.4) then desired
        MOVE,
        GRIPPER,
        RETURN
    }
    ArmState armState = ArmState.START;
    boolean collect; // set in START, whether arm will connect an object
    double desiredX;
    double desiredY;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
//        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
//        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();
//        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm1.setTargetPosition(0);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        total2 += gamepad1.right_stick_y*2;

//        arm1.setPower(gamepad2.left_stick_y);
//        arm2.setPower(gamepad2.right_stick_y);

        drive.setMultiplier(0.5);
        if (gamepad1.right_bumper)
            drive.setMultiplier(1);
//        telemetry.addData(arm1.getTargetPosition() + " ",arm1.getCurrentPosition());
//        telemetry.addData(arm2.getTargetPosition() + " ",arm2.getCurrentPosition());
        drive.mecanum(gamepad1);

        arm.move(gamepad2);
    }

    @Override
    public void stop() {}

}