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
    private double currentTime;
    private Drive drive;
    private Arm arm;

    /* code to move the arm (once an object is collected) to its desired position (height of the
    highest pole), then let go of the object and reset to Collection state
    allow the driver to press button to stop machine and arm movement
    simultaneously control the drive train
    */



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
//        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
//        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        drive = new Drive(hardwareMap,telemetry);
        arm = new Arm(hardwareMap, telemetry);

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
//        switch (armState) {
//            case START:
//                // Waiting for some input
//                // just dpad button is open, rBumper is closed
//                // closed is left, open is right
//                // driver holds rBumper if needed, then presses button
//                if (gamepad1.dpad_left) {
//                    if (gamepad1.right_bumper) {
//                        desiredX = -0.4; // change the numbers
//                    } else {
//                        desiredX = 0.4;
//                    }
//                    desiredY = 0.15;
//                    collect = true;
//                    armState = ArmState.MOVE;
//                } else if (gamepad1.dpad_up) {
//                    if (gamepad1.right_bumper) {
//                        desiredX = -0.4;
//                    } else {
//                        desiredX = 0.4;
//                    }
//                    desiredY = 0.7;
//                    armState = ArmState.MOVE;
//                } else if (gamepad1.dpad_right) {
//                    if (gamepad1.right_bumper) {
//                        desiredX = -0.4;
//                    } else {
//                        desiredX = 0.4;
//                    }
//                    desiredY = 0.5;
//                    armState = ArmState.MOVE;
//                } else if (gamepad1.dpad_down) {
//                    if (gamepad1.right_bumper) {
//                        desiredX = -0.4;
//                    } else {
//                        desiredX = 0.4;
//                    }
//                    desiredY = 0.3;
//                    armState = ArmState.MOVE;
//                }
//                if (!Double.isNaN(desiredX) && !Double.isNaN(desiredY)) {
//                    if ((drive.getArm1().getCurrentPosition() * 8 - 90) * (drive.getArm1().getCurrentPosition() * 8 - 90) < 0) {
//                        // if desired and current on opposite sides
//                        drive.armIk(0, 0.4, true);
//                        armState = armState.MOVE_MIDDLE;
//                    } else {
//                        drive.armIk(desiredX, desiredY, true);
//                        if (collect) {
////                            open gripper;
//                        }
//                    }
//                }
//                break;
//
//            case MOVE_MIDDLE:
//                if (drive.getArm1().getCurrentPosition() != drive.getArm1().getTargetPosition()) {
//                    break;
//                    roll
//                } else if (drive.getArm2().getCurrentPosition() != drive.getArm2().getTargetPosition()) {
//                    break;
//                }
//                drive.armIk(desiredX, desiredY, true);
//                if (collect) {
//                    open gripper;
//                }
//                armState = ArmState.MOVE;
//                break;
//
//            case MOVE:
//                if (drive.getArm1().getCurrentPosition() != drive.getArm1().getTargetPosition()) {
//                    break;
//                    roll
//                } else if (drive.getArm2().getCurrentPosition() != drive.getArm2().getTargetPosition()) {
//                    break;
//                }
//                if (collect) {
//                    close gripper
//                } else {
//                    open gripper
//                }
//                armState = ArmState.GRIPPER;
//                currentTime = runtime.now(TimeUnit.MILLISECONDS);
//                break;
//
//            case GRIPPER:
//                if (runtime.now(TimeUnit.MILLISECONDS) == currentTime + 200) {
//                    drive.armIk(desiredX, desiredY, true);
//                    armState = ArmState.RETURN;
//                }
//                break;
//
//            case RETURN:
//                if (drive.getArm1().getCurrentPosition() != drive.getArm1().getTargetPosition()) {
//                    break;
//                }
//                if (drive.getArm2().getCurrentPosition() != drive.getArm2().getTargetPosition()) {
//                    break;
//                }
//                armState = ArmState.START;
//            default:
//                // should never be reached, as armState should never be null
//                armState = ArmState.START;
//        }
//
//        if(!drive.imu.isGyroCalibrated()) return;
//        total += gamepad2.left_stick_y*2;
//        total2 += gamepad1.right_stick_y*2;

//        arm1.setPower(gamepad2.left_stick_y);
//        arm2.setPower(gamepad2.right_stick_y);

        arm.move(gamepad2);
        drive.setMultiplier(0.5);
        if (gamepad1.right_bumper)
            drive.setMultiplier(1);
//        telemetry.addData(arm1.getTargetPosition() + " ",arm1.getCurrentPosition());
//        telemetry.addData(arm2.getTargetPosition() + " ",arm2.getCurrentPosition());
        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}

}