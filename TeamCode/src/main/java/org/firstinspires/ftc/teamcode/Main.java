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
        MOVE,
        GRIPPER,
        RETURN
    }
    ArmState armState = ArmState.START;
    boolean collect; // set in START, whether arm will connect an object
    double desiredX; // bad
    double desiredY;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        calibrate motors here
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
        /*switch (armState) {
            case armState.START:
                // Waiting for some input
                // just dpad button is open, rBumper is closed
                // closed is left, open is right
                // driver holds rBumper if needed, then presses button
                if (gamepad1.dpad_left) {
                    if (gamepad1.right_bumper) {
                        desiredX = -0.4; // change the numbers
                    } else {
                        desiredX = 0.4;
                    }
                    desiredY = 0.15;
                    collect = true;
                    armState = armState.MOVE;
                } else if (gamepad1.dpad_up) {
                    if (gamepad1.right_bumper) {
                        desiredX = -0.4;
                    } else {
                        desiredX = 0.4;
                    }
                    desiredY = 0.7;
                    armState = armState.MOVE;
                } else if (gamepad1.dpad_right) {
                    if (gamepad1.right_bumper) {
                        desiredX = -0.4;
                    } else {
                        desiredX = 0.4;
                    }
                    desiredY = 0.5;
                    armState = armState.MOVE;
                } else if (gamepad1.dpad_down) {
                    if (gamepad1.right_bumper) {
                        desiredX = -0.4;
                    } else {
                        desiredX = 0.4;
                    }
                    desiredY = 0.3;
                    armState = armState.MOVE;
                }
            if (!Double.isNaN(desiredX) && !Double.isNaN(desiredY)) {
                ArmKinematics(desiredX, desiredY);
                if (collect) { open gripper; }
                 pitch and yaw
            }
            break;

            case armState.MOVE:
                if (currentX != desiredX || currentY != desiredY) {
                    break // change to if it has recorded correct number of ticks
                    pitch and yaw
                }
                if (collect) {
                    close gripper
                } else {
                    open gripper
                }
                armState = armState.GRIPPER;
                currentTime = runtime.now(TimeUnit.MILLISECONDS);
                break;

            case armState.GRIPPER:
                if (runtime.now(TimeUnit.MILLISECONDS) == currentTime + 800) {
//                    ArmKinematics(0, 0);
                    armState = armState.RETURN;
                }
                break;

            case armState.RETURN:
                if (currentX == 0 && currentY == 0) {
             angle not position
                    armState = armState.START;
                }
                break;
            default:
                // should never be reached, as armState should never be null
                armState = armState.START;
        }*/

//        telemetry.addData("arm current", arm.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("arm2 current", arm2.getCurrent(CurrentUnit.AMPS));
        if(!drive.imu.isGyroCalibrated()) return;
        total += gamepad2.left_stick_y*2;
        total2 += gamepad1.right_stick_y*2;

//        arm.setPower(gamepad2.left_stick_y);
//        arm2.setPower(gamepad2.right_stick_y);

        drive.setMultiplier(0.5);
        if (gamepad1.right_bumper)
            drive.setMultiplier(1);
//        telemetry.addData(arm.getTargetPosition() + " ",arm.getCurrentPosition());
//        telemetry.addData(arm2.getTargetPosition() + " ",arm2.getCurrentPosition());
        drive.mecanum(gamepad1);
    }

    @Override
    public void stop() {}

//    public static double[] move(double x, double y, double currentX) {
//        double[] arr = new double[2];
//        if (x * currentX > 0 || x == currentX) {
//            arr[2], arr[3] = getAngles(x, y);
//        } else {
//            double[] arr = getAngles(0, 0.2);
//            double[] arr2 = getAngles(x, y);
//        }
//    }

    public static double ticksToAngles(double ticks) {
        return ticks / 8;
    }
    public static double anglesToTicks(double angles) {
        return angles * 8;
    }
}