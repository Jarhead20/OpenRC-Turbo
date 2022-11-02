package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.usb.UsbSerialNumber;

import java.text.DecimalFormat;

public class Drive {
    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private double multiplier = 1;

    public Drive(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;
        leftFrontDrive = map.get(DcMotor.class, "motor0");
        rightFrontDrive = map.get(DcMotor.class, "motor1");
        leftBackDrive = map.get(DcMotor.class, "motor2");
        rightBackDrive = map.get(DcMotor.class, "motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void mecanum(Gamepad gamepad) {

        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y) * multiplier;
        double robotAngle = Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        telemetry.addData("angle", robotAngle);
        double rightX = gamepad.right_stick_x/3;
        double[] v = new double[4];
        v[0] = r * Math.cos(robotAngle) + rightX;
        v[1] = r * Math.sin(robotAngle) - rightX;
        v[2] = r * Math.sin(robotAngle) + rightX;
        v[3] = r * Math.cos(robotAngle) - rightX;
//        double max = 0;
//        for (int i = 0; i < 4; i++) {
//            if (Math.abs(v[i]) > max) max = Math.abs(v[i]);
//        }
//        max = Range.clip(max, -1, 1);
//        DecimalFormat df = new DecimalFormat("0.00");
//        telemetry.addData("Status", "max: " + max);
//        for (int i = 0; i < 4; i++) {
//            double e = v[i] / max;
//            telemetry.addData("Status", df.format(e) + " " + df.format(v[i]) + " " + df.format(max));
//            v[i] = e;
//        }
        telemetry.addData("0", v[0]);
        telemetry.addData("1", v[1]);
        telemetry.addData("2", v[2]);
        telemetry.addData("3", v[3]);
        leftFrontDrive.setPower(v[0]);
        rightFrontDrive.setPower(v[1]);
        leftBackDrive.setPower(v[2]);
        rightBackDrive.setPower(v[3]);
    }

    public DcMotor getLeftFrontDrive() {
        return leftFrontDrive;
    }

    public DcMotor getRightFrontDrive() {
        return rightFrontDrive;
    }

    public DcMotor getLeftBackDrive() {
        return leftBackDrive;
    }

    public DcMotor getRightBackDrive() {
        return rightBackDrive;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }


}