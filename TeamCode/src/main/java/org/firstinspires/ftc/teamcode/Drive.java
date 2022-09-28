package org.firstinspires.ftc.teamcode;

import android.sax.TextElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    public BNO055IMU imu = null;
    private double multiplier = 1;

    public Drive(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;
    }

    public void setup() {
        leftFrontDrive = map.get(DcMotor.class, "Motor0");
        rightFrontDrive = map.get(DcMotor.class, "Motor1");
        leftBackDrive = map.get(DcMotor.class, "Motor2");
        rightBackDrive = map.get(DcMotor.class, "Motor3");
        arm1 = map.get(DcMotorEx.class, "Arm1");
        arm2 = map.get(DcMotorEx.class, "Arm2");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // resetting arm2 encoder
        do {
            arm2.setVelocity(-5);
        } while (arm2.getCurrent(CurrentUnit.AMPS) < 2.5);
        arm2.setVelocity(0);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Setup");
    }

    public void mecanum(Gamepad gamepad) {

        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y) * multiplier;
        double robotAngle = Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        telemetry.addData("angle", robotAngle);
        double rightX = gamepad.right_stick_x;
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

    public DcMotorEx getArm1() {
        return arm1;
    }

    public DcMotorEx getArm2() {
        return arm2;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }


}