package org.firstinspires.ftc.teamcode;

import android.sax.TextElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.usb.UsbSerialNumber;

import java.text.DecimalFormat;


public class Drive {
    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    public BNO055IMU imu = null;
    private double multiplier = 1;

    public Drive(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;
    }

    public void setup() {
        leftFrontDrive = map.get(DcMotor.class, "Motor1");
        rightFrontDrive = map.get(DcMotor.class, "Motor0");
        leftBackDrive = map.get(DcMotor.class, "Motor2");
        rightBackDrive = map.get(DcMotor.class, "Motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addData("Status", "Setup");
    }

    public void mecanum(Gamepad gamepad) {
        double angle = Math.atan2(gamepad.left_stick_y,gamepad.left_stick_y); //+ imu.getAngularOrientation().firstAngle;
        double power = Math.max(-1, Math.min(1, Math.hypot(gamepad.left_stick_x,gamepad.left_stick_y)));
        double x = Math.cos(angle) * power;
        double y = Math.sin(angle) * power;
        double turning = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turning), 1);
        double frontLeftPower = (y + x + turning) / denominator;
        double backLeftPower = (y - x + turning) / denominator;
        double frontRightPower = (y - x - turning) / denominator;
        double backRightPower = (y + x - turning) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
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