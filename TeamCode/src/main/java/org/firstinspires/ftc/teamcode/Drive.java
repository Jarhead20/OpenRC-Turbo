package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;


public class Drive {
    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor duck = null;
    private DcMotor intake = null;
    private double multiplier = 1;
    public Drive(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
    }
    public void setup(){
        leftFrontDrive  = map.get(DcMotor.class, "leftFront");
        rightFrontDrive = map.get(DcMotor.class, "rightFront");
        leftBackDrive  = map.get(DcMotor.class, "leftBack");
        rightBackDrive = map.get(DcMotor.class, "rightBack");
        duck = map.get(DcMotor.class, "duck");
        intake = map.get(DcMotor.class, "intake");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        duck.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Setup");
    }

    public void mecanum(Gamepad gamepad){
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(-gamepad.left_stick_y, -gamepad.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad.right_stick_x;
        double[] v = new double[4];
        v[0] = r * Math.cos(robotAngle) + rightX;
        v[1] = r * Math.sin(robotAngle) - rightX;
        v[2] = r * Math.sin(robotAngle) + rightX;
        v[3] = r * Math.cos(robotAngle) - rightX;
        double max = 0;
        for (int i = 0; i < 4; i++) {
            if(Math.abs(v[i]) > max) max = Math.abs(v[i]);
        }
        max = Range.clip(max,-1,1);
        DecimalFormat df = new DecimalFormat("0.00");
        telemetry.addData("Status", "max: " + max);
        for (int i = 0; i < 4; i++) {
            double e = v[i] / max;
//            telemetry.addData("Status", df.format(e) + " " + df.format(v[i]) + " " + df.format(max));
            if(v[i] < 0) {
                v[i] = -e;
//                telemetry.addData("Status", "test:                                     yes");
            }
            else v[i] = e;
        }


        leftFrontDrive.setPower(v[0]*multiplier);
        rightFrontDrive.setPower(v[1]*multiplier);
        leftBackDrive.setPower(v[2]*multiplier);
        rightBackDrive.setPower(v[3]*multiplier);
        telemetry.addData("Status", v[0]);
        telemetry.addData("Status", v[1]);
        telemetry.addData("Status", v[2]);
        telemetry.addData("Status", v[3]);
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

    public DcMotor getDuck() {
        return duck;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }
}
