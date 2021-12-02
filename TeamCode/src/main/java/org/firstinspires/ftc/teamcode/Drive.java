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
    private DcMotor duckDrive = null;
    private DcMotor intakeDrive = null;
    private DuckSpinner ds;
    private Thread t;
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
        duckDrive = map.get(DcMotor.class, "duck");
        intakeDrive = map.get(DcMotor.class, "intake");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        duckDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ds = new DuckSpinner(duckDrive, -0.1,200,-1,200);
        t = new Thread(ds);

        telemetry.addData("Status", "Setup");
    }

    public void mecanum(Gamepad gamepad){

        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;
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
            telemetry.addData("Status", df.format(e) + " " + df.format(v[i]) + " " + df.format(max));
            v[i] = e;
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

    public void runDuck(){
        if(!t.isAlive()) t.start();
    }
    public void stopDuck(){
        if(!t.isInterrupted() && t.isAlive()) t.interrupt();
    }

    public void runIntake(){ intakeDrive.setPower(1); }
    public void stopIntake(){ intakeDrive.setPower(0); }

    public DcMotor getLeftFrontDrive() {
        return leftFrontDrive;
    }

    public DcMotor getRightFrontDrive() {
        return rightFrontDrive;
    }

    public DcMotor getLeftBackDrive() { return leftBackDrive; }

    public DcMotor getRightBackDrive() {
        return rightBackDrive;
    }

    public DcMotor getDuck() {
        return duckDrive;
    }

    public DcMotor getIntake() {
        return intakeDrive;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }
}

class DuckSpinner implements Runnable {
    private DcMotor duck;
    private double s1;
    private long t1;
    private double s2;
    private long t2;
    public DuckSpinner(DcMotor duck, double s1, long t1, double s2, long t2){
        this.duck = duck;
        this.s1 = s1;
        this.t1 = t1;
        this.s2 = s2;
        this.t2 = t2;
    }

    @Override
    public void run() {
        try {
            duck.setPower(s1);
            Thread.sleep(t1);
            duck.setPower(s2);
            Thread.sleep(t2);
            duck.setPower(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
