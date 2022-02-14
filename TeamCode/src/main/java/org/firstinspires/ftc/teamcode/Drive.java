package org.firstinspires.ftc.teamcode;

import android.sax.TextElementListener;

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
    private DcMotor duckDrive = null;
    private DcMotor intakeDrive = null;
    public DcMotor slideDrive = null;
    public  Servo servo = null;
    private DuckSpinner ds;
    private Thread t;
    public int position = 0;
    private double multiplier = 1;
    public final double MIN_POSITION = 0, MAX_POSITION = 1;
    public double servoPosition = 0.5;

    public Drive(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
    }
    public void setup(){
        leftFrontDrive  = map.get(DcMotor.class, "leftFront");
        rightFrontDrive = map.get(DcMotor.class, "rightFront");
        leftBackDrive  = map.get(DcMotor.class, "leftRear");
        rightBackDrive = map.get(DcMotor.class, "rightRear");
        duckDrive = map.get(DcMotor.class, "duck");
        intakeDrive = map.get(DcMotor.class, "intake");
        slideDrive = map.get(DcMotor.class, "slide");
        servo = map.get(Servo.class, "servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        duckDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeDrive.setDirection(DcMotor.Direction.REVERSE);
        slideDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     CLSck ;nk
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo.getController().pwmEnable();
        servo.setPosition(0.5);
        duckDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ds = new DuckSpinner(duckDrive, -0.3,1500,-0.7,200);
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
//        telemetry.addData("Status", "max: " + max);
        for (int i = 0; i < 4; i++) {
            double e = v[i] / max;
//            telemetry.addData("Status", df.format(e) + " " + df.format(v[i]) + " " + df.format(max));
            v[i] = e;
        }
        leftFrontDrive.setPower(v[0]*multiplier);
        rightFrontDrive.setPower(v[1]*multiplier);
        leftBackDrive.setPower(v[2]*multiplier);
        rightBackDrive.setPower(v[3]*multiplier);
//        telemetry.addData("Status", v[0]);
//        telemetry.addData("Status", v[1]);
//        telemetry.addData("Status", v[2]);
//        telemetry.addData("Status", v[3]);
    }

    public void runDuck(){
        if(!t.isAlive()) t.start();
    }
    public void stopDuck(){
        if(!t.isInterrupted() && t.isAlive()) {
            t.interrupt();
            duckDrive.setPower(0);
        }
    }

    public void runIntake(){ intakeDrive.setPower(1); }
    public void stopIntake(){ intakeDrive.setPower(0); }

    public void slide(int position){
        //9 rotations for full extension

//        if(!st.isAlive()) {
//            st = new Thread(new Slide(position, slideDrive, servo, telemetry));
//            st.run();
//        }
        telemetry.addData("d",position + " " + slideDrive.getPower());
        switch(position){
            case 1:
                if(!slideDrive.isBusy()) {
                    slideDrive.setTargetPosition(0);
                    slideDrive.setPower(1);
                    slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(slideDrive.getCurrentPosition() >= -1500) servo.setPosition(0.5);
                else servo.setPosition(0.3);
                break;
            case 2:
                if(!slideDrive.isBusy()) {
                    slideDrive.setTargetPosition(-7250);
                    slideDrive.setPower(1);
                    slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(slideDrive.getCurrentPosition() <= -7000) servo.setPosition(0);
                else if(slideDrive.getCurrentPosition() >= -1600) servo.setPosition(0.5);
                else servo.setPosition(0.3);
                break;
            case 3:
                if(!slideDrive.isBusy()){
                    slideDrive.setTargetPosition(-10000);
                    slideDrive.setPower(1);
                    slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(slideDrive.getCurrentPosition() <= -9900) servo.setPosition(0);
                else if(slideDrive.getCurrentPosition() >= -1600) servo.setPosition(0.5);
                else servo.setPosition(0.3);
                break;
            default:
                break;

        }
    }

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

