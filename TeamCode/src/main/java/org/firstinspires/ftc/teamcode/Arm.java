package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private final double LINK1 = 0.465;
    private final double LINK2 = 0.434;
    private final double voltageTolerance = 0.1;
    private final double encoderTolerance = 10;


    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Servo gripper;
    private AnalogInput pot;
    private double[] angles = {0,0};
    private double armX = 0;
    private double armY = 0;
    PIDFController armPID = new PIDFController(new PIDCoefficients(6,0.01,3));

    public Arm (HardwareMap map, Telemetry telemetry){

        arm1 = map.get(DcMotorEx.class, "arm1");
        arm2 = map.get(DcMotorEx.class, "arm2");
        pot = map.get(AnalogInput.class, "pot");
        gripper = map.get(Servo.class, "gripper");
    }

    public void move(Gamepad gamepad){
        if(gamepad.a) toggleGripper();

        armX += gamepad.right_stick_y;
        armY += gamepad.left_stick_y;

        if (gamepad.dpad_down) {
            armY = 0.15;
            armX = 0.7;
        } else if (gamepad.dpad_up) {
            armY = 0.7;
            armX = 0.3;
        } else if (gamepad.dpad_right) {
            armY = 0.5;
            armX = 0.5;
        } else if (gamepad.dpad_left) {
            armY = 0.3;
            armX = 0.7;
        }
        if(gamepad.right_bumper) armX *= -1;
        move(armX, armY);

        //pitch and yaw
    }

    public void move(double x, double y){
        angles = getAngles(armX, armY);
        arm2.setTargetPosition(angleToTicks(angles[1]));
        armPID.setTargetPosition(angleToVoltage(angles[0]));
    }

    public void update(){
        arm1.setPower(armPID.update(pot.getVoltage()));
    }

    public boolean atTarget(){
        double[] angles = getAngles(armX, armY);
        double pV = pot.getVoltage();
        double tV = angleToVoltage(angles[0]);
        //the 2 numbers are the target tolerances
        return (Math.abs(pV-tV) < voltageTolerance && Math.abs(arm2.getCurrentPosition() - arm1.getTargetPosition()) < encoderTolerance);
    }

    public double[] getAngles(double x, double y) {
        double link = LINK1; // link length
        double link2 = LINK2;
        double v = Math.pow(x, 2) + Math.pow(y, 2);
        double[] angles = {Math.atan(y / x),
                Math.acos((v - (link*link) - (link2*link2)) / (2 * link * link2))};

        double e1 = Math.atan(link2 * Math.sin(angles[1]) / (link + link2 * Math.cos(angles[1])));

        if (x > 0) {
            angles[0] -= e1;
        } else {
            angles[1] *= -1;
            angles[0] += e1;
        }
        return angles;
    }

    public void closeGripper(){
        gripper.setPosition(0);
    }

    public void openGripper(){
        gripper.setPosition(1.0);
    }

    public void toggleGripper(){
        gripper.setPosition(gripper.getPosition() > 0.9 ? 0 : 1.0);
    }

    public int angleToTicks(double a) {
        return (int) Math.floor(a*8);
    }

    public double angleToVoltage(double a) {
        return (445.5*(a-270))/((a*a)-(270*a)-36450);
    }
}