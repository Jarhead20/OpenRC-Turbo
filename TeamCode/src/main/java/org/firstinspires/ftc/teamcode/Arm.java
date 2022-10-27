package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Arm {
    private final double LINK1 = 0.465;
    private final double LINK2 = 0.434;
    private final double arm1Tolerance = 10;
    private final double arm2Tolerance = 10;

    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Servo gripper;
    private Servo pitch;
    private Servo roll;
    private double[] angles = {0,0};
    private double armX = 0;
    private double armY = 0;
    double arm1Offset = 50; // angle between maxEncoder position and ground
    int maxEncoder1 = 200;
    int maxEncoder2 = 200;
    private Telemetry telemetry;

    public Arm (HardwareMap map, Telemetry telemetry){

        arm1 = map.get(DcMotorEx.class, "arm1");
        arm2 = map.get(DcMotorEx.class, "arm2");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper = map.get(Servo.class, "gripper");
        pitch = map.get(Servo.class, "pitch");
        roll = map.get(Servo.class, "roll");
//        gripper.getController().pwmEnable();
        this.telemetry = telemetry;

//        do {
//            arm1.setVelocity(-5);
//        } while (arm1.getCurrent(CurrentUnit.AMPS) < 2.5);
//        arm1.setVelocity(0);
//        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        do {
//            arm1.setVelocity(5);
//        } while (arm1.getCurrent(CurrentUnit.AMPS) < 2.5);
//        arm1.setVelocity(0);
//        maxEncoder1 = arm1.getCurrentPosition();
//
//        do {
//            arm2.setVelocity(-5);
//        } while (arm2.getCurrent(CurrentUnit.AMPS) < 2.5);
//        arm2.setVelocity(0);
//        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        do {
//            arm2.setVelocity(5);
//        } while (arm2.getCurrent(CurrentUnit.AMPS) < 2.5);
//        arm2.setVelocity(0);
//        maxEncoder2 = arm2.getCurrentPosition();
    }

    public void move(Gamepad gamepad){
        if(gamepad.a) openGripper();
        if(gamepad.b) closeGripper();

        armX += gamepad.right_stick_y/10;
        armY += gamepad.left_stick_y/10;

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
        telemetry.addData("test", armX + " " + armY);
        if (gamepad.left_bumper) {
            // assuming 1 is FORWARD, 0 is BACKWARD
            if (gripper.getDirection() == Servo.Direction.FORWARD) {
                gripper.setDirection(Servo.Direction.REVERSE);
            } else {
                gripper.setDirection(Servo.Direction.FORWARD);
            }
        }
    }

    public void move(double x, double y) {
        angles = getAngles(x, y);
//        arm1.setTargetPosition((int) angles[0]);
//        arm2.setTargetPosition((int) angles[1]);
        pitch.setPosition(angles[3]/180);
    }


    public boolean atTarget(){
        double[] angles = getAngles(armX, armY); // technically is now ticks not angles
        // the 2 numbers are the target tolerances
        return (Math.abs(arm2.getCurrentPosition() - arm2.getTargetPosition()) < arm2Tolerance && Math.abs(arm1.getCurrentPosition() - arm1.getTargetPosition()) < arm1Tolerance);
    }

    public double[] getAngles(double x, double y) {
        double maxPitch = 270;
        double maxRoll = 270;
        double[] angles = new double[4];
        if (x == 0) {
            if (arm1.getCurrentPosition() < maxEncoder1 / 2) {
                // doesn't really account for x == 0
                angles[0] = 0;
                angles[1] = maxEncoder2;
            } else {
                angles[0] = maxEncoder1;
                angles[1] = 0;
            }
        } else {
            angles[0] = Math.acos((x*x + y*y - LINK1*LINK1 - LINK2*LINK2)/(2*LINK1*LINK2));
            angles[1] = Math.acos((Math.pow(x, 2) + Math.pow(y, 2) - (LINK1 * LINK1) - (LINK2 * LINK2)) / (2 * LINK1 * LINK2));
            angles[2] = maxPitch / 2;
            angles[3] = maxRoll / 2;

            angles[0] = Math.atan((LINK2*Math.sin(angles[1]))/(LINK1 + LINK2*Math.cos(angles[1]))) - Math.atan(y/x);

            angleToTicks(Math.toDegrees(angles[0]));
            angleToTicks(Math.toDegrees(angles[1]));
            if (x > 0) {
                angles[0] = (int) (maxEncoder1 - angles[0] + arm1Offset);
                angles[1] = (int) (maxEncoder2 / 2 + angles[1]);
            } else if (x < 0) {
                angles[0] = (int) (angles[0] - arm1Offset);
                angles[1] = (int) (maxEncoder2 - angles[1]);
            }
        }
        if ((angles[1] - maxEncoder2 / 2 + 180 - angles[0] - arm1Offset) < 90) {
            angles[2] = 180 - angles[0] - arm1Offset - angles[1]  + maxEncoder1 / 2;
        } else if ((angles[1] - maxEncoder2 / 2 + 180 - angles[0] - arm1Offset) > 90) {
            angles[2] = angles[0] + angles[1] + arm1Offset - 180 - maxEncoder2 / 2;
        } else {
            angles[2] = 0;
        }
        angles[2] /= maxPitch; // to convert to servo motor value
        if ( (x < 0)) {
            // going right to left
            angles[3] -= 180;
        } else if ( (x > 0)) {
            // left to right
            angles[3] += 180;
        }
//        angles[3] /= maxRoll;
        return angles;
    }

    public void closeGripper(){
        gripper.setPosition(0);
        telemetry.addData("test", gripper.getPosition());
    }

    public void openGripper(){
        gripper.setPosition(1.0);
        telemetry.addData("test", gripper.getPosition());
    }

    public void toggleGripper(){

            gripper.setPosition(gripper.getPosition() > 0.9 ? 0 : 1.0);
            telemetry.addData("test", gripper.getPosition());

    }

    public int angleToTicks(double a) {
        return (int) Math.floor(a*8);
    }

    public double angleToVoltage(double a) {
        return (445.5*(a-270))/((a*a)-(270*a)-36450);
    }
}