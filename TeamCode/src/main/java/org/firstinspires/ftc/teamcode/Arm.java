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
    private final double voltageTolerance = 0.1;
    private final double encoderTolerance = 10;

    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Servo gripper;
    private Servo pitch;
    private Servo roll;
    private AnalogInput pot;
    private double armX = 0;
    private double armY = 0;
    PIDFController armPID = new PIDFController(new PIDCoefficients(6,0.01,3));
    double arm1Offset = 50; // angle between maxEncoder position and ground
    int maxEncoder1;
    int maxEncoder2;

    public Arm (HardwareMap map, Telemetry telemetry){
        arm1 = map.get(DcMotorEx.class, "arm1");
        arm2 = map.get(DcMotorEx.class, "arm2");
        pot = map.get(AnalogInput.class, "pot");
        gripper = map.get(Servo.class, "gripper");
        pitch = map.get(Servo.class, "pitch");
        roll = map.get(Servo.class, "roll");

        do {
            arm1.setVelocity(-5);
        } while (arm1.getCurrent(CurrentUnit.AMPS) < 2.5);
        arm1.setVelocity(0);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        do {
            arm1.setVelocity(5);
        } while (arm1.getCurrent(CurrentUnit.AMPS) < 2.5);
        arm1.setVelocity(0);
        maxEncoder1 = arm1.getCurrentPosition();

        do {
            arm2.setVelocity(-5);
        } while (arm2.getCurrent(CurrentUnit.AMPS) < 2.5);
        arm2.setVelocity(0);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        do {
            arm2.setVelocity(5);
        } while (arm2.getCurrent(CurrentUnit.AMPS) < 2.5);
        arm2.setVelocity(0);
        maxEncoder2 = arm2.getCurrentPosition();
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
        double[] angles = inverseKinematics(x, y);
        arm1.setTargetPosition((int) angles[0]);
        arm2.setTargetPosition((int) angles[1]);
        pitch.setPosition(angles[2]);
        roll.setPosition(angles[3]);
        armPID.setTargetPosition(angleToVoltage(angles[0]));
    }

    public void update(){
        arm1.setPower(armPID.update(pot.getVoltage()));
    }

    public boolean atTarget(){
        double[] angles = inverseKinematics(armX, armY); // technically is now ticks not angles
        double pV = pot.getVoltage();
        double tV = angleToVoltage(angles[0]);
        // the 2 numbers are the target tolerances
        return (Math.abs(pV-tV) < voltageTolerance && Math.abs(arm2.getCurrentPosition() - arm1.getTargetPosition()) < encoderTolerance);
    }

    public double[] inverseKinematics(double x, double y) {
        double maxEnc1Angle = ticksToAngle(maxEncoder1);
        double maxEnc2Angle = ticksToAngle(maxEncoder2);

        double maxPitch = 270;
        double maxRoll = 270;
        double[] angles = new double[4];
        // when x is 0 trig works differently
        if (x == 0) {
            // for now shouldn't ever be 0
            if (arm1.getCurrentPosition() < maxEnc1Angle / 2) {
                // doesn't really account for currentX == 0
                angles[0] = 0;
                angles[1] = maxEnc2Angle;
            } else {
                angles[0] = maxEnc1Angle;
                angles[1] = 0;
            }
        } else {
            // basic ik for /￣ form of the two arms
            angles[1] = -Math.acos((Math.pow(x, 2) + Math.pow(y, 2) - (LINK1 * LINK1) - (LINK2 * LINK2)) / (2 * LINK1 * LINK2));
            angles[0] = Math.atan((LINK2*Math.sin(angles[1]))/(LINK1 + LINK2*Math.cos(angles[1]))) + Math.atan(y/x);
            angles[2] = maxPitch / 2;
            angles[3] = maxRoll / 2;

            // angle[0] is measured from the ground on right side
            // angle[1] is measured from extension of link1
            // makes angles make sense compared to encoders
            angles[0] = Math.toDegrees(angles[0]);
            angles[0] = Math.toDegrees(angles[1]);
            if (x > 0) {
                angles[0] = (int) (maxEnc1Angle - angles[0] + arm1Offset);
                angles[1] = (int) (maxEnc2Angle / 2 + angles[1]);
            } else if (x < 0) {
                angles[0] = (int) (angles[0] - arm1Offset);
                angles[1] = (int) (maxEnc2Angle - angles[1]);
            }
        }

        // pitch
        if ((180 - angles[0] - arm1Offset - angles[1] + maxEnc2Angle / 2) > 0) { // if link2 points upwards
            angles[2] = 180 - angles[0] - arm1Offset - angles[1]  + maxEnc1Angle / 2;
        } else if ((180 - angles[0] - arm1Offset - angles[1] + maxEnc2Angle / 2) < 0) {
            angles[2] = angles[0] + angles[1] + arm1Offset - 180 - maxEnc2Angle / 2;
        }
        angles[2] /= maxPitch; // to convert to servo motor value
        if (arm1.getCurrentPosition() > maxEnc1Angle / 2) {
//            inverts pitch if on right side (since roll turns it upside down)
            angles[2] = 1 - angles[2];
        }

        // roll
        // assuming that view from left side is:
        /*
        bigger encoder value
              ___
             /   \
            |
            \___/
        smaller encoder value
        rotates anti-clockwise when moving to right side
         */
        if ((arm1.getCurrentPosition() > maxEnc1Angle / 2) && (x < 0)) {
            // going right to left
            angles[3] -= 180;
        } else if ((arm1.getCurrentPosition() < maxEnc1Angle / 2) && (x > 0)) {
            // left to right
            angles[3] += 180;
        }

        angles[0] = angleToTicks(Math.toDegrees(angles[0]));
        angles[1] = angleToTicks(Math.toDegrees(angles[1]));
        return angles;
    }

    public double[] forwardKinematics(int q1, int q2) {
        /**
         * Input is number of encoder ticks
         * Output is (x, y) coordinate of end of arm
         */
        double a1 = 180 - arm1Offset - ticksToAngle(q1);
        double a2 = Math.abs(maxEncoder2 / 2 - ticksToAngle(q2));

        double hyp = Math.sqrt(LINK1*LINK1 + LINK2*LINK2 - 2*LINK1*LINK2*Math.cos(180 - a2));
        double hyp_angle = Math.acos((LINK2*LINK2 - LINK1*LINK1 + hyp*hyp)/(-2*LINK1*hyp));

        double[] coords = {Math.sin(a1 - hyp_angle)*hyp,Math.cos(a1 - hyp)*hyp};
        return coords;
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
    public double ticksToAngle(int ticks) {
        return ((double) ticks) / 8;
    }

    public double angleToVoltage(double a) {
        return (445.5*(a-270))/((a*a)-(270*a)-36450);
    }
}