package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private int j = 0;
    PIDFController armPID = new PIDFController(new PIDCoefficients(6,0.01,3));

    double arm1Offset = 40; // angle between maxEncoder position and ground
    double maxEncoder1 = 800;
    double maxEncoder2 = 2250;
    int target1 = 0;
    int target2 = 0;
    double servopos1 = 0;
    double servopos2 = 0;
    double servoold1 = 0.01;
    double servoold2 = 0.01;
    double time = 4;
    double maxEnc1Angle = maxEncoder1 / 8.88;
    double maxEnc2Angle = maxEncoder2 / 8.88;
    double maxPitch = 270;
    double maxRoll = 270;
    double[] angles = new double[4];

    Telemetry telemetry;
    ElapsedTime runtime;

    public Arm (HardwareMap map, Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        this.telemetry = telemetry;
        arm1 = map.get(DcMotorEx.class, "arm1");
        arm2 = map.get(DcMotorEx.class, "arm2");
        arm2.setDirection(DcMotor.Direction.REVERSE);
        gripper = map.get(Servo.class, "gripper");
        pitch = map.get(Servo.class, "pitch");
        roll = map.get(Servo.class, "roll");

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initLoop() throws InterruptedException {
        telemetry.addData("arm1", arm1.getCurrentPosition());
        telemetry.addData("arm2", arm2.getCurrentPosition());
    }

    public void move(Gamepad gamepad){
        if(gamepad.a) openGripper();
        if(gamepad.b) closeGripper();

        target1 += -gamepad.right_stick_y*2;
        target2 += gamepad.left_stick_y*2;
        goToServo(servopos1, servopos2, servoold1, servoold2);

        if (gamepad.dpad_down) {
            armY = 0.15;
            armX = 0.7;
            servoold1 = servopos1;
            servoold2 = servopos2;
            servopos1 = 0.7;
            servopos2 = 0;

            target1 = 100;
            target2 = 1800;
            time = 0.01;
            runtime.reset();

        } else if (gamepad.dpad_up) {
            armY = 0.7;
            armX = 0.3;
            servoold1 = servopos1;
            servoold2 = servopos2;
            servopos1 = 0.3;
            servopos2 = 1;

            time = 4;

            target1 = 140;
            target2 = 600;
            runtime.reset();


            //140
            //600
        } else if (gamepad.dpad_right) {
            armY = 0.5;
            armX = 0.5;
        } else if (gamepad.dpad_left) {
            armY = 0.3;
            armX = 0.7;
        }

        arm1.setTargetPosition(target1);
        arm2.setTargetPosition(target2);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setPower(1);
        arm2.setPower(1);
        telemetry.addData("target1", target1);
        telemetry.addData("target2", target2);
        telemetry.addData("gripper", gripper.getPosition());
        telemetry.addData("pitch", pitch.getPosition());
        telemetry.addData("roll", roll.getPosition());

        if(gamepad.right_bumper) armX *= -1;
//        move(armX, armY);
    }

    public void move(double x, double y) {
        double[] angles = inverseKinematics(x, y);
        arm1.setTargetPosition((int) angles[0]);
        arm2.setTargetPosition((int) angles[1]);
        arm1.setPower(1);
        arm2.setPower(1);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("base target", angles[0] + " " + arm1Offset);
        telemetry.addData("top target", angles[1]);
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
        double currentArm1 = arm1.getCurrentPosition() * 8.88;

        // basic ik for  form of the two arms
        angles[1] = -Math.acos((Math.pow(x, 2) + Math.pow(y, 2) - (LINK1 * LINK1) - (LINK2 * LINK2)) / (2 * LINK1 * LINK2));
        angles[0] = Math.atan((LINK2*Math.sin(angles[1]))/(LINK1 + LINK2*Math.cos(angles[1]))) + Math.atan(Math.abs(y/x));
        angles[2] = maxPitch / 2;
        angles[3] = maxRoll / 2;

        // angle[0] is measured from the ground on right side
        // angle[1] is measured from extension of link1
        // makes angles make sense compared to encoders
        angles[0] = Math.toDegrees(angles[0]);
        angles[1] = Math.toDegrees(angles[1]);

        angles[1] = Math.abs(angles[1]);

        double angle1 = angles[0];
        double angle2 = angles[1];

        angles[0] = maxEnc1Angle - angles[0] + arm1Offset;
        angles[1] = maxEnc2Angle / 2 + angles[1];

        // pitch
        double temp = (180 - angles[0] - arm1Offset - angles[1] + maxEnc2Angle / 2);
        angles[2] = 180 - Math.abs(temp);

        angles[2] /= maxPitch; // to convert to servo motor value

        if (angles[0] > (maxEnc1Angle / 2)) {
//            inverts pitch if on right side (since roll turns it upside down)
            angles[2] = 1 - angles[2];
        }

        /* roll
        assuming that view from left side is:
        bigger encoder value
              ___
             /   \
            |
            \___/
        smaller encoder value
        rotates anti-clockwise when moving to right side
         */
        if ((currentArm1 > (maxEnc1Angle / 2)) && (x < 0)) {
            // going right to left
            angles[3] -= 180;
        } else if ((currentArm1 < (maxEnc1Angle / 2)) && (x > 0)) {
            // left to right
            angles[3] += 180;
        }
        angles[3] = Math.abs(angles[3]) / maxRoll;

        System.out.println("angle 1: " + angles[0]);
        System.out.println("angle 2: " + angles[1]);

        if (x > 0) {
            angles[0] = (maxEnc1Angle - angle1);
        } else {
            angles[1] = (maxEnc2Angle - angle2);
        }
        angles[0] *= 8.88;
        angles[1] *= 8.88;

        return angles;
    }

    public double[] forwardKinematics(int q1, int q2) {
        /*
          Input is number of encoder ticks
          Output is (x, y) coordinate of end of arm
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

    public void goToServo(double x, double y, double x2, double y2){
        double gradient1 = (x-x2)/time;
        double gradient2 = (y-y2)/time;
        double newx = Range.clip((runtime.seconds()*gradient1)+x2, Math.min(x, x2), Math.max(x, x2));
        double newy = Range.clip((runtime.seconds()*gradient2)+y2, Math.min(y, y2), Math.max(y, y2));
        pitch.setPosition(newx);
        roll.setPosition(newy);
    }
}