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
    private int armX = 0;
    private int armY = 800;
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
    ArmModel armModel = new ArmModel();
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

        armX += -gamepad.right_stick_y*2;
        armY += gamepad.left_stick_y*2;
//        goToServo(servopos1, servopos2, servoold1, servoold2);

        if (gamepad.dpad_down) {
            armY = 30;
            armX = 700;
            servoold1 = servopos1;
            servoold2 = servopos2;
            servopos1 = 0.7;
            servopos2 = 0;

            target1 = 100;
            target2 = 1800;
            time = 0.01;
            runtime.reset();

        } else if (gamepad.dpad_up) {
            armY = 700;
            armX = 30;
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
            armY = 500;
            armX = 500;
        } else if (gamepad.dpad_left) {
            armY = 300;
            armX = 700;
        }

//        arm1.setTargetPosition(target1);
//        arm2.setTargetPosition(target2);
//        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm1.setPower(1);
//        arm2.setPower(1);
//        telemetry.addData("target1", target1);
//        telemetry.addData("target2", target2);
//        telemetry.addData("gripper", gripper.getPosition());
//        telemetry.addData("pitch", pitch.getPosition());
//        telemetry.addData("roll", roll.getPosition());

        if(gamepad.right_bumper) armX *= -1;
        move(armX, armY);
    }

    public void move(int x, int y) {
        double[] angles = inverseKinematics(x, y);
        arm1.setTargetPosition((int) angles[1]);
        arm2.setTargetPosition((int) angles[0]);
        telemetry.addData("lower", angles[1]);
        telemetry.addData("upper", angles[0]);
        arm1.setPower(1);
        arm2.setPower(1);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("base target", angles[0] + " " + arm1Offset);
        telemetry.addData("top target", angles[1]);
//        pitch.setPosition(angles[2]);
//        roll.setPosition(angles[3]);
        armPID.setTargetPosition(angleToVoltage(angles[0]));
    }

    public void update(){
        arm1.setPower(armPID.update(pot.getVoltage()));
    }

    public double[] inverseKinematics(int x, int y) {
        double[] angles = armModel.calculateMotorPositions(x,y);
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