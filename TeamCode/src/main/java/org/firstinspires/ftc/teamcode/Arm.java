package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {
    private DcMotorEx shoulderMotor;
    private DcMotorEx elbowMotor;
    private Servo gripper;
    private Servo pitch;
    private Servo roll;
    private double armX = 0;
    private double armY = 0;
    private double targetArmX = -750; // -430
    private double targetArmY = 30; // 100
    private double targetLoadX = 460; // 200
    private double targetLoadY = 700; // 300
    private double targetShoulderAngle = 0;
    private double targetElbowAngle = 0;
    ArmModel model = new ArmModel();
    private boolean unloadPos = true;

    Telemetry telemetry;
    ElapsedTime runtime;

    public Arm (HardwareMap map, Telemetry telemetry, ElapsedTime runtime){
        this.runtime = runtime;
        this.telemetry = telemetry;
        shoulderMotor = map.get(DcMotorEx.class, "arm1");
        elbowMotor = map.get(DcMotorEx.class, "arm2");
        gripper = map.get(Servo.class, "gripper");
        pitch = map.get(Servo.class, "pitch");
        roll = map.get(Servo.class, "roll");

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initLoop() throws InterruptedException {
        reportCurrentPosition();
    }

    public void inputGamepad(Gamepad gamepad){

        if (gamepad.b){
            openGripper();
        }
        if (gamepad.a){
            closeGripper();
        }

        if (gamepad.dpad_up){
            unloadPos = true;
            closeGripper();
        }
        if (gamepad.dpad_down){
            unloadPos = false;
        }
        if (!unloadPos){
            targetArmX += gamepad.left_stick_y*4;
            targetArmY -= gamepad.right_stick_y*4;
            Range.clip(targetArmX, -900, -1);
            moveTo(targetArmX, targetArmY);
        }
        else{
            targetLoadX += gamepad.left_stick_y*4;
            targetLoadY -= gamepad.right_stick_y*4;
            Range.clip(targetArmX, 1, 900);
            moveTo(targetLoadX, targetLoadY);
        }
        //Inverse Kinematics


        //Forward Kinematics
        reportCurrentPosition();
    }

    public void moveTo(double x, double y){
        //TODO: Add Range of Motion Constraints

        double[] angles = model.calculateMotorPositions((int)x, (int)y);
        if (angles == null){
            return;
        }
        if (angles[1] < 0){
            return;
        }
        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];
        shoulderMotor.setTargetPosition((int)targetShoulderAngle);
        elbowMotor.setTargetPosition((int)-targetElbowAngle);
        elbowMotor.setPower(1);
        shoulderMotor.setPower(1);
        telemetry.addData("elbow velo", elbowMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("elbow PIDF", elbowMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER)); // P: 10.00, I:0.05, D: 0.00, F: 0.00
//        elbowMotor.setPositionPIDFCoefficients(100);
//        elbowMotor.setVelocityPIDFCoefficients(100, 0.05, 0, 0);
        // 0.01 = slow
        // 10 = normal

        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("shoulder", targetShoulderAngle);
        telemetry.addData("elbow", targetElbowAngle);
        pitch.setPosition(1-angles[2]);
        telemetry.addData("Pitch", angles[2]);
        roll.setPosition(angles[3]);
    }

    public void reportCurrentPosition(){
        int shoulderRot = shoulderMotor.getCurrentPosition();
        int elbowRot = elbowMotor.getCurrentPosition();
        int[] armCoords = model.anglesToPosition(model.encoderToDegrees(shoulderRot) + 45, model.encoderToDegrees(elbowRot)+180);
        armX = armCoords[0];
        armY = armCoords[1];
        telemetry.addData("shoulderRot", shoulderMotor.getCurrentPosition());
        telemetry.addData("elbowRot", elbowMotor.getCurrentPosition());

        telemetry.addData("armX", armX);
        telemetry.addData("armY", armY);
        telemetry.addData("targetX", targetArmX);
        telemetry.addData("targetY", targetArmY);
    }

    public void closeGripper(){
        gripper.setPosition(1.0);
    }

    public void openGripper(){
        gripper.setPosition(0);
    }

    public void toggleGripper(){
        gripper.setPosition(gripper.getPosition() > 0.9 ? 0 : 1.0);
    }

    public void goToServo(double x, double y, double x2, double y2){
        double gradient1 = (x-x2)/4;
        double gradient2 = (y-y2)/4;
        double newx = Range.clip((runtime.seconds()*gradient1)+x2, Math.min(x, x2), Math.max(x, x2));
        double newy = Range.clip((runtime.seconds()*gradient2)+y2, Math.min(y, y2), Math.max(y, y2));
        pitch.setPosition(newx);
        roll.setPosition(newy);
    }
}