package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private DcMotorEx shoulderMotor;
    private DcMotorEx elbowMotor;
    private Servo gripper;
    private Servo pitch;
    private Servo roll;
    private double armX = 0;
    private double armY = 0;
    private double targetArmX = -150;
    private double targetArmY = 700;
    private double targetShoulderAngle = 0;
    private double targetElbowAngle = 0;
    ArmModel model = new ArmModel();

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
        shoulderMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setPower(1);
        shoulderMotor.setPower(1);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initLoop() throws InterruptedException {
        return;
    }

    public void inputGamepad(Gamepad gamepad){
        //Inverse Kinematics
        targetArmX -= gamepad.left_stick_y*5;
        targetArmY -= gamepad.right_stick_y*5;

        moveTo(targetArmX, targetArmY);

        //Forward Kinematics
        reportCurrentPosition();
    }

    public void moveTo(double x, double y){
        double[] angles = model.calculateMotorPositions((int)x, (int)y);

        targetShoulderAngle = model.radiansToEncoder(angles[0]);
        targetElbowAngle = model.radiansToEncoder(angles[1]);

        pitch.setPosition(angles[2]);
        roll.setPosition(angles[3]);
    }

    public void reportCurrentPosition(){
        int shoulderRot = shoulderMotor.getCurrentPosition();
        int elbowRot = elbowMotor.getCurrentPosition();
        int[] armCoords = model.anglesToPosition(model.encoderToDegrees(shoulderRot) + 45, model.encoderToDegrees(elbowRot)+180);
        armX = armCoords[0];
        armY = armCoords[1];
        telemetry.addData("shoulderRot", model.encoderToDegrees(shoulderMotor.getCurrentPosition())+45);
        telemetry.addData("elbowRot", model.encoderToDegrees(elbowMotor.getCurrentPosition())+180);
        telemetry.addData("armX", armX);
        telemetry.addData("armY", armY);
        telemetry.addData("targetX", targetArmX);
        telemetry.addData("targetY", targetArmY);
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

    public void goToServo(double x, double y, double x2, double y2){
        double gradient1 = (x-x2)/4;
        double gradient2 = (y-y2)/4;
        double newx = Range.clip((runtime.seconds()*gradient1)+x2, Math.min(x, x2), Math.max(x, x2));
        double newy = Range.clip((runtime.seconds()*gradient2)+y2, Math.min(y, y2), Math.max(y, y2));
        pitch.setPosition(newx);
        roll.setPosition(newy);
    }
}