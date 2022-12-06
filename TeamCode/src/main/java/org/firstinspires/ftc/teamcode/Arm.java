package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo gripper;
    private Servo pitch;
    private Servo roll;
    private double armX = 0;
    private double armY = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double tolerance = 100;
    private Vector2[] armPoses = new Vector2[]{
            new Vector2(460, 700),
            new Vector2(-450, 200),
            new Vector2(-350, 200),
            new Vector2(200, 300),
            new Vector2(-430, 200),
            new Vector2(-10, 700),
            new Vector2(-450, 500),
            new Vector2(-350, 200),
    };

    public int index = 0;
    public int position = 0;

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
        roll.setDirection(Servo.Direction.REVERSE);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initLoop() throws InterruptedException {
        reportCurrentPosition();
    }

    boolean up = false;

    ElapsedTime armTimer = new ElapsedTime();

    public void inputGamepad(Gamepad gamepad){

        if (gamepad.b){
            openGripper();
        }
        if (gamepad.a){
            closeGripper();
        }

        elbowMotor.setPower(0.4);
        shoulderMotor.setPower(0.4);

        if (gamepad.dpad_up){
            position = 0;
//            elbowMotor.setPower(0.8);
//            shoulderMotor.setPower(0.3);
            index = gamepad.left_bumper ? position : position+3;
            armTimer.reset();
        }

        else if (gamepad.dpad_down){
            position = 1;
            index = gamepad.left_bumper ? position : position+3;
        }

        else if (gamepad.dpad_left){
            position = 2;
            index = gamepad.left_bumper ? position : position+3;
        }

        switch (position){
            case 0:
                if(armTimer.milliseconds() < 500)
                    index = 6;
                else index = position;
                break;
            case 1:
                if(armTimer.milliseconds() < 1000)
                    index = 7;
                else index = position;
                break;
            case 2:
                break;
        }




        Vector2 vec = armPoses[index];
        vec.x += gamepad.left_stick_y*4;
        vec.y -= gamepad.right_stick_y*4;
//            vec.x = Range.clip(vec.x, -900, -1);
        moveTo(vec);
        //Inverse Kinematics


        //Forward Kinematics
        reportCurrentPosition();
    }

    public void moveTo(Vector2 vec){
        //TODO: Add Range of Motion Constraints

        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetX = vec.x;
        targetY = vec.y;
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
//        telemetry.addData("elbow velo", elbowMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("elbow PIDF", elbowMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER)); // P: 10.00, I:0.05, D: 0.00, F: 0.00
//        elbowMotor.setPositionPIDFCoefficients(100);
//        elbowMotor.setVelocityPIDFCoefficients(100, 0.05, 0, 0);
        // 0.01 = slow
        // 10 = normal

        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("gripper", gripper.getPosition());
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
        telemetry.addData("targetX", armPoses[index].toString());
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

    public void setPower(double shoulder, double elbow){
        elbowMotor.setPower(elbow);
        shoulderMotor.setPower(shoulder);
    }

    int atCount = 0;

    public boolean atTarget(Vector2 vec){
        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];
        if(Math.abs(shoulderMotor.getCurrentPosition() - targetShoulderAngle) < tolerance){
            if(Math.abs(elbowMotor.getCurrentPosition() - (-targetElbowAngle)) < tolerance) return true;
        }
        return false;
    }

    public boolean atTarget(Vector2 vec, double tol){
        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];
        if(Math.abs(shoulderMotor.getCurrentPosition() - targetShoulderAngle) < tol){
            if(Math.abs(elbowMotor.getCurrentPosition() - (-targetElbowAngle)) < tol) return true;
        }
        return false;
    }
}