package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp()
public class ArmPIDTest extends LinearOpMode {

    public double x = 400;
    public double y = 400;

    private Arm arm;
    private SampleMecanumDrive drive;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        time.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        arm = new Arm(hardwareMap, telemetry, time);
//        drive = new SampleMecanumDrive(hardwareMap);

        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "Servo2");
        double prevX = x;
        double prevY = y;
        double gripperPos = 0.7;
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
//
//            double normalizedY = (gamepad1.left_stick_y+1.0)/2.0;
//            arm.shoulderMotor.setTargetPosition((int) (normalizedY/1700.0));
//            arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //2500 estimated elbow encoder ticks
//            if (gamepad2.left_bumper){
                // Calculate the average angle for both servos
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
                if (gamepad2.a) {
                    gripperPos = 0.6;
                }
                if (gamepad2.b)
                    gripperPos = 0.95;
                gripper.setPosition(gripperPos);

//            }
            if(gamepad2.left_bumper){
                x += gamepad2.left_stick_y*10;
                y += gamepad2.right_stick_y*10;
            }

//            arm.elbowMotor.setPower((gamepad2.left_stick_y+1.0)/2.0);
//            arm.shoulderMotor.setPower((gamepad2.right_stick_y+1.0)/2.0);
//            arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            arm.elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            telemetry.addData("arm1", arm.elbowMotor.getCurrentPosition());
//            telemetry.addData("arm0", arm.shoulderMotor.getCurrentPosition()); //1700 encoder ticks for full range of motion, rotating from rear to front of the robot makes encoder go negative
//            if(!arm.moveTo(new Vector2(x, y))){
//                x = prevX;
//                y = prevY;
//            }
//            prevX = x;
//            prevY = y;
            telemetry.addData("x", x);
            telemetry.addData("y", y);
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            gamepad1.left_stick_y,
//                            gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
            telemetry.update();
        }

    }
}
