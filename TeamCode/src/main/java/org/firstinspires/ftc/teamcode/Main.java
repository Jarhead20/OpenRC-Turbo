package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

// Very nice job @Jarhead.

//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative Opmode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if(gamepad2.a) drive.runDuck();
        if(gamepad2.x) drive.stopDuck();

        if(gamepad2.b) drive.runIntake();
        if(gamepad2.y) drive.stopIntake();

       // if(gamepad2.dpad_left) drive.servo.setPosition();
       // if(gamepad2.dpad_right) drive.servo.setPosition();
        // +1/2

//        drive.servoPosition = 1;
        //telemetry.addData("yes",drive.servo.getController().getPwmStatus());
        if(gamepad2.dpad_down)
            drive.servoPosition -= 0.2;

        else if(gamepad2.dpad_up)
            drive.servoPosition += 0.2;



        drive.servo.setPosition((gamepad2.right_stick_y+1)/2);
//        drive.servo.setPosition(0.65);
        //drive.servo.setPosition(Range.clip(((gamepad2.right_stick_y+1)/2) + 0.2, drive.MIN_POSITION, drive.MAX_POSITION));

        telemetry.addData("Degrees:", drive.servo.getPosition());
        telemetry.addData("servoPosition Variable", drive.servoPosition);

        drive.slide(gamepad2.left_stick_y);
        drive.setMultiplier(1-gamepad1.right_trigger);
        drive.mecanum(gamepad1);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {}
}