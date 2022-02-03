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



        drive.slideDrive.setTargetPosition(-10000);
        drive.slideDrive.setPower(0.5);
        drive.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(drive.slideDrive.isBusy()){
            System.out.println(drive.slideDrive.getCurrentPosition());
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
            if(drive.slideDrive.getCurrentPosition() <= -9900) {
                drive.servo.setPosition(0);
                break;
            }
            else if(drive.slideDrive.getCurrentPosition() >= -1600) drive.servo.setPosition(0.5);

            else drive.servo.setPosition(0.3);
        }
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

        //drive.servo.setPosition(Range.clip(((gamepad2.right_stick_y+1)/2) + 0.2, drive.MIN_POSITION, drive.MAX_POSITION));

        telemetry.addData("Degrees:", drive.servo.getPosition());


        if(gamepad2.dpad_up && drive.position < 3) drive.slide(++drive.position);
        else if(gamepad2.dpad_down && drive.position > 0) drive.slide(--drive.position);

        drive.servo.setPosition((gamepad2.right_stick_y+1)/2);
        drive.slideDrive.setPower(gamepad2.left_stick_y);

        telemetry.addData("1", drive.slideDrive.getCurrentPosition() + " " + drive.slideDrive.getTargetPosition());

        drive.setMultiplier(1-gamepad1.right_trigger);
        drive.mecanum(gamepad1);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {}
}