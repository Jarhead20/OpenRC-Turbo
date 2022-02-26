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



//192.168.43.1:8080/dash
@TeleOp(name="Main", group="Iterative Opmode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private boolean temp = false;

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
        if(runtime.seconds() >= 120) stop();
        if(gamepad2.a) drive.runDuck();
        if(gamepad2.x) drive.stopDuck();

        if(gamepad2.b) drive.runIntake();
        if(gamepad2.y) drive.stopIntake();

        if(gamepad2.right_bumper) {
            drive.slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.slideDrive.setTargetPosition(0);
            drive.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        telemetry.addData("Degrees:", drive.servo.getPosition());
        if(gamepad2.left_bumper) temp = !temp;
        telemetry.addData("temp",temp);
        if(temp){
            if(gamepad2.dpad_up) drive.position=3;
            else if (gamepad2.dpad_left) drive.position=2;
            else if(gamepad2.dpad_down) drive.position=1;
            drive.slide(drive.position);
        }
        else{

            drive.servo.setPosition((gamepad2.right_stick_y+1)/2);
            //drive.slideDrive.setTargetPosition((int) ((drive.slideDrive.getTargetPosition()+gamepad2.left_stick_y*10)));

            drive.slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.slideDrive.setPower(gamepad2.left_stick_y*(1-gamepad2.right_trigger));
            telemetry.addData("ooga booga", "ooga booga");
            drive.ramp.setPosition(0.5-gamepad2.left_trigger/2);
        }


        telemetry.addData("1", drive.slideDrive.getCurrentPosition() + " " + drive.slideDrive.getTargetPosition());

        drive.setMultiplier((gamepad1.right_trigger/2)+0.5);
        drive.mecanum(gamepad1);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {}
}