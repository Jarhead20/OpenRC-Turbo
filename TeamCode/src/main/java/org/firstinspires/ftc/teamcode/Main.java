package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

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

        drive.setMultiplier(1-gamepad1.right_trigger);
        drive.mecanum(gamepad1);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {}
}