package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

//192.168.43.1:8080/dash
@TeleOp(name="SensorTest", group="Iterative Opmode")
public class SensorTest extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;
    private TouchSensor sensor = null;
    private TouchSensor mag = null;
    private DistanceSensor distance = null;
    private ColorSensor color = null;
    private RevBlinkinLedDriver blinker = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

//        sensor = hardwareMap.get(TouchSensor.class, "touch");
//        mag = hardwareMap.get(TouchSensor.class, "mag");
//        distance = hardwareMap.get(DistanceSensor.class, "distance");
//        color = hardwareMap.get(ColorSensor.class, "color");
        blinker = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        blinker.setPattern(pattern);
//        telemetry.addData("b", distance.getDistance(DistanceUnit.CM));
//        telemetry.addData("e", sensor.isPressed());
//        telemetry.addData("a", mag.isPressed());
//        telemetry.addData("c",color.argb());
//        if(sensor.isPressed() || mag.isPressed()) motor.setPower(1);
//        else motor.setPower(0);
    }

    @Override
    public void stop() {}
}