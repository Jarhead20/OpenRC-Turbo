package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class AutoOp extends LinearOpMode {
    public SampleMecanumDrive drive;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    int cameraMonitorViewId;

    int offset;
    int pickHeight;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    Arm arm;
    double downAmount = 30;

    Vector2 pickup1;
    Vector2 pickupGrab;
    Vector2 pickupUp;
    Vector2 depositLoc;

    public AutoOp(int offset, int pickHeight) {
        this.offset = offset;
        this.pickHeight = pickHeight;

        pickup1 = new Vector2(-450+offset, pickHeight);
        pickupGrab = new Vector2(-600+offset, pickHeight);
        pickupUp = new Vector2(-500+offset, 400);
        depositLoc = new Vector2(170, 690);
    }

    public AutoOp() {
        // for AutoPark since it does not need offset or pickHeight
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
