package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Main TeleOP")
public class MainTeleOP extends LinearOpMode {
    FieldCentric fc = new FieldCentric();
    MecanumDrive md = new MecanumDrive();
    BNO055IMU.Parameters params = new BNO055IMU.Parameters();
    boolean enableMecanumDrive = true;
    boolean running = false;
    int runningOpt = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor rl = hardwareMap.get(DcMotor.class, "rl");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rr");
        DcMotor el = hardwareMap.get(DcMotor.class, "el");
        TouchSensor high = hardwareMap.get(TouchSensor.class, "high");
        TouchSensor low = hardwareMap.get(TouchSensor.class, "low");
        CRServo coneServo = hardwareMap.get(CRServo.class, "coneServo");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        fc.setUp(new DcMotor[] {fl, fr, rl, rr}, imu);
        md.setUp(fl, fr, rl, rr);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        el.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized and Ready!");
        telemetry.update();

        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        el.setDirection(DcMotor.Direction.REVERSE);

        float powerMult = 1;
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("FieldCentric: " + (enableMecanumDrive ? "OFF" : "ON"));
            if (running) {
                if (runningOpt == 0)
                    telemetry.addLine("AutoRunning Arm to cone pickup.");
                else if (runningOpt == 1)
                    telemetry.addLine("AutoRunning Arm to low drop-off.");
                else if (runningOpt == 2)
                    telemetry.addLine("AutoRunning Arm to medium drop-off.");
                else if (runningOpt == 3)
                    telemetry.addLine("AutoRunning Arm to high drop-off.");
            }
            telemetry.update();
            if (activated(gamepad1.left_stick_x) || activated(gamepad1.left_stick_y) || activated(gamepad1.right_stick_x)) {
                if (enableMecanumDrive) md.Drive(-gamepad1.left_stick_x * 1.1 * powerMult, gamepad1.left_stick_y * powerMult, -gamepad1.right_stick_x * powerMult);
                else fc.Drive(-gamepad1.left_stick_y * powerMult, gamepad1.left_stick_x * 1.1 * powerMult, -gamepad1.right_stick_x);
            }
            else {
                fl.setPower(0);
                fr.setPower(0);
                rl.setPower(0);
                rr.setPower(0);
            }
            if (activated(gamepad2.left_stick_y)) {
                if (el.isBusy()) running = false;
                if (gamepad2.left_stick_y > 0 && !low.isPressed()) el.setPower(-gamepad2.left_stick_y);
                if (gamepad2.left_stick_y < 0 && !high.isPressed()) el.setPower(-gamepad2.left_stick_y);
                if (high.isPressed() && gamepad2.left_stick_y < 0)
                    el.setPower(0);
                if (low.isPressed() && gamepad2.left_stick_y > 0)
                    el.setPower(0);
            }
            else if (!running) {
                el.setPower(0);
            }
            if (gamepad2.x) {
                running = true;
                runningOpt = 0;
            }
            if (gamepad2.a) {
                running = true;
                runningOpt = 1;
            }
            if (gamepad2.b) {
                running = true;
                runningOpt = 2;
            }
            if (gamepad2.y) {
                running = true;
                runningOpt = 3;
            }
            if (running && runningOpt == 0) {
                if (el.getCurrentPosition() > 500 && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 500 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 1) {
                if (el.getCurrentPosition() > 3200 && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 3200 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 2) {
                if (el.getCurrentPosition() > 5800 && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 5800 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 3) {
                if (el.getCurrentPosition() > 7000 && !high.isPressed() && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 7000 && !high.isPressed() && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (low.isPressed()) {
                el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                el.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            powerMult = 1 - gamepad1.left_trigger;
            if (gamepad1.left_stick_button)
                fc.newOffset();
            if (gamepad1.right_stick_button)
                enableMecanumDrive = !enableMecanumDrive;
            if (gamepad2.left_bumper) {
                coneServo.setPower(-1);
            }
            else if (gamepad2.right_bumper) {
                coneServo.setPower(1);
            }
            else {
                coneServo.setPower(0);
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
    public boolean activated(float val) {
        return Math.abs(val) > 0.075;
    }
}
