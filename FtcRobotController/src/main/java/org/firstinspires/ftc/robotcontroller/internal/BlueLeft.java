package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="BlueLeft")
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor rl = hardwareMap.get(DcMotor.class, "rl");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rr");
        DcMotor el = hardwareMap.get(DcMotor.class, "el");
        CRServo input1 = hardwareMap.get(CRServo.class, "input1");
        CRServo input2 = hardwareMap.get(CRServo.class, "input2");
        ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //el.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        telemetry.addLine("Initialized and Ready!");
        telemetry.update();

        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        RobotDirections rd = new RobotDirections(this, fl, fr, rl, rr, 0.8);

        waitForStart();

        rd.runTicks(24, Directions.Right);
        rd.runTicks(30, Directions.Forward);
        //cone on pole
        rd.runTicks(5, Directions.Backward);
        rd.TurnTo(Directions.Left);
        // pickup cone
        // {
        rd.TurnTo(Directions.Right);
        rd.runTicks(27, Directions.Forward);
        rd.TurnTo(Directions.Forward);
        // drop cone
        rd.TurnTo(Directions.Left);
        rd.runTicks(27, Directions.Forward);
        // pickup cone
        // } repeat

        //park

        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
    public boolean activated(float val) {
        return Math.abs(val) > 0.075;
    }
}
