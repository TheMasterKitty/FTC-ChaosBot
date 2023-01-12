    package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name="Main TeleOP")
public class MainTeleOP extends LinearOpMode {
    FieldCentric fc = new FieldCentric();
    MecanumDrive md = new MecanumDrive();
    BNO055IMU.Parameters params = new BNO055IMU.Parameters();
    boolean enableMecanumDrive = true;
    boolean running = false;
    int runningOpt = 0;
    int LiftMotorVariance = 50; // Identify in TICKS how close the left motor needs to get to target hieght, this provides some flexiblity and prevents weird oscilations when a specific value can not be reached.
    float powerMult = 1; // Variable to store the power multiplier based on triggers pressed by the driver

    // Variables to determine when buttons A or Y are pressed, and not cause the code to run on every cycle
    boolean LastValueGP1A = false;
    boolean LastValueGP1Y = false;

    // These variables are about in the Intake server running and ramp down variables.
        // Varialbes with UP are used for when the cone is being picked up
        // Variables with DOWN are for when the cone is being dropped.
    double servopower=0;
    double ServoUpMuitly=0.75; // Percent to change the servo speed per step
    double ServoDownMuilty=0.90; // Percent to change the servo speed per step
    double ServoThershold=0.01; // Theashold as to what is considered 0 and shold stop the motor
    double servoMax = 0.75; // Maximum Servo spped in value of 0->1.0
    double servoStepTime = 0.1; // How often to step down the motor in seconds.

    ElapsedTime tm = new ElapsedTime((ElapsedTime.Resolution.MILLISECONDS)); // Used for tracking main loop duration.

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

        waitForStart();
        LastValueGP1A = gamepad1.a; // Initialize the current value of the A button
        LastValueGP1Y = gamepad1.y; // Initialize the current value of the Y button
        servopower=0; // Default the lift servo power.


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
            telemetry.addLine("Loop Time: " + tm.toString()); // This is a timer to know how quickly the main code loop is running
            tm.reset(); // Reset the loop timer

            if(servopower != 0) telemetry.addLine("Servo Power: " + servopower); // When the cone Servo is running, display power.

            telemetry.update();
            if (activated(gamepad1.left_stick_x) || activated(gamepad1.left_stick_y) || activated(gamepad1.right_stick_x)) {
                if (enableMecanumDrive) md.Drive(-gamepad1.left_stick_x * 1.1 * powerMult, gamepad1.left_stick_y * powerMult, -gamepad1.right_stick_x * powerMult);
                else fc.Drive(-gamepad1.left_stick_y * powerMult, gamepad1.left_stick_x  * powerMult, -gamepad1.right_stick_x * powerMult); // removed - * 1.1
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
            // A windows of "LiftMotorVariance" ticks was added to each condition to help when the motor is not exactly at the specific value.
            // The specific value in both conditions was causing the arm to sometimes fluctuate at trying to reach a specific place.
            if (running && runningOpt == 0) {
                if (el.getCurrentPosition() > 500 + LiftMotorVariance && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 500 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 1) {
                if (el.getCurrentPosition() > 3200 + LiftMotorVariance && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 3200 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 2) {
                if (el.getCurrentPosition() > 5800 + LiftMotorVariance && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 5800 && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (running && runningOpt == 3) {
                if (el.getCurrentPosition() > 7000 + LiftMotorVariance && !high.isPressed() && opModeIsActive() && running) el.setPower(-1);
                else if (el.getCurrentPosition() < 7000 && !high.isPressed() && opModeIsActive() && running) el.setPower(1);
                else running = false;
            }
            if (low.isPressed()) {
                el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                el.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Set specific runs speeds based on trigger pressed, instead of how much trigger is pressed
            // per request from the drivers.
            if(activated(gamepad1.left_trigger))
                powerMult = 0.25f;
            else if(activated(gamepad1.right_trigger))
                powerMult = 0.75f;
            else
                powerMult = 1;



            // Only run this code when the value of the A button has changed
            // This ensures that this code is only run once per button press
            if (gamepad1.a && LastValueGP1A != gamepad1.a)
                enableMecanumDrive = !enableMecanumDrive;
            LastValueGP1A = gamepad1.a;

            // Only run this when the value of the Y button has changed.
            // This ensures that the code is only run once per button press.
            if (gamepad1.y && LastValueGP1Y != gamepad1.y)
                fc.newOffset();
            LastValueGP1Y = gamepad1.y;


            // The following code is realted to handling the servo that is used for picking up and
            // placing down the cone.  After the user releases the trigger, the servo will go
            // through a period of ramp down in speed, this helps extend out the motion and allow for
            // immediate restart of the motion if needed.
            if (gamepad2.left_bumper || activated(gamepad2.left_trigger)) {
                // Pickup Cone / UP
                servopower=-1 * servoMax;
                resetRuntime();
            }
            else if (gamepad2.right_bumper || activated(gamepad2.right_trigger)) {
                // Drop Conde / Down
                servopower=1 * servoMax;
                resetRuntime();
            }
            else if (servopower == 0){
                // Do nothing, but prevent the rest of the else clauses from being checked.
            }
            else if ( java.lang.Math.abs(servopower) > 0 &&  java.lang.Math.abs(servopower) < ServoThershold){
                // Within the threashold to stop the servo.
                servopower=0;
            }
            else if (servopower<0 && getRuntime() > servoStepTime){
                servopower=servopower*ServoUpMuitly;
                resetRuntime();
            } else if (servopower>0 && getRuntime() > servoStepTime){
                servopower=servopower*ServoDownMuilty;
                resetRuntime();
            }
            coneServo.setPower(servopower); // Set the actual Servo power based on all the logic.
        } // End main code Loop

        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
    public boolean activated(float val) {
        return Math.abs(val) > 0.075;
    }
}
