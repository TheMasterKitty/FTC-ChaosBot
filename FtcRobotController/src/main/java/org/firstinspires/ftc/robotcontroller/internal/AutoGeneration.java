package org.firstinspires.ftc.robotcontroller.internal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="AutoGeneration")
public class AutoGeneration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx rl = hardwareMap.get(DcMotorEx.class, "rl");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(params);

        List<String> logs = new ArrayList<>();

        DcMotorEx[] loggingMotors = new DcMotorEx[] { fl, fr, rl, rr };

        for (DcMotorEx motor : loggingMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Initialized and Ready to Debug!");
        telemetry.update();

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        rr.setDirection(DcMotorEx.Direction.REVERSE);

        for (DcMotorEx motor : loggingMotors) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        boolean turning = false;

        int moving = 0;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive()) {
            packet = new TelemetryPacket();
            for (String str : logs) {
                telemetry.addLine(str);
                packet.addLine(str);
            }
            for (DcMotorEx motor : loggingMotors) {
                telemetry.addLine(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                packet.addLine(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
            }
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

            if (!turning && gamepad1.a && (Math.abs(loggingMotors[0].getCurrentPosition()) > 1 || Math.abs(loggingMotors[1].getCurrentPosition()) > 1 || Math.abs(loggingMotors[2].getCurrentPosition()) > 1 || Math.abs(loggingMotors[3].getCurrentPosition()) > 1)) {
                for (DcMotorEx motor : loggingMotors) {
                    logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
                logs.add("| BREAK |");
                for (DcMotorEx motor : loggingMotors) {
                    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
            }
            if (!turning && (gamepad1.dpad_up || gamepad1.dpad_down) && !(gamepad1.dpad_left || gamepad1.dpad_right) && !gamepad1.b) {
                float power = 0;
                if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                    power = -1;
                    if (moving != 0) {
                        for (DcMotorEx motor : loggingMotors) {
                            logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        logs.add("| BREAK (-1, -1, -1, -1) |");
                        for (DcMotorEx motor : loggingMotors) {
                            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        }
                    }
                    moving = 0;
                }
                else if (!gamepad1.dpad_up && gamepad1.dpad_down) {
                    power = 1;
                    if (moving != 2) {
                        for (DcMotorEx motor : loggingMotors) {
                            logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        logs.add("| BREAK (1, 1, 1, 1) |");
                        for (DcMotorEx motor : loggingMotors) {
                            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        }
                    }
                    moving = 2;
                }
                fl.setVelocity(power * 1000);
                fr.setVelocity(power * 1000);
                rl.setVelocity(power * 1000);
                rr.setVelocity(power * 1000);
            }
            else if (!turning && !(gamepad1.dpad_up || gamepad1.dpad_down) && (gamepad1.dpad_left || gamepad1.dpad_right) && !gamepad1.b) {
                float power = 0;
                if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                    power = 1;
                    if (moving != 3) {
                        for (DcMotorEx motor : loggingMotors) {
                            logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        logs.add("| BREAK (1, -1, -1, 1) |");
                        for (DcMotorEx motor : loggingMotors) {
                            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        }
                    }
                    moving = 3;
                }
                else if (!gamepad1.dpad_left && gamepad1.dpad_right) {
                    power = -1;
                    if (moving != 1) {
                        for (DcMotorEx motor : loggingMotors) {
                            logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        logs.add("| BREAK (-1, 1, 1, -1) |");
                        for (DcMotorEx motor : loggingMotors) {
                            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        }
                    }
                    moving = 1;
                }
                fl.setVelocity(power * 1000);
                fr.setVelocity(-power * 1000);
                rl.setVelocity(-power * 1000);
                rr.setVelocity(power * 1000);
            }
            else if (gamepad1.b && (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down)) {
                if (gamepad1.dpad_down || gamepad1.dpad_up) {
                    turning = false;
                    fl.setVelocity(0);
                    fr.setVelocity(0);
                    rl.setVelocity(0);
                    rr.setVelocity(0);
                    for (DcMotorEx motor : loggingMotors) {
                        logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    logs.add("| BREAK |");
                    for (DcMotorEx motor : loggingMotors) {
                        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                }
                if (!turning && (gamepad1.dpad_left || gamepad1.dpad_right)) {
                    for (DcMotorEx motor : loggingMotors) {
                        logs.add(hardwareMap.getNamesOf(motor) + ": " + motor.getCurrentPosition());
                        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    for (DcMotorEx motor : loggingMotors) {
                        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    if (gamepad1.dpad_right) {
                        turning = true;
                        logs.add("| TURN (1, -1, 1, -1) |");
                        fl.setVelocity(500);
                        fr.setVelocity(-500);
                        rl.setVelocity(500);
                    }
                    else if (gamepad1.dpad_left) {
                        turning = true;
                        logs.add("| TURN (1, -1, 1, -1) |");
                        fl.setVelocity(500);
                        fr.setVelocity(-500);
                        rl.setVelocity(500);
                        rr.setVelocity(-500);
                    }
                }
            }
            else if (!turning) {
                fl.setVelocity(0);
                fr.setVelocity(0);
                rl.setVelocity(0);
                rr.setVelocity(0);
            }
        }

        fl.setVelocity(0);
        fr.setVelocity(0);
        rl.setVelocity(0);
        rr.setVelocity(0);
    }
}
