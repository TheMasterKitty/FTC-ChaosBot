package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotDirections {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    BNO055IMU imu;
    LinearOpMode opmode;
    public double power;
    public RobotDirections(LinearOpMode self, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double defaultPower) {
        opmode = self;
        fl = frontLeft;
        fr = frontRight;
        rl = backLeft;
        rr = backRight;
        imu = self.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
        power = defaultPower;
    }
    public void runTicks(int ticks, Directions direction) {
        if (direction == Directions.Forward) {
            fl.setTargetPosition(ticks + fl.getCurrentPosition());
            fr.setTargetPosition(ticks + fr.getCurrentPosition());
            rl.setTargetPosition(ticks + rl.getCurrentPosition());
            rr.setTargetPosition(ticks + rr.getCurrentPosition());
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(power);
            fr.setPower(power);
            rl.setPower(power);
            rr.setPower(power);
        }
        else if (direction == Directions.Right) {
            fl.setTargetPosition(ticks + fl.getCurrentPosition());
            fr.setTargetPosition(-ticks + fr.getCurrentPosition());
            rl.setTargetPosition(ticks + rl.getCurrentPosition());
            rr.setTargetPosition(-ticks + rr.getCurrentPosition());
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(power);
            fr.setPower(-power);
            rl.setPower(power);
            rr.setPower(-power);
        }
        else if (direction == Directions.Backward) {
            fl.setTargetPosition(-ticks + fl.getCurrentPosition());
            fr.setTargetPosition(-ticks + fr.getCurrentPosition());
            rl.setTargetPosition(-ticks + rl.getCurrentPosition());
            rr.setTargetPosition(-ticks + rr.getCurrentPosition());
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(-power);
            fr.setPower(-power);
            rl.setPower(-power);
            rr.setPower(-power);
        }
        else if (direction == Directions.Left) {
            fl.setTargetPosition(-ticks + fl.getCurrentPosition());
            fr.setTargetPosition(ticks + fr.getCurrentPosition());
            rl.setTargetPosition(-ticks + rl.getCurrentPosition());
            rr.setTargetPosition(ticks + rr.getCurrentPosition());
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setPower(-power);
            fr.setPower(power);
            rl.setPower(-power);
            rr.setPower(power);
        }
        while (opmode.opModeIsActive() && fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) { }
    }

    public void TurnTo(Directions newDirection) {
        TurnTo(newDirection, 1);
    }
    public void TurnTo(Directions newDirection, int leeway) {
        int newAngle = 0;
        if (newDirection == Directions.Right) newAngle = 90;
        else if (newDirection == Directions.Backward) newAngle = 180;
        else if (newDirection == Directions.Left) newAngle = -90;
        if (newAngle != Math.round(imu.getAngularOrientation().firstAngle)) {
            int updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
            if (newAngle > updatedAngle)
                while (newAngle > updatedAngle + leeway && opmode.opModeIsActive()) {
                    fl.setPower(-power);
                    fr.setPower(power);
                    rl.setPower(power);
                    rr.setPower(-power);
                    updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
                }
            else
                while (newAngle < updatedAngle - leeway && opmode.opModeIsActive()) {
                    fl.setPower(power);
                    fr.setPower(-power);
                    rl.setPower(-power);
                    rr.setPower(power);
                    updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
                }
        }
    }
    public void TurnTo(int angleDegrees) {
        TurnTo(angleDegrees, 2);
    }
    public void TurnTo(int angleDegrees, int leeway) {
        int angle = Math.round(imu.getAngularOrientation().firstAngle);
        if (angleDegrees != angle) {
            if (angle < 0) angle += 360;
            int updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
            if (angle - angleDegrees < angleDegrees - angle)
                while (angleDegrees > updatedAngle + leeway && opmode.opModeIsActive()) {
                    fl.setPower(-power);
                    fr.setPower(power);
                    rl.setPower(power);
                    rr.setPower(-power);
                    updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
                }
            else
                while (angleDegrees < updatedAngle - leeway && opmode.opModeIsActive()) {
                    fl.setPower(power);
                    fr.setPower(-power);
                    rl.setPower(-power);
                    rr.setPower(power);
                    updatedAngle = Math.round(imu.getAngularOrientation().firstAngle);
                }
        }
    }
}