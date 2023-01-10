package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

public class FieldCentric {
    private DcMotor[] motors;
    private BNO055IMU imu;
    private double currentAngle;
    private double offsetAngle = 0;
    public double rotatedX;
    public double rotatedY;

    public void setUp(DcMotor[] motors, BNO055IMU imu) {
        this.motors = motors;
        this.imu = imu;
        newOffset();
    }

    private void getAngle() {
        currentAngle = -wrap((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offsetAngle) + Math.PI);
    }

    public void Drive(double x, double y, double turn) {

        getAngle();

        rotatedX = x * Math.cos(-currentAngle) - y * Math.sin(-currentAngle);
        rotatedY = y * Math.cos(currentAngle) - x * Math.sin(currentAngle);

        motors[0].setPower(rotatedY - rotatedX + turn);
        motors[1].setPower(rotatedY + rotatedX - turn);
        motors[2].setPower(rotatedY + rotatedX + turn);
        motors[3].setPower(rotatedY - rotatedX - turn);
    }

    public void newOffset() {
        offsetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Math.PI/2;
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
}