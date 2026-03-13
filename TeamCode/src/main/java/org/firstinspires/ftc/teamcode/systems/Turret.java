package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private CRServo turret;
    private DcMotorEx encoder;

    private double targetAngle;

    // BIG ERROR PID (your current one)
    double kP_big = 0.008;
    double kD_big = 0.00001;

    // SMALL ERROR PID (precision PID)
    double kP_small = 0.02;
    double kD_small = 0.0002;

    // threshold between them
    double switchThreshold = 15; // degrees

    private double lastError;
    private long lastTime;

    private static final double TICKS_PER_REV = 8192.0;

    public Turret(HardwareMap hardwareMap, DcMotorEx turretE) {
        turret = hardwareMap.get(CRServo.class, "Turret");
        encoder = turretE;

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
    }

    public double teoXSqrd(double x, double vMin) {
        if (x == 0) return 0;

        double speed = 1.55 * x * x;
        double sign = x / Math.abs(x);

        if (vMin > Math.abs(speed)) return vMin * sign;
        else if (Math.abs(speed) > 1) return sign;
        else return speed * sign;
    }

    public void setTurretDegrees(double angle) {
        if (angle < -120) angle = -120;
        else if (angle > 120) angle = 120;


    }

    public void setTargetAngle(double angle) {
        if (angle > -120 && angle < 120) {
            targetAngle = angle;
        }
    }

    public double getCurrentAngle() {
        return (double) encoder.getCurrentPosition() / 6 / TICKS_PER_REV * 360.0;
    }

    public void update() {

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double current = getCurrentAngle();
        double error = targetAngle - current;

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        double derivative = (error - lastError) / dt;

        double power;

        // choose which PID to use
        if (Math.abs(error) > switchThreshold) {
            power = kP_big * error + kD_big * derivative;
        } else {
            power = kP_small * error + kD_small * derivative;
        }

        power = Math.max(-1, Math.min(1, power));

        if (Math.abs(error) < 1) power = 0;

        turret.setPower(-power);

        lastError = error;
    }
}