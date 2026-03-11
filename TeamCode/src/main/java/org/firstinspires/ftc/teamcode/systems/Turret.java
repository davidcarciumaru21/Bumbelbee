package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;

public class Turret {

    private CRServo turret;
    private DcMotorEx encoder;

    private double targetAngle;

    private double kP = 0.015;
    private double kD = 0.0006;

    private double lastError;

    private static final double TICKS_PER_REV = 8192.0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(CRServo.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getCurrentAngle() {
        return encoder.getCurrentPosition() / TICKS_PER_REV * 360.0;
    }

    public void update() {

        double current = getCurrentAngle();

        double error = targetAngle - current;

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        double derivative = error - lastError;

        double power = kP * error + kD * derivative;

        power = Math.max(-1, Math.min(1, power));

        if (Math.abs(error) < 1) power = 0;

        turret.setPower(power);

        lastError = error;
    }
}