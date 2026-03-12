package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private CRServo turret;
    private DcMotorEx encoder;

    private double targetAngle;

    double kP = 0.005;
    double kD = 0.00001;

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

    public void setTargetAngle(double angle) {
        if(angle > -120 && angle < 120) {
            targetAngle = angle;
        }
        else{
            targetAngle = 0;
        }
    }

    public double getCurrentAngle() {
        return (double) encoder.getCurrentPosition() / 6 / TICKS_PER_REV * 360.0;
    }

    public void update() {

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // seconds
        lastTime = now;

        double current = getCurrentAngle();

        double error = targetAngle - current - 7;

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        double derivative = (error - lastError) / dt;

        double power = kP * error + kD * derivative;

        power = Math.max(-1, Math.min(1, power));

        if (Math.abs(error) < 1) power = 0;



        turret.setPower(-power);

        lastError = error;
    }
}
