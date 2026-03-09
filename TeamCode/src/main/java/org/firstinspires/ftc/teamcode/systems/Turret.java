package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.TurretConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.TurretConstants.KP;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.TurretConstants.KI;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.TurretConstants.KD;

public class Turret {

    private final DcMotorEx turret;

    private double integral = 0;
    private double lastError = 0;

    private double targetTicks = 0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetAngleDegrees(double angle) {

        if (angle <= 180) {
            turret.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            turret.setDirection(DcMotorEx.Direction.REVERSE);
        }

        targetTicks = 4 * Math.abs((angle / 360.0) * TICKS_PER_REV);
    }

    public void update() {

        double current = turret.getCurrentPosition();

        double error = targetTicks - current;

        integral += error;

        double derivative = error - lastError;

        double power = (KP * error) + (KI * integral) + (KD * derivative);

        power = Math.max(-1.0, Math.min(1.0, power));

        turret.setPower(power);

        lastError = error;
    }

    public double getCurrentAngle() {
        return (turret.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }
}