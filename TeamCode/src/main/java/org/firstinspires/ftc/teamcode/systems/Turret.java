package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;

public class Turret {

    private final Servo turret;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "turret");
    }

    public void setTargetAngleDegrees(double angle) {

        double position = angle / 360 * 4;

        if (angle <= 180) turret.setDirection(Servo.Direction.FORWARD);
        else if (angle > 180) turret.setDirection(Servo.Direction.REVERSE);

        turret.setPosition(position);
    }

    public double getCurrentAngle() {
        return turret.getPosition() * 360 * 4;
    }
}