package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StaticTurret {

    private Servo turret;

    public StaticTurret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "servo");
    }

    public void stuck() {
        turret.setPosition(0.7);
    }
}