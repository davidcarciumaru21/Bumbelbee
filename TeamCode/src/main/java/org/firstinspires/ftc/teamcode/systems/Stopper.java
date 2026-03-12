package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.StopperConstants.STOPPER_OPENED_POSITON;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.StopperConstants.STOPPER_CLOSED_POSITON;

public class Stopper {

    private final Servo stopper;

    public Stopper(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "Stopper");
    }

    public void open() {
        stopper.setPosition(STOPPER_OPENED_POSITON);
    }

    public void close() {
        stopper.setPosition(STOPPER_CLOSED_POSITON);
    }
}
