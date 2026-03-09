package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IntakeConstants.PULL_POWER;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IntakeConstants.PUSH_POWER;

import androidx.appcompat.widget.TintTypedArray;

public class Intake {

    private final DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void pull() {
        intake.setPower(PULL_POWER);
    }

    public void push() {
        intake.setPower(PUSH_POWER);
    }

    public void stop() {
        intake.setPower(0.0);
    }
}
