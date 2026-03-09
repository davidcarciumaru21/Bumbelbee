package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.turretTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "turretAngleTestTeleOp", group = "dev")
public class TurrestAngleTestTeleOp extends OpMode {

    private Turret turret;
    private double targetAngle = 0.0;

    public void init() {
        turret = new Turret(hardwareMap);
    }

    public void loop() {
        if (gamepad1.dpadUpWasPressed()) targetAngle += 5;
        else if (gamepad1.dpadDownWasPressed()) targetAngle -= 5;

        turret.setTargetAngleDegrees(targetAngle);

        telemetry.addData("targetAngle", targetAngle);
        telemetry.update();
    }
}
