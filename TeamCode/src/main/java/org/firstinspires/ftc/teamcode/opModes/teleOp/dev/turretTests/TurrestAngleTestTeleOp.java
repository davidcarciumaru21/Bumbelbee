package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.turretTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "turretAngleTestTeleOp", group = "dev")
public class TurrestAngleTestTeleOp extends OpMode {

    private Turret turret;
    private Indexer indexer;
    private double targetAngle = 0.0;

    public void init() {
        indexer = new Indexer(hardwareMap);
        turret = new Turret(hardwareMap, indexer.getTurret());
    }

    public void loop() {
        if (gamepad1.dpadUpWasPressed()) targetAngle += 5;
        else if (gamepad1.dpadDownWasPressed()) targetAngle -= 5;

        turret.setTargetAngle(targetAngle);
        turret.update();

        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("currentAngle", turret.getCurrentAngle());
        telemetry.update();
    }
}
