package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.intakeTests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Test TeleOp", group = "dev")
public class IntakeTestTeleOp extends OpMode {

    private DcMotorEx intake;

    private TelemetryManager panelsTelemetry;

    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        intake.setPower(1.0);

        panelsTelemetry.addData("Amps", intake.getCurrent(CurrentUnit.MILLIAMPS));
        panelsTelemetry.addData("V", voltageSensor.getVoltage());
        panelsTelemetry.update();
    }
}
