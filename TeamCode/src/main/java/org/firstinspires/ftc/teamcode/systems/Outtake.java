package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.OuttakeConstants.KP;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.OuttakeConstants.KI;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.OuttakeConstants.KS;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.OuttakeConstants.KV;

public class Outtake {

    private final DcMotorEx outtakeMotor1;
    private final DcMotorEx outtakeMotor2;
    private final VoltageSensor voltageSensor;

    private static final double TICKS_PER_REV = 28.0;

    public static double MAX_ACCEL_RPM_PER_SEC = 24000;
    public static double I_ENABLE_ERROR = 0;

    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double integral = 0;

    private double filteredVoltage = 13.0;
    private static final double alpha = 0.01;

    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor1  = hardwareMap.get(DcMotorEx.class, "OuttakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "OuttakeMotor2");

        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    //TODO
    public void moveBallAtInchesPerSeconds(double speed) {

        targetRPM = 2000;
    }

    public void moveFlyWheelAtRPM(double rpm) {
        targetRPM = rpm;
    }

    public void stop() {
        targetRPM = 0;
        rampedRPM = 0;
        integral = 0;
        setPower(0);
    }

    public double getRPM() {
        double ticksPerSec1 = outtakeMotor1.getVelocity();
        double ticksPerSec2 = outtakeMotor2.getVelocity();
        return ((ticksPerSec1 + ticksPerSec2) / 2.0 * 60.0) / TICKS_PER_REV;
    }

    public double getTargetRPM(){
        return targetRPM;
    }

    public double getPower() {
        return outtakeMotor1.getPower();
    }

    public void setPower(double power) {
        outtakeMotor1.setPower(power);
        outtakeMotor2.setPower(power);
    }

    public void update(double deltaTime) {

        double currentRPM = getRPM();

        double maxStep = MAX_ACCEL_RPM_PER_SEC * deltaTime;
        double diff = targetRPM - rampedRPM;
        rampedRPM += Range.clip(diff, -maxStep, maxStep);

        double ff = KS * Math.signum(rampedRPM)
                + KV * rampedRPM;

        double error = rampedRPM - currentRPM;
        double p = KP * error;

        if (Math.abs(error) < I_ENABLE_ERROR) {
            integral += error * deltaTime;
        }
        double i = KI * integral;

        double measuredVoltage = voltageSensor.getVoltage();
        filteredVoltage += alpha * (measuredVoltage - filteredVoltage);

        double power = (ff + p + i) / filteredVoltage;
        power = Range.clip(power, 0.0, 1.0);

        setPower(power);
    }

    public void changePI(double p, double i) {
        KP = p;
        KI = i;
    }

    public void changeKs(double s) {
        KS = s;
    }

    public void changeKv(double v) {
        KV = v;
    }

    public double getKs() {
        return KS;
    }

    public double getKv() {
        return KV;
    }
}