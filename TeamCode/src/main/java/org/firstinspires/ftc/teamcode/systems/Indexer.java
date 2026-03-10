package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Indexer {

    private DcMotorEx indexer;

    public Indexer(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setDirection(DcMotorEx.Direction.FORWARD);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setVelocityPIDFCoefficients(0,0,0,0.001);
    }

    public void pull() {
        indexer.setVelocity(11210.4);
    }

    public void push() {
        indexer.setPower(-1.0);
    }

    public void off() {
        indexer.setPower(0.0);
    }

    public double getSpeed() {
        return indexer.getVelocity();
    }

}