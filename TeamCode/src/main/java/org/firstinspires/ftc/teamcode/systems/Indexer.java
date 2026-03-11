package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.PULL_SPEED;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.PUSH_SPEED;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.P;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.I;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.D;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IndexerConstants.F;

public class Indexer {

    private DcMotorEx indexer;

    private static final double TICKS_PER_REV = 103.8;

    public Indexer(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setDirection(DcMotorEx.Direction.FORWARD);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        indexer.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void pull() {
        indexer.setVelocity(PULL_SPEED);
    }

    public void push() {
        indexer.setVelocity(-PUSH_SPEED);
    }

    public void off() {
        indexer.setVelocity(0);
    }

    public double getRPM() {
        double ticksPerSecond = indexer.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
}
