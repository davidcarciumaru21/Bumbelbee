package org.firstinspires.ftc.teamcode.opModes.teleOp.use;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Stopper;

@TeleOp(name = "ManualRobotTeleOp", group = "use")
public class ManualRobotTeleOp extends OpMode {

    private TelemetryManager panelsTelemetry;
    private VoltageSensor voltageSensor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private double x = 0.0, y = 0.0, rx = 0.0;
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;
    private double denominator = 1.0;

    private Outtake outtake;
    private Intake intake;
    private Deflector deflector;
    private Stopper stopper;
    private Indexer indexer;

    private double rpm = 0.0, pose = 0.0;

    private ElapsedTime timer;

    private double speedCoefficient = 1.0;

    @Override
    public void init() {
        timer = new ElapsedTime();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        backRightMotor = hardwareMap.dcMotor.get("BackRight");

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        deflector = new Deflector(hardwareMap);
        stopper = new Stopper(hardwareMap);
        indexer = new Indexer(hardwareMap);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger >= 0.3) speedCoefficient = 0.5;
        else if (gamepad1.left_trigger >= 0.3) speedCoefficient = 0.25;
        else speedCoefficient = 1.0;

        x = gamepad1.left_stick_x * speedCoefficient;
        y = -gamepad1.left_stick_y  * speedCoefficient;
        rx = gamepad1.right_stick_x  * speedCoefficient;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.dpadUpWasPressed()) rpm += 50;
        if (gamepad1.dpadDownWasPressed()) rpm -= 50;

        if (gamepad1.dpadRightWasPressed()) pose += 0.01;
        if (gamepad1.dpadLeftWasPressed()) pose -= 0.01;

        outtake.moveFlyWheelAtRPM(rpm);
        outtake.update(timer.seconds());
        deflector.move(pose);
        intake.pull();
        if (gamepad1.bWasPressed()) {
            stopper.open();
            indexer.pull();
        }
        if (gamepad1.aWasPressed()) {
            indexer.off();
            stopper.close();
        }
        timer.reset();

        telemetry.addData("rpm", rpm);
        telemetry.addData("pose", pose);
        telemetry.addData("outtake speed", outtake.getRPM());
        telemetry.addData("intake speed", indexer.getSpeed());
        telemetry.update();

        //telemetry.addData("watts", intake.getCurrent() * voltageSensor.getVoltage());
        panelsTelemetry.addData("current rpm", outtake.getRPM());;
        panelsTelemetry.addData("target rpm", rpm);
        panelsTelemetry.addData("power", outtake.getPower());
        panelsTelemetry.update();
    }
}