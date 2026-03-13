package org.firstinspires.ftc.teamcode.opModes.teleOp.use;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.global.AllianceColor;
import org.firstinspires.ftc.teamcode.global.DrivingType;
import org.firstinspires.ftc.teamcode.global.Poses;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.global.GamepadsConstants;
import java.io.FileReader;
import java.io.IOException;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;
import java.util.Objects;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.systems.StaticTurret;
import org.firstinspires.ftc.teamcode.systems.Stopper;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "MainTeleOp", group = "use")
public class MainTeleOp extends OpMode {

    private Intake intake;
    private Indexer indexer;
    private Deflector deflector;
    private Outtake outtake;
    //private Turret turret;
    private Stopper stopper;

    private StaticTurret turret;

    private IMU imu;
    private double imuOffset;
    private double lastImuYaw = 0;
    private double continuousHeading = 0;
    private Limelight limelight;

    private IntakingManager intakingManager;
    private ShootingManager shootingManager;
    double error;
    private ElapsedTime timer;
    private ElapsedTime cameraTimer;

    private Follower follower;

    private double forward = 0.0, strafe = 0.0, rotation = 0.0;
    private boolean turretAim = true;
    private boolean isRelocalizing = false;

    private boolean gamepad1IsActive = false;
    private DrivingType drivingTypeGm1 = DrivingType.ROBOT_CENTRIC;
    private double gamepad1Coef = 1.0;

    private String allianceColor;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        follower = Constants.createFollower(hardwareMap);

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileReader reader = new FileReader(file)) {
            JsonParser parser = new JsonParser();
            JsonObject json = parser.parse(reader).getAsJsonObject();

            double startPoseX = json.get("x").getAsDouble();
            double startPoseY = json.get("y").getAsDouble();
            double startPoseHeading = json.get("heading").getAsDouble();
            allianceColor = json.get("color").getAsString();

            follower.setStartingPose(new Pose (startPoseX, startPoseY, startPoseHeading));
        } catch (IOException e) {
            //follower.setStartingPose(new Pose(18.392523364485978, 120, Math.toRadians(-37)));

        }
        //follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();
        imuOffset = Math.toDegrees(follower.getPose().getHeading());

        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        //turret = new Turret(hardwareMap, indexer.getTurret());
        turret = new StaticTurret(hardwareMap);
        deflector = new Deflector(hardwareMap);
        outtake = new Outtake(hardwareMap);
        stopper = new Stopper(hardwareMap);

        if (allianceColor.equals(AllianceColor.BLUE.toString())) {
            limelight = new Limelight(hardwareMap, 0);
        }
        else {
            limelight = new Limelight(hardwareMap, 1);
        }

        intakingManager = new IntakingManager(intake);
        shootingManager = new ShootingManager(outtake, stopper, deflector, indexer, intakingManager);

        timer = new ElapsedTime();
        cameraTimer = new ElapsedTime();
    }

    @Override
    public void start() {

        follower.startTeleOpDrive();
        timer.reset();
        cameraTimer.reset();
        lastImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        //turret.update();
        turret.stuck();
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double delta = currentYaw - lastImuYaw;

        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        continuousHeading += delta;
        lastImuYaw = currentYaw;

        gamepad1IsActive = Math.abs(gamepad1.left_stick_y) > GamepadsConstants.GAMEPAD_TRASHOLD ||
                Math.abs(gamepad1.left_stick_x) > GamepadsConstants.GAMEPAD_TRASHOLD ||
                Math.abs(gamepad1.right_stick_x) > GamepadsConstants.GAMEPAD_TRASHOLD ||
                Math.abs(gamepad1.right_trigger) > GamepadsConstants.GAMEPAD_TRASHOLD;

        if (gamepad1.shareWasPressed()) {
            drivingTypeGm1 = drivingTypeGm1 == DrivingType.ROBOT_CENTRIC
                    ? DrivingType.FIELD_CENTRIC
                    : DrivingType.ROBOT_CENTRIC;
        }

        if (gamepad1.left_trigger > 0.1) gamepad1Coef = 0.4;
        else gamepad1Coef = 1.0;

        forward = -gamepad1.left_stick_y * gamepad1Coef;
        strafe = -gamepad1.left_stick_x * gamepad1Coef;
        rotation = -
                gamepad1.right_stick_x * gamepad1Coef;

        if (drivingTypeGm1 == DrivingType.ROBOT_CENTRIC) {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    rotation,
                    true
            );
        } else {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    rotation,
                    false
            );
        }

        if (gamepad1.rightBumperWasPressed()) shootingManager.shoot(1);
        if(gamepad1.leftBumperWasPressed()) shootingManager.shoot(2);
        if(gamepad1.right_trigger > 0.1){
            intakingManager.pull();
        }
        else{
            intakingManager.off();
        }
        if(gamepad1.aWasPressed()) turretAim = !turretAim;

        follower.update();

            double targetHeading = Math.atan2(
                    Poses.blueGoalPose.getY() - follower.getPose().getY(),
                    Poses.blueGoalPose.getX() - follower.getPose().getX()
            );

            double currentHeading = follower.getPose().getHeading();

            error = targetHeading - currentHeading;
           // turret.setTurretDegrees(Math.toDegrees(-error));

        /*

        if (allianceColor.equals(AllianceColor.RED.toString())) {
            shootingManager.update(
                    follower.getPose().distanceFrom(Poses.redGoalPose),
                    timer.seconds(),
                    follower.poseTracker.getVelocity(),
                    Math.atan2((Poses.redGoalPose.getY() - follower.getPose().getY()), (Poses.redGoalPose.getX() - follower.getPose().getX()))
            );
        } else if (allianceColor.equals(AllianceColor.BLUE.toString())) {
            shootingManager.update(
                    follower.getPose().distanceFrom(Poses.blueGoalPose),
                    timer.seconds(),
                    follower.poseTracker.getVelocity(),
                    Math.atan2((Poses.blueGoalPose.getY() - follower.getPose().getY()), (Poses.blueGoalPose.getX() - follower.getPose().getX()))
            );
        }
        */


        shootingManager.update(
                follower.getPose().distanceFrom(Poses.blueGoalPose),
                timer.seconds(),
                follower.poseTracker.getVelocity(),
                Math.atan2((Poses.blueGoalPose.getY() - follower.getPose().getY()), (Poses.blueGoalPose.getX() - follower.getPose().getX()))
        );

        /*
        telemetry.addData("distance", visualManager.getDistance());
        telemetry.addData("YAW", limelight.getYaw());
        telemetry.addData("speed", outtake.getRPM());
        telemetry.addData("target", outtake.getTargetRPM());
        telemetry.addData("Distance", follower.getPose().distanceFrom(Poses.blueGoalPose));
        */

        intakingManager.update();
        timer.reset();

        Pose visionPose = limelight.getPose();

        if (gamepad1.aWasPressed()){
            isRelocalizing = true;
        }

        if (visionPose != null && isRelocalizing) {
            visionPose.setHeading(Math.toRadians(continuousHeading + imuOffset));
            /*
            telemetry.addData("x", visionPose.getX());
            telemetry.addData("y", visionPose.getY());
            telemetry.addData("angle", Math.toDegrees(visionPose.getHeading()));
             */
            telemetry.addLine("relocalized");
            isRelocalizing = false;

            follower.setPose(visionPose);
            telemetry.update();
        }

        telemetry.addData("error", Math.toDegrees(error));
        /*
        telemetry.addData("angle from imu", continuousHeading + imuOffset);
        telemetry.addData("curent pose x", follower.getPose().getX());
        telemetry.addData("curent pose y", follower.getPose().getY());
        telemetry.addData("curent pose h", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("distance", follower.getPose().distanceFrom(Poses.blueGoalPose));

         */
        telemetry.addData("rpm", outtake.getRPM());


        telemetry.addData("angle deflector", deflector.getPose());

        /*telemetry.addData("target angle", Math.toDegrees(-error));
        telemetry.addData("delta", turret.deltaAngle);
        telemetry.addData("interpolator", turret.interpolatorTurret);
        telemetry.addData("current angle", turret.getCurrentAngle());

         */
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}