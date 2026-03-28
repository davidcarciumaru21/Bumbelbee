package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.ShootingConstants;
import org.firstinspires.ftc.teamcode.global.SystemsConstants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.systems.Indexer;

import org.firstinspires.ftc.teamcode.systems.Stopper;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pair;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.StopperConstants.TIME_TO_OPEN;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.StopperConstants.TIME_TO_CLOSE;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.ShooterConstants.THREE_BALLS_TIME;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.DeflectorConstants.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.global.SystemsConstants.DeflectorConstants.MAX_ANGLE;

public class ShootingManager {

    private final Outtake outtake;
    private final Stopper stopper;
    private final Deflector deflector;
    private final Indexer indexer;

    private final IntakingManager intakingManager;
    private boolean shooting = false;
    private enum State {
        IDLE,
        RAISE_STOPPER,
        SHOOT,
        SHOOTAUTO,
        PULL_STOPER,
    }

    private State currentState = State.IDLE;

    private final ElapsedTime timer;

    public ShootingManager(Outtake outtake,
                           Stopper stopper,
                           Deflector deflector,
                           Indexer indexer,
                           IntakingManager intakingManager) {
        this.outtake = outtake;
        this.stopper = stopper;
        this.deflector = deflector;
        this.indexer = indexer;

        this.intakingManager = intakingManager;

        this.timer = new ElapsedTime();
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public void setState(State state) {
        timer.reset();
        currentState = state;
    }

    public void shootA(){
        setState(State.SHOOTAUTO);
    }

    public void shoot(int button) {
        if(button == 1) {
            shooting = true;
        }
        else if(button == 2){
            shooting = false;
        }
        if(shooting) {
            if (currentState == State.IDLE) {
                setState(State.RAISE_STOPPER);
            }
        }
        else{
            setState(State.PULL_STOPER);
        }
    }

    public Pair<Double, Double> getTargetAngleAndVelocity(double distance,
                                                          Vector velocityVector,
                                                          double angleToGoal) {

        double height;
        double angle;
        double g;
        if (distance < 50){
            distance = distance - ShootingConstants.VeryClose.PASS_THROUGH_POINT_RADIUS;
            height =  ShootingConstants.VeryClose.SCORE_HEIGHT;
            angle = ShootingConstants.VeryClose.SCORE_ANGLE;
            g = ShootingConstants.VeryClose.g;
        }
        else if (distance < 125 && distance >= 50) {
            distance = distance - ShootingConstants.Close.PASS_THROUGH_POINT_RADIUS;
            height =  ShootingConstants.Close.SCORE_HEIGHT;
            angle = ShootingConstants.Close.SCORE_ANGLE;
            g = ShootingConstants.Close.g;
        } else {
            distance = distance - ShootingConstants.Far.PASS_THROUGH_POINT_RADIUS;
            height =  ShootingConstants.Far.SCORE_HEIGHT;
            angle = ShootingConstants.Far.SCORE_ANGLE;
            g = ShootingConstants.Far.g;
        }

        double initialAngle = MathUtils.clamp(
                Math.atan(2 * height / distance - Math.tan(angle)),
                MIN_ANGLE,
                MAX_ANGLE
        );

        double initialFlyWheelSpeed = Math.sqrt(
                (g * Math.pow(distance, 2)) /
                        (2 * Math.pow(Math.cos(initialAngle), 2) *
                                (distance * Math.tan(initialAngle)
                                        - height))
        );

        double tetha = velocityVector.getTheta() - angleToGoal;

        double paralelComponent =
                -Math.cos(tetha) * velocityVector.getMagnitude();

        double perpendicularComponent =
                Math.sin(tetha) * velocityVector.getMagnitude();

        double yFlyWheelSpeedComponent =
                initialFlyWheelSpeed * Math.sin(initialAngle);

        double time =
                distance / (initialFlyWheelSpeed * Math.cos(initialAngle));

        double xComponentCompensation =
                distance / time + paralelComponent;

        double newXSpeed = Math.sqrt(
                Math.pow(xComponentCompensation, 2)
                        + Math.pow(perpendicularComponent, 2)
        );

        double newDistance = newXSpeed * time;

        double newAngle = MathUtils.clamp(
                Math.atan(yFlyWheelSpeedComponent / newXSpeed),
                MIN_ANGLE,
                MAX_ANGLE
        );

        double flyWheelSpeed = Math.sqrt(
                (g * Math.pow(newDistance, 2)) /
                        (2 * Math.pow(Math.cos(newAngle), 2) *
                                (newDistance * Math.tan(newAngle)
                                        - height))
        );

        if (Double.isNaN(newAngle) || Double.isNaN(flyWheelSpeed)) {
            return new Pair<>(MIN_ANGLE, 0.0);
        } else {
            return new Pair<>(newAngle, flyWheelSpeed);
        }
    }

    private void applyTargets(Pair<Double, Double> velocityAndAngle) {
        deflector.moveAtAngleInRadians(velocityAndAngle.first);
        outtake.moveBallAtInchesPerSeconds(velocityAndAngle.second);
    }

    public void update(double distance, double time, Vector velocityVector, double angleToGoal) {

        applyTargets(getTargetAngleAndVelocity(distance, velocityVector, angleToGoal));
        outtake.update(time);

        switch (currentState) {

            case IDLE:
                indexer.off();
                stopper.close();
                break;

            case RAISE_STOPPER:
                stopper.open();
                indexer.push();

                if (timer.milliseconds() > TIME_TO_OPEN) setState(State.SHOOT);

                break;

            case SHOOT:
                intakingManager.shootPull();
                indexer.pull();
                //if (timer.milliseconds() > THREE_BALLS_TIME) setState(State.PULL_STOPER);

                break;
            case SHOOTAUTO:
                intakingManager.shootPull();
                indexer.pull();
                if (timer.milliseconds() > THREE_BALLS_TIME) setState(State.PULL_STOPER);

                break;
            case PULL_STOPER:
                indexer.off();
                stopper.close();
                if (timer.milliseconds() > TIME_TO_CLOSE) setState(State.IDLE);

                break;
        }
    }
}