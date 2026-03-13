package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.global.SystemsConstants;

import static org.firstinspires.ftc.teamcode.global.SystemsConstants.IntakeConstants.INTAKE_STALL_CURRENT;

public class IntakingManager {
    private final ElapsedTime timer = new ElapsedTime();
    private final Intake intake;

    private enum State {
        IDLE,
        PULL,
        SHOOT,
        REVERSE
    }

    private State currentState = State.IDLE;

    public IntakingManager(Intake intake) {
        this.intake = intake;
    }

    public void togglePull() {
        currentState = (currentState == State.PULL) ? State.IDLE : State.PULL;
        timer.reset();
    }
    public void pull(){
        if(currentState == State.IDLE){
            currentState = State.PULL;
        }
    }

    public void reverse(){
        currentState = State.REVERSE;
    }

    public void shootPull() {
        currentState = State.SHOOT;
    }

    public void off() {
        currentState = State.IDLE;
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public void update() {

        switch (currentState) {

            case IDLE:
                intake.stop();
                break;

            case PULL:

                if (intake.getCurrent() >= INTAKE_STALL_CURRENT && timer.milliseconds() > 300) {
                    intake.stop();
                    currentState = State.IDLE;
                    break;
                }

                intake.pull();
                break;

            case SHOOT:
                intake.pull();
                break;

            case REVERSE:
                intake.push();
                break;
        }
    }
}