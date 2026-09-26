package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class IntakeController {

    private State state;
    private final Timer stateTimer = new Timer();

    private boolean isBusy;
    private final Robot robot;
    private final TelemetryUtils tm;

    private enum State {
        IDLE,
        INNER_INTAKE,
        INTAKE,
        MANUAL_INTAKE,
        OUTTAKE,
    }

    public IntakeController(Robot robot) {
        tm = robot.drivetrain.tm;
        setState(State.IDLE);
        this.robot = robot;
        isBusy = false;
    }

    /**
     * Updates the LEDs and the IntakeController state machine:<p>
     * INTAKE -> IDLE: When indexer is full<p>
     * Other transitions controlled via calling methods
     * @see #intake()
     * @see #innerIntake()
     * @see #outtake()
     * @see #stop()
     */
    public void update() {
        switch (state) {
            case IDLE:
                RobotState.normalIntaking = false;
                isBusy = false;
                robot.intake.power(0);
                break;
            case INNER_INTAKE:
                RobotState.normalIntaking = false;
                isBusy = false;
                robot.intake.powerInside(-1);
                robot.intake.powerOutside(0);
                break;
            case MANUAL_INTAKE:
            case INTAKE:
                RobotState.normalIntaking = true;
                robot.intake.power(-1);
                break;
            case OUTTAKE:
                RobotState.normalIntaking = false;
                robot.intake.power(1);
                break;
        }
    }

    /**
     * @return Whether the robot is actively intaking
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Turns outtaking on
     */
    public void outtake() {
        isBusy = true;
        setState(State.OUTTAKE);
    }

    /**
     * Turns intaking on
     */
    public void intake() {
        isBusy = true;
        setState(State.INTAKE);
    }

    public void manualIntake() {
        isBusy = true;
        setState(State.MANUAL_INTAKE);
    }

    /**
     * Turns on intaking the inner intake only
     */
    public void innerIntake() {
        setStateNoWait(State.INNER_INTAKE);
        isBusy = false;
    }

    /**
     * Stops the intake only if it is inner intaking
     */
    public void stopInnerIntake() {
        if (state == State.INNER_INTAKE) setStateNoWait(State.IDLE);
    }

    /**
     * Stops the intake
     */
    public void stop() {
        if (state != State.IDLE) setState(State.IDLE);
        isBusy = false;
    }

    private void setState(State state) {
        if (state != null && state != this.state)
            tm.log("IntakeController: " + this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    /**
     * Gets the IntakeController's current state
     *
     * @return Current IntakeController state as a String
     */
    public String getState() {
        return state.toString();
    }
}
