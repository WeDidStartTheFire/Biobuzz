package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public abstract class BaseAuto<S extends Enum<S>> extends OpMode {
    protected Robot robot;
    protected IntakeController intakeController;
    protected Pose startPose, shootPose;
    protected TelemetryUtils tm;
    protected Timer stateTimer = new Timer();
    protected S state;
    protected S initialState;
    protected Color color;
    protected String name;
    private double lastUpdateTime = 0;
    private int totalMs = 0;
    private int totalUpdates = 0;

    protected abstract void buildPaths();

    protected abstract void pathUpdate();

    protected abstract void configure();

    protected void onInit() {
    }

    protected void onStart() {
    }

    protected void onStop() {
    }

    protected void setState(S state) {
        stateTimer.resetTimer();
        tm.log(this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
    }

    protected void setStateNoWait(S state) {
        this.state = state;
    }

    @Override
    public final void init() {
        configure();
        RobotState.auto = true;
        RobotState.color = color;
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.follower.setStartingPose(startPose);
        tm = robot.drivetrain.tm;
        buildPaths();
        intakeController = new IntakeController(robot);
        tm.print(name + " auto initialized");
        tm.update();
        setStateNoWait(initialState);
        robot.initBulkCache();
        onInit();
    }

    @Override
    public final void start() {
        robot.drivetrain.follower.setPose(startPose);
        robot.limelight.start();
        robot.turret.setTarget(Turret.Target.GOAL);
        setState(initialState);
        onStart();
    }

    @Override
    public final void loop() {
        robot.updateBulkCache();
        robot.drivetrain.follower.update();
        pose = robot.drivetrain.follower.getPose();
        vel = robot.drivetrain.follower.getVelocity();
        pathUpdate();
        robot.turret.update(true);
        intakeController.update();
        robot.led.update();

        tm.drawRobot(robot.drivetrain.follower, 250);
        tm.print("Path State", state);
        tm.print("Intake State", intakeController.getState());
        if (pose != null) tm.print(pose);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose, null));
        tm.print("Launcher Vel", robot.launcher.getCachedVel());
        double t = getRuntime();
        if (lastUpdateTime != 0) {
            int ms = (int) ((t - lastUpdateTime) * 1000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms);
            tm.print("avg dt (ms)", totalMs / totalUpdates);
        }
        lastUpdateTime = t;
        tm.updateOnlyPanels(5);
    }

    @Override
    public final void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.breakFollowing();
        pose = robot.drivetrain.follower.getPose();
        if (pose != null) saveOdometryPosition(pose);
        intakeController.stop();
        robot.limelight.stop();
        tm.showLogs();
        tm.update();
        onStop();
    }
}
