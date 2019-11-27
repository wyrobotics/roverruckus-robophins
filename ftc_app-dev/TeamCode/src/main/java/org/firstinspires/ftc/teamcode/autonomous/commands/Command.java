package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public abstract class Command {
    private String name() {
        return this.getClass().getSimpleName();
    }

    public final void execute(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException {
        System.out.println("Executing Command: " + name());
        telemetry.addData("Executing Command", name());
        telemetry.update();
        executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
    }

    abstract void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException;
}
