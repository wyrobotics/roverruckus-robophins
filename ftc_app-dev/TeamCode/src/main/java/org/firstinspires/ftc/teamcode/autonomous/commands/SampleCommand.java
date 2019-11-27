package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class SampleCommand extends Command {
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException {
        Thread.sleep(500);
        SamplingConfiguration samplingConfiguration = visionProcessor.getSamplingConfigurationPhoneLeftOnlyGold();
        if (samplingConfiguration == null) {
            telemetry.addLine("Didn't get sampling configuration, defaulting to center");
            samplingConfiguration = SamplingConfiguration.CENTER;
        } else {
            telemetry.addData("Got sampling configuration", samplingConfiguration);
        }
        telemetry.update();
        visionProcessor.stopTfod();

        switch (navigationalState.startLocation) {
            case BLUE_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case BLUE_RIGHT:
                (new MovementCommand(-304.8f - 150, -304.8f - 150, -135, false)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                switch (samplingConfiguration) {
                    case LEFT:
                        (new MovementCommand(-609.6f + 150, -1219.2f - 150, -90, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                    case RIGHT:
                        (new MovementCommand(-1219.2f - 150, -609.6f + 150, 180, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                }
                //(new MovementCommand(-1219.2f, -1219.2f, -135, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                break;
            case RED_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case RED_RIGHT:
                (new MovementCommand(304.8f + 150, 304.8f + 150, 45, false)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                switch (samplingConfiguration) {
                    case LEFT:
                        (new MovementCommand(1219.2f + 150, 609.6f - 150, 0, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                    case RIGHT:
                        (new MovementCommand(609.6f - 150, 1219.2f + 150, 90, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                }
                //(new MovementCommand(1219.2f, 1219.2f, 45, true)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                break;
        }
    }

    private void servoSample(SamplingConfiguration samplingConfiguration, MainRobot mainRobot) throws InterruptedException {
        int time = 1000;
        switch (samplingConfiguration) {
            case LEFT:
                mainRobot.sampler.extendLeft();
                Thread.sleep(time);
                mainRobot.sampler.contractLeft();
                break;
            case RIGHT:
                mainRobot.sampler.extendRight();
                Thread.sleep(time);
                mainRobot.sampler.contractRight();
                break;
            case CENTER:
                mainRobot.sampler.extendCenter();
                Thread.sleep(time);
                mainRobot.sampler.contractCenter();
                break;
        }
    }
}
