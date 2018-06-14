package org.firstinspires.ftc.teamcode;

/**
 * This class manages a basic Tele-Op that drives the robot using a controller.
 *
 * @author Ben Gerrard & Owen Sorber
 * @date June 12th, 2018
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot Driving Mode", group="Iterative Opmode") // Sends opmode information to Robot Controller
public class RobotTeleOp extends OpMode {
    private Robot robot;
    private Hardware hardware;
    private ElapsedTime elapsed;

    public void init() {
        hardware = new Hardware(hardwareMap);
        hardware.init();

        robot = new Robot(hardware, gamepad1);
        elapsed = new ElapsedTime();
    }

    private void manageTelemetry() {
        telemetry.addData("Elapsed Seconds", (int) elapsed.seconds());
        telemetry.addData("Total Distance Travelled (inches)", robot.getDistance());

        if (robot.isTurning())
            telemetry.addLine("Robot is currently turning.");
        else if (robot.isMoving())
            telemetry.addLine("Robot is currently moving.");
        else
            telemetry.addLine("Robot is currently not moving.");

        telemetry.addData("Robot Angle (Degrees)", robot.getAngle());
        telemetry.addData("Robot Angle (Radians)", robot.angleInRadians());
        telemetry.update();
    }

    public void loop() {
        robot.move();

        // Print out things to telemetry for de-bugging purposes
        manageTelemetry();
    }
}
