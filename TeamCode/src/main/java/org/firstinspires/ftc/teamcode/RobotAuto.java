package org.firstinspires.ftc.teamcode;

/**
 * This class manages a basic autonomous that can use the automatic methods from the Robot to make it perform a set of pre-programmed tasks.
 *
 * @author Owen Sorber
 * @date June 12th, 2018
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Move to Position Automatic Mode 1", group = "Linear Opmode")
public class RobotAuto extends LinearOpMode {
    private Robot robot;
    private Hardware hardware;

    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);
        hardware.init();
        robot = new Robot(hardware, gamepad1);

        robot.moveToCoordinate(new Position(25, 25));
        robot.moveToCoordinate(new Position());
    }
}
