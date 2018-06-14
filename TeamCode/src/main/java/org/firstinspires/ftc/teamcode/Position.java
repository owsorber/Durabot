package org.firstinspires.ftc.teamcode;

/**
 * The Robot class contains the blueprint for all of the robot's properties, including position (Position object), angle, hardware (Hardware object), and controller (Gamepad object).
 *
 * @author Owen Sorber
 * @date June 8th, 2018
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

public class Robot {
    // Instance data
    private Position pos; // Robot's coordinate position
    private double distanceTraveled; // The total distance the robot has traveled in inches
    private int angle; // Direction robot is facing in degrees
    private int prevAngle; // Value of angle at last calibration of gyro sensor
    private Hardware hardware; // Hardware object for robot's hardware
    private Gamepad gamepad; // Controller to control robot
    private double leftPow, rightPow; // Doubles to alter power applied to motors

    private final static double pi = Math.PI;

    // Robot constants
    private final static double TETRIX_WHEEL_DIAMETER = 4; // Diameter of tetrix wheels (in inches)
    private final static double TETRIX_WHEEL_CIRCUMFERENCE = TETRIX_WHEEL_DIAMETER * pi; // Circumference of tetrix wheels (in inches)
    private final static double OMNI_DIAMETER = 3; // Diameter of omniwheels (in inches)
    private final static double OMNI_CIRCUMFERENCE = OMNI_DIAMETER * pi; // Circumference of omniwheels (in inches)
    private final static double NEVEREST_TICKS_PER_REV = 1120; // This is the encoder ticks per revolution of a Neverest 40 motor

    // Robot Constructor
    public Robot(Hardware robotHardware, Gamepad controller) {
        pos = new Position();
        distanceTraveled = 0;
        hardware = robotHardware;
        gamepad = controller;
        leftPow = 0;
        rightPow = 0;

        calibrateGyro(); // Sets gyro sensor to 0 degrees (similar to zeroing a scale)
        angle = 0; // Will equal zero due to calibration above
        prevAngle = 0; // Will equal zero due to calibration above

        resetEncoder(hardware.leftDrive);
        resetEncoder(hardware.rightDrive);
    }

    // Shortcut method for resetting encoders
    private void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Shortcut method for using encoders
    private void useEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Methods to convert from linear distance (inches) to encoder ticks and encoder ticks to linear distance (inches)
    private double linDistance(int encoderTicks) {
        return encoderTicks * TETRIX_WHEEL_CIRCUMFERENCE / NEVEREST_TICKS_PER_REV;
    }
    private int encoderTicks(int linearDistance) {
        return (int) (linearDistance * NEVEREST_TICKS_PER_REV / TETRIX_WHEEL_CIRCUMFERENCE);
    }

    private void calibrateGyro() {
        prevAngle = hardware.gyro.getHeading(); // Store current gyro val in prevAngle
        hardware.gyro.calibrate();
        while (hardware.gyro.isCalibrating()) {
            // WAIT UNTIL CALIBRATION IS COMPLETE - MAY TAKE ~3 SECONDS
        }
    }

    // Returns true if either of the triggers on the gamepad is being pressed
    public boolean triggerPressed() {
        return gamepad.right_trigger > 0 || gamepad.left_trigger > 0;
    }

    public boolean rightJoystickPressed() {
        return gamepad.right_stick_x != 0;
    }

    // Returns true if the robot is currently moving
    public boolean isMoving() {
        return hardware.leftDrive.getPower() != 0 && hardware.rightDrive.getPower() != 0;
    }

    // Returns true if the robot is turning based on the motor reading (if left power and right power are on different sides of zero)
    public boolean isTurning() {
        return ((hardware.leftDrive.getPower() > 0 && hardware.rightDrive.getPower() < 0) || (hardware.leftDrive.getPower() < 0 && hardware.rightDrive.getPower() > 0));
    }

    // Updates the angle of the robot based on the gyro sensor reading
    private void updateAngle() {
        angle = (prevAngle + gyroReading()) % 360;
    }

    // Performs all of the robots moving functions (turning, forward) using the gamepad and updates pos and angle
    public void move() {
        useEncoder(hardware.leftDrive);
        useEncoder(hardware.rightDrive);

        if (!rightJoystickPressed()) {
            // Moving forwards
            if (gamepad.left_stick_y < 0) {
                leftPow = -gamepad.left_stick_y;
                rightPow = -gamepad.left_stick_y;
            } else {
                leftPow = 0;
                rightPow = 0;
            }
        } else {
            // Turning
            leftPow = 0.2 * gamepad.right_stick_x;
            rightPow = -0.2 * gamepad.right_stick_x;
        }

        hardware.leftDrive.setPower(leftPow);
        hardware.rightDrive.setPower(rightPow);

        int avg = (hardware.leftDrive.getCurrentPosition() + hardware.rightDrive.getCurrentPosition()) / 2;
        distanceTraveled = (int) (linDistance(avg) * 10) / 10.0;

        updateAngle();
    }

    // Automatically moves a linear distance in inches (conversion from inches to encoder ticks completed in method)
    public void autoMove(int distance) {
        // Reset encoders
        resetEncoder(hardware.leftDrive);
        resetEncoder(hardware.rightDrive);

        // Set mode to RUN_TO_POSITION (forces the robot to move until it reaches a certain encoder position)
        hardware.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target position for motors to run to
        hardware.leftDrive.setTargetPosition(encoderTicks(distance));
        hardware.rightDrive.setTargetPosition(encoderTicks(distance));

        // Apply power to drive motors (positive if distance is positive, negative if distance is negative)
        hardware.leftDrive.setPower(0.3 * distance/Math.abs(distance));
        hardware.rightDrive.setPower(0.3 * distance/Math.abs(distance));

        while (hardware.leftDrive.isBusy() && hardware.rightDrive.isBusy() && !gamepad.left_bumper) {
            // Wait - DRIVE MOTORS ARE BUSY
        }

        // Reset power
        hardware.leftDrive.setPower(0);
        hardware.rightDrive.setPower(0);
    }

    // Automatically turns the robot a certain POSITIVE target angle, TODO: implement a way for robot to turn a negative angle
    public void autoTurn(int targetAngle) {
        // Use encoders on the drive motors
        hardware.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double pow = 0.1; // Magnitude of power that will be applied to motors during turn
        int currAngle = 0; // Current change in angle since the start of the turn

        calibrateGyro(); // Calibrates gyro (sets gyro value back to zero but stores previous angle of robot in angle)

        // Robot turns until currAngle has exceeded targetAngle
        while (currAngle < targetAngle && !gamepad.left_bumper) {
            hardware.leftDrive.setPower(pow);
            hardware.rightDrive.setPower(-pow);

            currAngle = hardware.gyro.getHeading(); // Update currAngle
        }

        hardware.leftDrive.setPower(0);
        hardware.rightDrive.setPower(0);

        resetEncoder(hardware.leftDrive);
        resetEncoder(hardware.rightDrive);

        updateAngle(); // Update angle based on prevAngle and the assumption that targetAngle has been reached
    }

    private double dotProduct(Position pos1, Position pos2) {
        return pos1.getX() * pos2.getX() + pos1.getY() * pos2.getY();
    }

    // Returns the angle at which the robot should turn if it is to face a target position
    public int calcAngleToTurn(Position targetPos) {
        Position changeInPos = pos.changeInPosition(targetPos); // Vector the robot would have to travel if moving towards targetPos

        // If position is at origin, use tan
        if (pos.getX() == 0 && pos.getY() == 0) {
            double tan = changeInPos.getY() / changeInPos.getX(); // Tangent of angle between x-axis and changeInPos
            double angleFromOrigin = 180 * Math.atan(tan) / pi; // Angle in degrees between x-axis and changeInPos

            // Account for the fact that zero on the gyro sensor faces the y-axis, while our calculation is angle from the x-axis
            return 90 - (int) angleFromOrigin;
        }

        double productOfMagnitudes = pos.distFromOrigin() * changeInPos.distFromOrigin();
        double angleBetween = 180 * Math.acos(dotProduct(pos, changeInPos) / productOfMagnitudes) / pi;

        return (int) angleBetween;
    }

    // Move the robot to a specific coordinate; first, it turns to face desired coordinate, and then it moves a calculated position towards it
    public void moveToCoordinate(Position targetPos) {
        autoTurn(calcAngleToTurn(targetPos));
        autoMove((int) targetPos.distanceFrom(pos));
        pos = targetPos; // Updates pos to be target pos
    }

    // Robot Properties Accessor Methods
    public Position getPos() {
        return pos;
    }
    public int getAngle() {
        return angle;
    }
    public double angleInRadians() {
        return angle * pi / 180;
    }
    public double getDistance() {
        return distanceTraveled;
    }
    public int gyroReading() {
        return hardware.gyro.getHeading();
    }
}
