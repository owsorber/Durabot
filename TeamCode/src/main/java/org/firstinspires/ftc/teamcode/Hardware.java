package org.firstinspires.ftc.teamcode;

/**
 * The Hardware class for the Durabot. Includes 2 drive motors and a gyro sensor.
 *
 * @author Ben Gerrard
 * @date May 24th, 2018
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    //Instance data: motors and gyro
    DcMotor rightDrive;
    DcMotor leftDrive;
    DcMotor forwardBackwardEncoder;
    GyroSensor gyro;

    HardwareMap hwMap; // HardwareMap is how the phone tells Java what device is in what physical port on the REV Module

    //Constructor function instantiates all instance data
    public Hardware(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        /*
            Each motor and sensor object is instantiated with the same format below. get() is a method of the HardwareMap class that tells the phone that we are looking for a motor/gyro
            with that name in the configuration.
        */
        rightDrive = hwMap.get(DcMotor.class, "RD");
        leftDrive = hwMap.get(DcMotor.class, "LD");
        forwardBackwardEncoder = hwMap.get(DcMotor.class, "Efb");
        gyro = hwMap.get(GyroSensor.class, "GS");
    }

    //Sets direction of all the hardware in init() of TeleOp
    public void init() {
        // Set the direction of motors and encoders so that their in the same direction
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        forwardBackwardEncoder.setDirection(DcMotor.Direction.REVERSE);

        // Initially set the power of the motors to zero
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

}
