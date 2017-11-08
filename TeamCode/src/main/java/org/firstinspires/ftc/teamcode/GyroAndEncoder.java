package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Created by anjanbharadwaj on 11/7/2017
 */
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Gyro and Encoderx", group="Autonomous")

public class GyroAndEncoder extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    //static final double COUNTS_PER_MOTOR_REV = 1440;        // eg: TETRIX Motor Encoder
    //static final double DRIVE_GEAR_REDUCTION = 1.0;          // This is < 1.0 if geared UP
    //static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    double degrees = 90;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    GyroSensor gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        gyro = hardwareMap.gyroSensor.get("sensor_gyro");

        gyro.calibrate();

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gyro Direction", "Gyro angle: " + gyro.getHeading());
            telemetry.update();
            turnTo(degrees);

            if(degrees > 360){
                degrees = 15;
            }

            degrees += 15;
        }
    }
    public void turnTo(double degrees){
        int turnBy = -1;                 //turns clockwise

        if(degrees < gyro.getHeading()){
            turnBy *= -1;
        }

        while(degrees != gyro.getHeading()){
            leftMotor.setPower(-turnBy);
            rightMotor.setPower(turnBy);
        }
    }
}