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
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
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