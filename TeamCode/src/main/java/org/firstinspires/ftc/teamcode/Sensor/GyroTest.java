package org.firstinspires.ftc.teamcode.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sreehari Ram Mohan on 11/07/17.
 */
@Autonomous(name= "Gyro Test", group = "Autonomous")
public class GyroTest extends LinearOpMode {

    ModernRoboticsI2cGyro gyro        = null;
    DcMotor leftMotor   = null;
    DcMotor rightMotor  = null;

    private ElapsedTime runtime = new ElapsedTime();

    double degrees = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
        leftMotor = (DcMotor) hardwareMap.dcMotor.get("left_drive");
        rightMotor = (DcMotor) hardwareMap.dcMotor.get("right_drive");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.calibrate();

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
