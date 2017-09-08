package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Michelle on 9/8/2017.
 */

@Autonomous(name="Michelle Autonomous", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class Michelle_Autonomous extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private ColorSensor colorSensor = null;

    /*
         * Code to run ONCE when the driver hits INIT
         */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("name_of_left_motor"); //we would configure this in FTC Robot Controller app
        rightMotor = hardwareMap.dcMotor.get("name_of_right_motor");

        colorSensor = hardwareMap.colorSensor.get("name_of_color_sensor"); //we would configure the name of the color sensor later in the
        //ftc robot controller


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        //this is a way to print to the screen of the iphone app, useful for debugging.
        telemetry.addData("Robot starting", "");
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());



//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i = 0; i < 4; i++) {
            leftMotor.setPower(1);
            rightMotor.setPower(1);

            leftMotor.setTargetPosition(10);
            rightMotor.setTargetPosition(10);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightMotor.setPower(-1);

            leftMotor.setTargetPosition(1);
            rightMotor.setTargetPosition(1);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        }






    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
