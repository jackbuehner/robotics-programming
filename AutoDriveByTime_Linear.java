/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 */

@Autonomous(name="MaristBot: Auto Drive By Time", group="MaristBot")
//@Disabled
public class AutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot        robot   = new MaristBaseRobot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        driveStraight(0.6, 3.0);
        delay(0.5);

        // Step 2:  Spin right for 1.3 seconds
        pointTurn(0.6, 1.3);
        delay(0.5);

        // Step 3:  Drive Backwards for 1 Second
        driveStraight(-0.6, 1.0);
        delay(0.5);

        // Step 4:  Stop and close the claw.
        openHands();
        delay(1.0);
        closeHands();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // Functions for Autonomous Based on FTC Labs with Arduino

    public void driveStraight(double p, double t) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
        delay(t);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void driveStraight(double p) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
    }

    public void pointTurn(double p, double t) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(-p);
        delay(t);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void pointTurn(double p) {
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(-p);
    }

    public void openHands() {  // You will need to modify this
        robot.leftClaw.setPosition(1.0);
        robot.rightClaw.setPosition(1.0);
    }

    public void closeHands() { // You will need to modify this
        robot.leftClaw.setPosition(0.0);
        robot.rightClaw.setPosition(0.0);
    }

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void moveArm(double p, double t) {
        robot.armMotor.setPower(p);
        delay(t);
        robot.armMotor.setPower(0);
    }

    // Add more functions here if needed

}
