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
 * This file illustrates the concept of driving a path based on Encoders.
 * It uses the MaristBaseRobot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 */

@Autonomous(name="MaristBot: Auto Drive By Encoder", group="MaristBot")
//@Disabled
public class AutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot        robot   = new MaristBaseRobot();   // Use a MaristBaseRobot's hardware
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

        // Step 1:  Drive forward for 12 Inches
        robot.driveStraightInches(0.6, 12, 2);
        delay(0.5);

        // Step 2:  Turn Right 90 Degrees (270 Wheel Degrees)
        robot.pointTurnDegrees(0.5, 270, 2);
        delay(0.5);

        // Step 3:  Drive Backwards for 12 Inces
        robot.driveStraightInches(0.6, -12, 2);
        delay(0.5);

        // Step 4:  Turn Arm Motor 180 Degrees Forward
        robot.armMotorDeg(0.5, 180, 1);
        delay(1.0);

        // Step 6: Turn Lift Motor 180 Degrees Forward
        robot.liftMotorDeg(0.5, 180, 1);
        delay(0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // Old Time Based Functions for Autonomous Based on FTC Labs with Arduino

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
