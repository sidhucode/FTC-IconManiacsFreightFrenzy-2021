/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class IconManiacsHardwareBot {
    /* Public OpMode members. */

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor armLeft = null;
    public DcMotor armRight = null;
    public DcMotor intake = null;
    public DcMotor duckMotor = null;


    // constants and variables to be used when running the code (specifically servos)

    HardwareMap hwMap = null;

    /* Constructor */
    public IconManiacsHardwareBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors\

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        duckMotor.setPower(0);
        armLeft.setPower(0);
        armRight.setPower(0);


        // sets zeroPowerBehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets motor directions
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        armRight = hwMap.get(DcMotor.class, "armRight");
        armLeft = hwMap.get(DcMotor.class, "armLeft");
        intake = hwMap.get(DcMotor.class, "intake");
        duckMotor = hwMap.get(DcMotor.class, "duckMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        duckMotor.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void powerOff() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    public void move(double power, double distance, String direction) { //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance / circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int) (rotationsRequired * 537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction.toLowerCase();
        if (direction.equals("forward")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        } else if (direction.equals("backward")) {
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        }
        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()) {

        }

        powerOff();

    }

    public void turn(double power, double distance, String direction) { //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance / circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int) (rotationsRequired * 537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction = direction.toLowerCase();
        if (direction.equals("right")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        } else if (direction.equals("left")) {
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        }

        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()) {
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void strafe(double power, double distance, String direction) { //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance / circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int) (rotationsRequired * 537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        if (direction.equals("right")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        } else if (direction.equals("left")) {
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        }
        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()) {
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void moveArm(double power, double distance, String direction, String sideMotor) { //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 3.937; //pi*diameter
        double rotationsRequired = distance / circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int) (rotationsRequired * 537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction = direction.toLowerCase();
        sideMotor = sideMotor.toLowerCase();
        if (sideMotor.equals("right")) {
            if (direction.equals("up")) {
                armRight.setTargetPosition(encoderTargetPOS);
            } else if (direction.equals("down")) {
                armRight.setTargetPosition(-encoderTargetPOS);
            }
        } else if (sideMotor.equals("left")) {
            if (direction.equals("up")) {
                armLeft.setTargetPosition(encoderTargetPOS);
            } else if (direction.equals("down")) {
                armLeft.setTargetPosition(-encoderTargetPOS);
            }
        }
        // sets the motors to the respective power that was input
        armRight.setPower(power);
        armLeft.setPower(power);
        //the mode is set to run to the target position that was set above
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armRight.isBusy() || armLeft.isBusy()) {
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();

        }
        if (sideMotor.equals("armRight")) {
            armRight.setPower(0);
        } else if (sideMotor.equals("armLeft")) {
            armLeft.setPower(0);
        }

    }

    public void intake(double power, double distance) throws InterruptedException {
        intake.setPower(power);
        sleep((long) distance);
        intake.setPower(0);
    }

}


