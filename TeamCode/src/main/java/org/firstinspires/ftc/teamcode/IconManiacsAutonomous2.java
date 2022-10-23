
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
//@Disabled
public class IconManiacsAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    IconManiacsHardwareBot bot   = new IconManiacsHardwareBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    /*static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;*/

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        bot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Moving Forward
        bot.move(1,5,"forward");

        //Moving Backward
        bot.move(1,5,"backward");

        //Turning Left
        bot.turn(1,5,"left");

        //Turning Right
        bot.turn(1,5,"right");

        //Strafing Right
        bot.strafe(1,20,"right");

        //Strafing Left
        bot.strafe(1,5,"left");
        
        //Duck Motor On
        bot.duckMotor.setPower(1);
        
        //Duck Motor Off
        bot.duckMotor.setPower(0);

        //Move Arm to Right
        bot.moveArm(0.5,5,"down","right");
        bot.moveArm(0.5,5,"up","right");

        //Move Arm to Left
        bot.moveArm(0.5,20,"down","left");
        bot.moveArm(0.5,20,"up","left");
        
        
        //Autonomous Code: need to fix values
        
        //from shipping hub to duck carsouel
        bot.turn(1,15,"right");
        bot.move(1,25,"forward");
        bot.turn(1,15,"right");
        bot.strafe(1,15,"left");
        bot.duckMotor.setPower(0.2);

        //from duck carosel to

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
