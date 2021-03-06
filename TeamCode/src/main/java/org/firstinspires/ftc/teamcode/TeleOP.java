package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOP MechGarb")
public class TeleOP extends LinearOpMode
{
    // Declare drive motors
    private DcMotor motorLeft;
    private DcMotor motorRight;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        motorLeft.setDirection(DcMotor.Direction.REVERSE);


        // Wait until start button is pressed
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        while(opModeIsActive())
        {
            // Tank drive
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

        }

        // Give hardware a chance to catch up
        idle();
    }
}


