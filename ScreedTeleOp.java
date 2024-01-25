package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Screed: TeleOp", group="Screed")

public class ScreedTeleOp extends LinearOpMode {

    ScreedHardware robot = new ScreedHardware();
    
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime debounce = new ElapsedTime();
    
    boolean grabberOpen, intakeToggle = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            
            double rx = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.right_stick_x;
            
            float speedScalar;
            
            if (gamepad1.left_bumper) speedScalar = 0.1f;
            else if (gamepad1.left_trigger > 0.1) speedScalar = 0.4f;
            else if (gamepad1.right_bumper) speedScalar = 1.0f;
            else speedScalar = 0.75f;

            robot.leftFront.setPower((y - x + rx) * speedScalar);
            robot.rightFront.setPower((y - x - rx) * speedScalar);
            robot.leftBack.setPower((y + x + rx) * speedScalar);
            robot.rightBack.setPower((y + x - rx) * speedScalar);
            
            double a = 0.004;
            double b = 0.0;
            
            double armSpeed = (gamepad2.left_trigger - gamepad2.right_trigger);
            
            if (robot.limit.isPressed()) armSpeed = Math.min(0, armSpeed);
            
            // if (robot.dist.getDistance(DistanceUnit.CM) < 8) armSpeed = Math.max(-0.2, armSpeed);
            if (robot.dist.getDistance(DistanceUnit.CM) < 25) armSpeed = Math.min(0.1, armSpeed);
            
            robot.m1.setPower(armSpeed);
            
            robot.s1.setPosition(grabberOpen ? 0.18 : 0);
            robot.s2.setPosition(intakeToggle ? 1 : 0.5);
            
            // Toggle Buttons
            
            if (gamepad2.b && debounce.milliseconds() > 500)
            {
                grabberOpen = !grabberOpen;
                debounce.reset();
            }
            
            if (gamepad2.dpad_up && debounce.milliseconds() > 500)
            {
                intakeToggle = !intakeToggle;
                debounce.reset();
            }
            
            telemetry.addData("Grabber", grabberOpen);
            telemetry.addData("Distance", "%.3f, %.3f", robot.dist.getDistance(DistanceUnit.CM), armSpeed);
            telemetry.addData("Limit", robot.limit.isPressed());
            telemetry.update();
        }
    }
    
    double lerp(double a, double b, double f)
    {
        return a * (1.0 - f) + (b * f);
    }
}
