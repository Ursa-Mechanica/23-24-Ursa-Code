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
    
    public int intakeToggle = 0;
    
    boolean grabberOpen = false;
    boolean grabberPressed = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        
        waitForStart();
        runtime.reset();

        double originPoint = 0;

        while (opModeIsActive()) {
            
            if (robot.limit.isPressed()) originPoint = robot.m1.getCurrentPosition();
            
            telemetry.addData("Origin", originPoint);
            telemetry.addData("Current", robot.m1.getCurrentPosition());
            
            double rx = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.right_stick_x;
            
            float speedScalar;
            
            if (gamepad1.left_bumper) speedScalar = 0.15f;
            else if (gamepad1.left_trigger > 0.1f) speedScalar = 0.4f;
            else if (gamepad1.right_trigger > 0.1f) speedScalar = 1.0f;
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
            // if (robot.dist.getDistance(DistanceUnit.CM) < 25 && !gamepad2.left_bumper) armSpeed = Math.min(0.15, armSpeed);
            
            robot.m1.setPower(armSpeed);
            
            robot.s1.setPosition(grabberOpen ? 0.19 : 0);
            robot.s2.setPosition((intakeToggle / 2.0f) + 0.5f);
            
            if (gamepad1.x && gamepad2.x) robot.droneLauncher.setPosition(1);
            
            if (gamepad2.a || gamepad1.a) robot.droneLauncher.setPosition(0);
            
            // Toggle Buttons
            
            if (gamepad2.b && grabberPressed == false)
            {
                grabberOpen = !grabberOpen;
                grabberPressed = true;
                debounce.reset();
            } else if (!gamepad2.b) grabberPressed = false;
            
            if (gamepad2.dpad_up && debounce.milliseconds() > 250)
            {
                intakeToggle = (intakeToggle != 0) ? 0 : 1;
                debounce.reset();
            }
            
            if (gamepad2.dpad_down && debounce.milliseconds() > 250)
            {
                intakeToggle = (intakeToggle != 0) ? 0 : -1;
                debounce.reset();
            }
            
            telemetry.addData("Grabber", grabberOpen);
            telemetry.addData("intakeToggle", intakeToggle);
            // telemetry.addData("Distance", "%.3f, %.3f", robot.dist.getDistance(DistanceUnit.CM), armSpeed);
            telemetry.addData("Limit", robot.limit.isPressed());
            telemetry.update();
        }
    }
    
    double lerp(double a, double b, double f)
    {
        return a * (1.0 - f) + (b * f);
    }
}
