package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    
    public int intakeToggle, spinPos = 0;
    public double wristPos = 0.5;
    public double[] spinPositions = new double[]{0.15, 0.3725, 0.53};
    public double armPos = 0;
    
    public boolean isArmUp = false;
    
    double a = 0;
    double b = -1883.05;
    
    boolean spinPressed = false;
    boolean grabberOpen = false;
    boolean grabberPressed = false;
    boolean enableLifting = false;
    boolean liftToggle = false;
    boolean launcherToggle = false;
    boolean autoLevelMode = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        a = (2.1071f * Math.pow(10, -8));
        
        waitForStart();
        runtime.reset();

        double originPoint = robot.m1.getCurrentPosition();
        
        wristPos = 0.53;

        while (opModeIsActive()) {
            
            if (robot.limit.isPressed()) originPoint = robot.m1.getCurrentPosition();
            
            armPos = robot.m1.getCurrentPosition();
            
            telemetry.addData("Origin", originPoint);
            telemetry.addData("Current", originPoint - robot.m1.getCurrentPosition());
            telemetry.addData("Wrist", robot.wrist.getPosition());
            telemetry.addData("Spinner", robot.spinner.getPosition());
            
            // ----- DRIVE CODE -----
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
            
            // -----  -----
            
            double a = 0.004;
            double b = 0.0;
            
            double armSpeed = (gamepad2.left_trigger - gamepad2.right_trigger);
            
            if (robot.limit.isPressed()) armSpeed = Math.min(0, armSpeed);
            
            // if (robot.dist.getDistance(DistanceUnit.CM) < 8) armSpeed = Math.max(-0.2, armSpeed);
            if (robot.dist.getDistance(DistanceUnit.CM) < 25 && !gamepad2.left_bumper) armSpeed = Math.min(0.15, armSpeed);
            
            robot.m1.setPower(armSpeed);
            
            robot.s1.setPosition(grabberOpen ? 0.4 : 0);
            // robot.s1.setPosition(grabberOpen ? 0.19 : 0);
            // robot.s1.setPosition(grabberOpen ? .125 : .2);
            robot.intake.setPower(intakeToggle);
            
            if (armPos != robot.m1.getCurrentPosition())
            {
                armPos = robot.m1.getCurrentPosition();
                wristPos = calcWrist(originPoint - robot.m1.getCurrentPosition());
            }
            
            wristPos += gamepad2.right_stick_y * 0.025;
            robot.wrist.setPosition(wristPos);
            
            if (gamepad2.right_stick_x < -0.5 && !spinPressed) { spinPos++; spinPressed = true; }
            if (gamepad2.right_stick_x > 0.5 && !spinPressed) { spinPos--; spinPressed = true; }
            if (gamepad2.right_stick_x > -0.25 && gamepad2.right_stick_x < 0.25) spinPressed = false;
            
            if (spinPos >= spinPositions.length) spinPos = spinPositions.length - 1;
            if (spinPos <= 0) spinPos = 0;
            
            if ((originPoint - robot.m1.getCurrentPosition()) <= 2600) {
                spinPos = 0;
                isArmUp = false;
            }
            
            if ((originPoint - robot.m1.getCurrentPosition()) >= 2600 && !isArmUp) {
                spinPos = spinPositions.length - 1;
                isArmUp = true;
            }
            
            robot.spinner.setPosition(spinPositions[spinPos]);

            // ----- Drone -----
            if (gamepad1.x && gamepad2.x) robot.droneLauncher.setPosition(1);
            else robot.droneLauncher.setPosition(0);
            
            if (launcherToggle) robot.droneAngle.setPosition(0.15);
            else robot.droneAngle.setPosition(0.3);
            
            // ----- Lifting mechanism -----
            if (enableLifting) {
                if (gamepad1.dpad_up) // go up
                {
                    robot.lifter.setPower(0.6);
                    robot.winch.setPower(-1);
                } else if (gamepad1.dpad_down) {
                    robot.lifter.setPower(0.05);
                    robot.winch.setPower(1);
                } else {
                    robot.lifter.setPower(0.1);
                    robot.winch.setPower(0.01);
                }
            } else {
                robot.lifter.setPower(0);
                robot.winch.setPower(0);
            }

            // Auto Level Mode
            if (gamepad1.left_stick_button && debounce.milliseconds() > 250) {
                debounce.reset();
                autoLevelMode = true;
            }

            while(autoLevelMode && opModeIsActive()) {
                autoLevel(20);

                if (gamepad1.left_stick_button && debounce.milliseconds() > 250) {
                    debounce.reset();
                    autoLevelMode = false;;
                }
            }
            
            // Toggle Buttons
            
            if (gamepad2.a && gamepad1.a && debounce.milliseconds() > 250) {
                debounce.reset();
                launcherToggle = !launcherToggle;
            }
            
            if (wristPos > 1) wristPos = 1;
            if (wristPos < 0) wristPos = 0;
            
            if (gamepad1.y && !liftToggle)
            {
                enableLifting = !enableLifting;
                liftToggle = true;
            } else if (!gamepad1.y) liftToggle = false;
            
            if (gamepad2.b && !grabberPressed)
            {
                grabberOpen = !grabberOpen;
                grabberPressed = true;
                debounce.reset();
            } else if (!gamepad2.b) grabberPressed = false;
            
            if (gamepad2.dpad_up && debounce.milliseconds() > 250)
            {
                intakeToggle = (intakeToggle != 0) ? 0 : -1;
                debounce.reset();
            }
            
            if (gamepad2.dpad_down && debounce.milliseconds() > 250)
            {
                intakeToggle = (intakeToggle != 0) ? 0 : 1;
                debounce.reset();
            }
            
            telemetry.addData("Grabber", grabberOpen);
            telemetry.addData("intakeToggle", intakeToggle);
            telemetry.addData("Distance", "%.3f, %.3f", robot.dist.getDistance(DistanceUnit.CM), armSpeed);
            telemetry.addData("Limit", robot.limit.isPressed());
            telemetry.update();
        }
    }
    
    double calcWrist(double x)
    {
        double y = a * (x + b) * (x + b) + 0.407569f;
        
        if (x <= 1700) y = 0.4;
        if (x >= 500 && x <= 1700) y = 0.44;
        
        return y;
    }

    private void autoLevel(double targetDist)
    {
        if (robot.distLeft.getDistance(DistanceUnit.CM) > targetDist + error)
        {
            robot.leftFront.setPower(-0.25);
            robot.leftBack.setPower(-0.25);
        } else if (robot.distLeft.getDistance(DistanceUnit.CM) < targetDist - error)
        {
            robot.leftFront.setPower(0.25);
            robot.leftBack.setPower(0.25);
        } else
        {
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
        }
        if (robot.distRight.getDistance(DistanceUnit.CM) > targetDist + error)
        {
            robot.rightFront.setPower(-0.25);
            robot.rightBack.setPower(-0.25);
        } else if (robot.distRight.getDistance(DistanceUnit.CM) < targetDist - error)
        {
            robot.rightFront.setPower(0.25);
            robot.rightBack.setPower(0.25);
        } else
        {
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
        }
    }
    
    double lerp(double a, double b, double f)
    {
        return a * (1.0 - f) + (b * f);
    }
}
