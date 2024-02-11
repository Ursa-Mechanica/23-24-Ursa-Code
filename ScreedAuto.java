package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ScreedVisionProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name="Screed: Auto", group="Screed")
public class ScreedAuto extends LinearOpMode
{
    ScreedHardware robot = new ScreedHardware();
    private ElapsedTime runtime = new ElapsedTime();

    VisionPortal vision;
    ScreedVisionProcessor screedProcessor = new ScreedVisionProcessor();
    
    static final double COUNTS_PER_INCH = 58.639;
    Orientation referenceAngle = new Orientation();
    double globalAngle;
    
    @Override
    public void runOpMode()
    {
        WebcamName web = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        vision = VisionPortal.easyCreateWithDefaults(web, screedProcessor);
        vision.setProcessorEnabled(screedProcessor, false);
        
        // vision.setProcessorEnabled(screedProcessor, true);
        
        boolean isBlue = false;
        
        initMotors();
        
        // select colors
        while (!opModeIsActive() || !robot.imu.isGyroCalibrated()) {
            if (gamepad1.x) isBlue = true;
            else if (gamepad1.b) isBlue = false;
            
            idle();
            
            telemetry.addData("Color", isBlue ? "Blue" : "Red");
            telemetry.addData("IMU Status", robot.imu.getCalibrationStatus().toString());
            telemetry.update();
        }
        
        screedProcessor.setColor(isBlue);
    
        waitForStart();
        vision.setProcessorEnabled(screedProcessor, true);
        
        robot.m1.setPower(0.15); // lower arm
        
        while (opModeIsActive() && !robot.limit.isPressed()) {
            sleep(10);
        }
        
        robot.m1.setPower(0);
        robot.s1.setPosition(0.19); // open grabber
        
        double origin = robot.m1.getCurrentPosition();
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Position: ", screedProcessor.getPosition());
            telemetry.update();
        }
        
        switch (screedProcessor.getPosition())
        {
            case -1: // left
                encoderDrive(0.6, 0.6, 6, 6, 6, 6, 5.0);
                encoderDrive(1, 0.4, 28, 28, 28, 28, 5.0);
                encoderDrive(1, 0.4, -28, -28, -28, -28, 5.0);
                encoderDrive(0.1, 0.1, -1, -1, -1, -1, 1.0);
                break;
            case 0: // middle
                encoderDrive(0.6, 0.6, 29, 29, 29, 29, 5.0);
                encoderDrive(0.6, 0.6, -24, -24, -24, -24, 5.0);
                break;
            case 1: // right
                encoderDrive(0.6, 0.6, 6, 6, 6, 6, 5.0);
                encoderDrive(0.4, 1, 30, 30, 30, 30, 5.0);
                encoderDrive(0.4, 1, -30, -30, -30, -30, 5.0);
                encoderDrive(0.1, 0.1, -1, -1, -1, -1, 1.0);
                break;
        }
        
        // align to wall
        autoLevel(20);
        
        rotate(90 * (isBlue ? -1 : 1), 0.5);
        
        encoderDrive(0.6, 0.6, -32, -32, -32, -32, 5.0);
        
        if (!isBlue) encoderDrive(0.6, 0.6, 18, -18, -18, 18, 5.0);
        else encoderDrive(0.6, 0.6, -18, 18, 18, -18, 5.0);
        
        autoLevel(5.0);
        
        robot.m1.setPower(-0.4);
        
        while (opModeIsActive() && robot.m1.getCurrentPosition() > origin - 4000);
        
        robot.m1.setPower(0);
        sleep(250);
        robot.s1.setPosition(0);
        sleep(250);
        
        encoderDrive(0.6, 0.6, 4, 4, 4, 4, 2.0);
        if (!isBlue) encoderDrive(0.6, 0.6, -18, 18, 18, -18, 5.0);
        else encoderDrive(0.6, 0.6, 18, -18, -18, 18, 5.0);
        
        sleep(250);
        
        vision.close();
    }
    
    public void encoderDrive(double speedL, double speedR,
                             double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speedL));
            robot.leftBack.setPower(Math.abs(speedL));
            robot.rightFront.setPower(Math.abs(speedR));
            robot.rightBack.setPower(Math.abs(speedR));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d %7d %7d %7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d %7d %7d %7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.leftBack.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            
            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
    
    private void rotate(double degrees, double power)
    {
        resetAngle();
        
        double leftPower = power;
        double rightPower = power;
        
        // left is positive
        while (opModeIsActive() && (getAngle() > degrees + 5 || getAngle() < degrees - 5))
        {
            telemetry.addData("Degrees", degrees);
            telemetry.addData("Heading", getAngle());
            telemetry.update();
            
            if (getAngle() < degrees)
            {
                robot.leftFront.setPower(power);
                robot.leftBack.setPower(power);
                robot.rightFront.setPower(-power);
                robot.rightBack.setPower(-power);
            } else {
                robot.leftFront.setPower(-power);
                robot.leftBack.setPower(-power);
                robot.rightFront.setPower(power);
                robot.rightBack.setPower(power);
            }
        }
            
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        
        resetAngle();
    }
    
    private void autoLevel(double targetDist)
    {
        double error = 2.0;
        
        boolean leftCheck = false, rightCheck = false;
        
        while (opModeIsActive() && (!leftCheck && !rightCheck))
        {
            leftCheck = (robot.distLeft.getDistance(DistanceUnit.CM) < targetDist + error/2) && (robot.distLeft.getDistance(DistanceUnit.CM) > targetDist - error/2);
            rightCheck = (robot.distRight.getDistance(DistanceUnit.CM) < targetDist + error/2) && (robot.distRight.getDistance(DistanceUnit.CM) > targetDist - error/2);
            
            telemetry.addData("Left Check", leftCheck);
            telemetry.addData("Right Check", rightCheck);
            telemetry.addData("Left", robot.distLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Right", robot.distRight.getDistance(DistanceUnit.CM));
            
            telemetry.update();
            
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
        
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        
        sleep(250);
    }
    
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - referenceAngle.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        referenceAngle = angles;
        
        if (globalAngle < -180)
            globalAngle += 360;
        else if (globalAngle > 180)
            globalAngle -= 360;

        return globalAngle;
    }
    
    private void resetAngle()
    {
        referenceAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    
    public void initMotors()
    {
        robot.init(hardwareMap);
        
        robot.leftFront.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotor.Direction.FORWARD);
        robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
