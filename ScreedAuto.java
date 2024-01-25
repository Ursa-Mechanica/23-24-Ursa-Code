package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ScreedVisionProcessor;

@Autonomous(name="Screed: Auto", group="Screed")
public class ScreedAuto extends LinearOpMode
{
    ScreedHardware robot = new ScreedHardware();
    private ElapsedTime runtime = new ElapsedTime();

    VisionPortal vision;
    ScreedVisionProcessor screedProcessor = new ScreedVisionProcessor();
    
    static final double COUNTS_PER_INCH = 58.639;
    
    @Override
    public void runOpMode()
    {
        WebcamName web = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        vision = VisionPortal.easyCreateWithDefaults(web, screedProcessor);
        vision.setProcessorEnabled(screedProcessor, false);
        
        // vision.setProcessorEnabled(screedProcessor, true);
        
        screedProcessor.setColor(false);
        
        robot.init(hardwareMap);
        
        robot.leftFront.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotor.Direction.FORWARD);
        robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        waitForStart();
        vision.setProcessorEnabled(screedProcessor, true);
        
        while (opModeIsActive() && runtime.seconds() < 5) {
            telemetry.addData("Position: ", screedProcessor.getPosition());
            telemetry.update();
        }
        
        switch (screedProcessor.getPosition())
        {
            case -1: // left
                encoderDrive(1, 0.4, 28, 28, 28, 28, 5.0);
                break;
            case 0: // middle
                encoderDrive(0.6, 0.6, 30, 30, 30, 30, 5.0);
                break;
            case 1: // right
                encoderDrive(0.3, 1, 30, 30, 30, 30, 5.0);
                break;
        }
        
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
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d %7d %7d %7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d %7d %7d %7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.leftBack.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
