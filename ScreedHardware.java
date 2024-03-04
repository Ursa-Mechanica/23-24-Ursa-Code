package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ScreedHardware {


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    
    public DcMotor m1, intake = null;
    public DcMotorEx winch, lifter = null;
    public Servo s1, s2, wrist, spinner, droneLauncher = null;
    public CRServo cr1 = null;
    
    public DistanceSensor dist = null;
    public TouchSensor limit = null;
    
    public DistanceSensor distLeft, distRight = null;
    
    HardwareMap hardwareMap = null;
    BNO055IMU imu = null;
    
    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        leftFront = hardwareMap.dcMotor.get("front_left");
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftBack = hardwareMap.dcMotor.get("back_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        
        m1 = hardwareMap.dcMotor.get("m1");
        winch = hardwareMap.get(DcMotorEx.class, "winch");
        lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        wrist = hardwareMap.servo.get("wrist");
        spinner = hardwareMap.servo.get("spinner");
        intake = hardwareMap.dcMotor.get("intake");
        droneLauncher = hardwareMap.servo.get("s3");
        
        dist = hardwareMap.get(DistanceSensor.class, "dist");
        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        limit = hardwareMap.get(TouchSensor.class, "limit");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        m1.setDirection(DcMotor.Direction.REVERSE);
        
        droneLauncher.setPosition(0);
        
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    void zeroArm(boolean opModeIsActive)
    {
        m1.setPower(0.15); // lower arm
        
        while (opModeIsActive && !limit.isPressed()) {
            // sleep(10);
        }
        
        m1.setPower(0);
    }
}
