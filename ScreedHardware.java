package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    
    public DcMotor m1 = null;
    public Servo s1, s2 = null;
    public CRServo cr1 = null;
    
    public DistanceSensor dist = null;
    public TouchSensor limit = null;
    
    HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        leftFront = hardwareMap.dcMotor.get("front_left");
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftBack = hardwareMap.dcMotor.get("back_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        
        m1 = hardwareMap.dcMotor.get("m1");

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        
        dist = hardwareMap.get(DistanceSensor.class, "dist");
        limit = hardwareMap.get(TouchSensor.class, "limit");
        
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        m1.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
