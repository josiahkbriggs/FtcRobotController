package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.io.StringWriter;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
(name="AutoFarBlue", preselectTeleOp="Linear Opmode")
public class AutoSmallBlue extends LinearOpMode {
   
   // Tells the code what an hardware object is
    private HuskyLens huskyLens;
    private DcMotorEx LFront;
    private DcMotorEx LBack;
    private DcMotorEx RFront;  
    private DcMotorEx RBack;
    private DcMotorEx Intake;
    private DcMotorEx FlyWheel;
    private Servo PaddleLeft;
    private Servo PaddleRight;
    private CRServo ConveyorLeft;
    private CRServo ConveyorRight;

    double flDFW;
    double blDFW;
    double frDFW;
    double brDFW;
    double F = 18.6;
    double P = 30.2;
   
    private IMU imu;
   
    YawPitchRollAngles robotOrientation;
   
    int done = 0;
    public void runOpMode() {
        // Initialization Stage
        {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        LFront = hardwareMap.get(DcMotorEx.class, "LFront");
        LBack = hardwareMap.get(DcMotorEx.class, "LBack");
        RFront = hardwareMap.get(DcMotorEx.class, "RFront");
        RBack = hardwareMap.get(DcMotorEx.class, "RBack");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        FlyWheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
        PaddleLeft = hardwareMap.servo.get("PaddleLeft");
        PaddleRight = hardwareMap.servo.get("PaddleRight");
        ConveyorLeft = hardwareMap.crservo.get("ConveyorLeft");
        ConveyorRight = hardwareMap.crservo.get("ConveyorRight");
        imu = hardwareMap.get(IMU.class, "imu");
       
            IMU.Parameters myIMUparameters;
            myIMUparameters = new IMU.Parameters
            (new RevHubOrientationOnRobot
            (RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
            
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // Sets Direction of the Motors Default State
            LFront.setDirection(DcMotor.Direction.FORWARD);
            RFront.setDirection(DcMotor.Direction.REVERSE);
            LBack.setDirection(DcMotor.Direction.FORWARD);
            RBack.setDirection(DcMotor.Direction.REVERSE);
            FlyWheel.setDirection(DcMotor.Direction.REVERSE);

            // Forces the Wheels to "Brake" when there is no power being applied
            LFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            RFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            LBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            RBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            FlyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            FlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            telemetry.addLine("Init Complete");
           
            LFront.setTargetPositionTolerance(18);
            RFront.setTargetPositionTolerance(18);
            LBack.setTargetPositionTolerance(18);
            RBack.setTargetPositionTolerance(18);

            // Lets the Driver know that the bot is ready to be started
            telemetry.addLine("Done");
            telemetry.update();
        }
   
        waitForStart();


        while (opModeIsActive()) {
           
            if (done == 0) {
                preformSequence();
                
                done = 1;
                sleep(1000);
            } else {
               
                idle();
               
            }
        }
    }
   
    public double getIMUHeading() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }
   
    void drive(int distance, double speed) {
            LFront.setTargetPosition(-distance);
            RFront.setTargetPosition(-distance);
            LBack.setTargetPosition(-distance);
            RBack.setTargetPosition(-distance);
           
            LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            LFront.setVelocity(speed);
            RFront.setVelocity(speed);
            LBack.setVelocity(speed);
            RBack.setVelocity(speed);
           
            while (LFront.isBusy() || RFront.isBusy() || LBack.isBusy() || RBack.isBusy()){
               sleep(1);
           }
           
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
    }
   
    void fortyfivestrafe(int distance, int distanceTwo, double speed) { //distance=LeftBack;
            LFront.setTargetPosition(distance);                         //-distance=RightFront
            RFront.setTargetPosition(distanceTwo);                      //distanceTwo=RightBack
            LBack.setTargetPosition(distanceTwo);                       //-distanceTwo=LeftFront
            RBack.setTargetPosition(distance);
           
            LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            LFront.setVelocity(speed * 1.1);
            RFront.setVelocity(speed);
            LBack.setVelocity(speed);
            RBack.setVelocity(speed * 1.1);
           
            while (LFront.isBusy() || RFront.isBusy() || LBack.isBusy() || RBack.isBusy()){
               sleep(1);
           }
           
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
       
    }
   
    void strafe(int distance, double speed) {
            LFront.setTargetPosition(-distance);
            RFront.setTargetPosition(distance);
            LBack.setTargetPosition(distance);
            RBack.setTargetPosition(-distance);
           
            LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            LFront.setVelocity(speed * 0.9);
            RFront.setVelocity(speed * 0.9);
            LBack.setVelocity(speed);
            RBack.setVelocity(speed);
           
            while (LFront.isBusy() || RFront.isBusy() || LBack.isBusy() || RBack.isBusy()){
               sleep(1);
           }
           
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
    }
    
    void turn(int distance, double speed) {
            LFront.setTargetPosition(distance);
            RFront.setTargetPosition(-distance);
            LBack.setTargetPosition(distance);
            RBack.setTargetPosition(-distance);
           
            LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            LFront.setVelocity(speed);//*0.9
            RFront.setVelocity(speed);
            LBack.setVelocity(speed);
            RBack.setVelocity(speed);
           
            while (LFront.isBusy() || RFront.isBusy() || LBack.isBusy() || RBack.isBusy()){
               sleep(1);
           }
           
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
    }
    
    void veloDrive(double speed) {
        LFront.setVelocity(speed);
        RFront.setVelocity(speed);
        LBack.setVelocity(speed);
        RBack.setVelocity(speed);
    }
   
    void veloTurn(double speed) {
        LFront.setVelocity(-speed);
        RFront.setVelocity(speed);
        LBack.setVelocity(speed);
        RBack.setVelocity(-speed);
    }
   
    void veloStrafe(double speed) {
        LFront.setVelocity(-speed);
        RFront.setVelocity(speed);
        LBack.setVelocity(-speed);
        RBack.setVelocity(speed);
    }

    void stopAndReset() {
            LFront.setVelocity(0);
            RFront.setVelocity(0);
            LBack.setVelocity(0);
            RBack.setVelocity(0);
           
            sleep(500);
           
            LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
    }
   
    void FlyWheelPowerUp() {
        FlyWheel.setVelocity(1570);
    }
    
    void FlyWheelPowerUp2() {
        FlyWheel.setVelocity(1672);
    }
    
    void FlyWheelPowerDown() {
        FlyWheel.setVelocity(0);
    }
    
    void FlyWheelReverse() {
        FlyWheel.setVelocity(-1000);
    }
        
    void ConveyorStart() {
        ConveyorLeft.setPower(-0.70);
        ConveyorRight.setPower(0.70);
    }
    
    void ConveyorStop() {
        ConveyorLeft.setPower(0);
        ConveyorRight.setPower(0);
    }
    
    void ConveyorReverse() {
        ConveyorLeft.setPower(0.45);
        ConveyorRight.setPower(-0.45);
    }

    void IntakeReverse() {
        Intake.setPower(1);
        FlyWheel.setVelocity(-250);
    }
    
    void IntakeStart() {
        Intake.setPower(1);
    }
    
    void IntakeStop() {
        Intake.setPower(0);
        FlyWheel.setVelocity(0);
    }
    
    void PaddleForward() {
        PaddleLeft.setPosition(0);
        PaddleRight.setPosition(0.645);
    }
    
    void PaddleBack() {
        PaddleLeft.setPosition(0.65);
        PaddleRight.setPosition(0);
    }
    
    void Reverse() {
        ConveyorReverse();
        FlyWheelReverse();
        sleep(750);
        ConveyorStop();
        FlyWheelPowerDown();
    }

    void Launch3() {
        IntakeStart();
        ConveyorStart();
        sleep(500);
        ConveyorStop();
        FlyWheelPowerUp2();
        sleep(500);
        ConveyorStart();
        sleep(500);
        ConveyorStop();
        FlyWheelPowerUp2();
        sleep(1000);
        ConveyorStart();
        sleep(1500);
        PaddleForward();
        sleep(400);
        PaddleBack();
        ConveyorStop();
    }
    
    void PickUp3() {
        FlyWheelPowerDown();
        ConveyorStart();
        drive(1550, 700);
        ConveyorStop();
    }
   
    /*void armDrive3(int distance, int distance2, double speed) {
           
            GB.setPosition(0.35);
           
            FL.setTargetPosition(distance);
            FR.setTargetPosition(distance);
            BL.setTargetPosition(distance);
            BR.setTargetPosition(distance);
           
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
            FL.setVelocity(speed);
            FR.setVelocity(speed);
            BL.setVelocity(speed);
            BR.setVelocity(speed);
           
            SkibidiLeft.setTargetPosition(distance2);
            SkibidiRight.setTargetPosition(distance2);
           
            SkibidiLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SkibidiRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             
            SkibidiLeft.setVelocity(2400);
            SkibidiRight.setVelocity(2400);
           
            while (SkibidiLeft.isBusy() || SkibidiRight.isBusy() || FL.isBusy()) {
                sleep(1);
            }
           
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1);
    }*/
   
    void preformSequence() {
        FlyWheelPowerUp();
        drive(100, 4000);
        turn(225, 4000);
        sleep(1000);
        IntakeStart();
        sleep(250);
        Launch3();
        /*drive(1052, 4000);
        turn(665, 4000);
        sleep(1);
        PickUp3();
        turn(-500, 4000);
        fortyfivestrafe(0, 2250, 4000);
        turn(-272, 4000);
        Reverse();
        drive(-975, 3000);
        FlyWheelPowerUp();
        sleep(1000);
        Launch3();*/
        turn(-225, 4000);
        strafe(-900, 4000);
        FlyWheelPowerDown();
        IntakeStop();
        }
}
