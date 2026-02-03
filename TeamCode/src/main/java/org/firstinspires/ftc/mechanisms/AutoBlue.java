package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
(name="Josiah's Success Blue", preselectTeleOp="Linear Opmode")
public class AutoBlue extends LinearOpMode {
   
   // Tells the code what an hardware object is
    private DcMotorEx LFront;
    private DcMotorEx LBack;
    private DcMotorEx RFront;  
    private DcMotorEx RBack;
    private DcMotorEx Intake;
    private DcMotorEx FlyWheel;
    private Servo Paddle;
    private IMU imu;

    double flDFW;
    double blDFW;
    double frDFW;
    double brDFW;
    double F = 28.6;
    double P = 40.2;
   
    YawPitchRollAngles robotOrientation;
   
    int done = 0;
    public void runOpMode() {
        
        // Initialization Stage
        {
        LFront = hardwareMap.get(DcMotorEx.class, "LFront");
        LBack = hardwareMap.get(DcMotorEx.class, "LBack");
        RFront = hardwareMap.get(DcMotorEx.class, "RFront");
        RBack = hardwareMap.get(DcMotorEx.class, "RBack");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        FlyWheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
        Paddle = hardwareMap.servo.get("Paddle");
        imu = hardwareMap.get(IMU.class, "imu");
       
            IMU.Parameters myIMUparameters;
            myIMUparameters = new IMU.Parameters
            (new RevHubOrientationOnRobot
            (RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            
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
            LFront.setDirection(DcMotor.Direction.REVERSE);
            RFront.setDirection(DcMotor.Direction.FORWARD);
            LBack.setDirection(DcMotor.Direction.REVERSE);
            RBack.setDirection(DcMotor.Direction.FORWARD);
            FlyWheel.setDirection(DcMotor.Direction.REVERSE);
            Intake.setDirection(DcMotor.Direction.REVERSE);

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
            LFront.setTargetPosition(distance);
            RFront.setTargetPosition(distance);
            LBack.setTargetPosition(distance);
            RBack.setTargetPosition(distance);
           
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
   
    void fortyfivestrafe(int distance, int distanceTwo, double speed) { //-distance=LeftBack;
            LFront.setTargetPosition(distance);                         //distance=RightFront
            RFront.setTargetPosition(distanceTwo);                      //-distanceTwo=RightBack
            LBack.setTargetPosition(distanceTwo);                       //distanceTwo=LeftFront
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
           
            LFront.setVelocity(speed * 1.1);
            RFront.setVelocity(speed * 1.1);
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
        FlyWheel.setVelocity(340);
    }
    
    void FlyWheelPowerUp2() {
        FlyWheel.setVelocity(350);
    }

    
    void FlyWheelPowerDown() {
        FlyWheel.setVelocity(0);
    }
    
    void FlyWheelReverse() {
        FlyWheel.setVelocity(-1000);
    }

    void IntakeReverse() {
        Intake.setPower(-1);
        FlyWheel.setVelocity(-1000);
    }
    
    void IntakeStart() {
        Intake.setPower(0.5);
    }
    
    void IntakeStop() {
        Intake.setPower(0);
    }
    
    void PaddleForward() {
        Paddle.setPosition(0.40);
    }
    
    void PaddleBack() {
        Paddle.setPosition(1);
    }
    
    void Paddle() {
        PaddleForward();
        sleep(400);
        PaddleBack();
    }
    
    void Reverse() {
        FlyWheelReverse();
        sleep(500);
        FlyWheelPowerDown();
    }

    void Launch3() {
        IntakeStart();
        sleep(1000);
        IntakeStop();
        sleep(100);
        FlyWheelPowerUp2();
        IntakeStart();
        sleep(700);
        IntakeStop();
        sleep(200);
        FlyWheelPowerUp2();
        IntakeStart();
        sleep(700);
        IntakeStop();
        Paddle();
        IntakeStop();
        FlyWheelPowerDown();
    }
    
    void PickUp3() {
        FlyWheelPowerDown();
        IntakeStart();
        drive(1400, 4000);
        IntakeStop();
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
        drive(-2000, 2000);
        sleep(500);
        getIMUHeading();
        Launch3();
        sleep(100);
        fortyfivestrafe(-320, 0, 4000);
        turn(-450, 4000);
        IntakeStart();
        sleep(100);
        PickUp3();
        turn(500, 4000);
        //Reverse();
        strafe(-1400, 4000);
        drive(-150, 4000);
        turn(-75, 5000);
        getIMUHeading();
        FlyWheelPowerUp();
        sleep(500);
        Launch3();
        fortyfivestrafe(-2650, 0, 4000);
        turn(-475, 4000);
        sleep(100);
        PickUp3();
        drive(-250, 4000);
        turn(480, 4000);
        //Reverse();
        strafe(-2300, 4000);
        FlyWheelPowerUp();
        sleep(1000);
        Launch3();
        strafe(1000, 5000);
        IntakeStop();
        FlyWheelPowerDown();
    }
   
}
