package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFramework {



    PIDFController armnPID = new PIDFController(.01,01,.01,0);
    PIDFController sliderPID = new PIDFController(.01,.01,.01,0);
    double sliderpos;
    double slidertargetpos;

    double targetpos;
    double armpos;
    DcMotorEx ArmIntake;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor ;
    DcMotor backLeftMotor ;
    DcMotor backRightMotor ;
    DigitalChannel limitSwitch;

    DcMotorEx leftSlider;
    DcMotorEx rightSlider;


    public RobotFramework(LinearOpMode mode, HardwareMap hardwareMap){

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        limitSwitch = hardwareMap.digitalChannel.get("LimitSwitch");
        ArmIntake  = hardwareMap.get(DcMotorEx.class, "ArmIntake");
        leftSlider  = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider  = hardwareMap.get(DcMotorEx.class, "rightSlider");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armpos = 0;
        while (!mode.opModeIsActive()){

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                System.out.println("done wait");
            }

        }

        while (mode.opModeIsActive()){

            controlLoop();

        }
    }
    public void intakedown(){
      targetpos =0;

    }
    public void sliderOut(){
        slidertargetpos =5;

    }
    public void sliderin(){
        slidertargetpos =0;

    }

    public void intakeup(){
        targetpos =10;

    }
    public void controlLoop() {
        armpos = ArmIntake.getCurrentPosition() / 28 * 15;


        double armPower = armnPID.calculate(armpos, targetpos);
        ArmIntake.setPower(armPower);
        // if (armpos - targetpos < -0.2){
//            ArmIntake.setPower(1);
//        } else if (armpos-targetpos > 0.2) {
//            ArmIntake.setPower(-1);
//        }
//        else {
//            ArmIntake.setPower(0);
//        }

        if (limitSwitch.getState() == true) ;
        {
            armpos = 0;
        }
        sliderpos = rightSlider.getCurrentPosition();


        double sliderPower = sliderPID.calculate(sliderpos, slidertargetpos);
        leftSlider.setPower(sliderPower);
        rightSlider.setPower(sliderPower);
    }


    public void drive(double forward,double horizontal, double rotate){

        double denom = Math.max(Math.abs(forward) + Math.abs(horizontal) + Math.abs(rotate),1);
        frontLeftMotor.setPower(horizontal+forward+rotate / denom);
        frontRightMotor.setPower(forward-horizontal - rotate/ denom);
        backLeftMotor.setPower(forward-horizontal +rotate/ denom);
        backRightMotor.setPower(horizontal+forward -rotate/ denom);
    }
    public void stop(){
        drive(0,0,0);

    }
}
