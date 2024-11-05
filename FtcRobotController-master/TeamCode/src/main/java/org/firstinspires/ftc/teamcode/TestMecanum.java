package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class TestMecanum extends LinearOpMode { //going to write new code in this file



    @Override
    public void runOpMode() throws InterruptedException {
        boolean currentSpikeIntake = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DigitalChannel limitSwitch = hardwareMap.digitalChannel.get("LimitSwitch");
        DcMotorEx ArmIntake  = hardwareMap.get(DcMotorEx.class, "ArmIntake");
        RobotFramework robot = new RobotFramework(this ,hardwareMap);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
        double targetpos = 0;
        while (opModeIsActive()) {


            boolean switchPressed = limitSwitch.getState();
            double pos = ArmIntake.getCurrentPosition() / 28 * 15;







            double horizontalInput = gamepad1.left_stick_x * 1.1;
            double verticalInput = -gamepad1.left_stick_y ;
            double rotationalInput = gamepad1.right_stick_x / 2;
            telemetry.addData("HorizInput", horizontalInput);
            telemetry.update();

            robot.drive(verticalInput,horizontalInput,rotationalInput);



                if (gamepad1.right_trigger > 0.5) {

                    robot.sliderOut();

                } else if (gamepad1.left_trigger>0.5) {
                    robot.sliderin();

                }


            if (gamepad1.right_bumper) {

                robot.intakedown();

            } else if (gamepad1.left_bumper) {
                robot.intakeup();

            }

//
//            if(currentSpikeIntake == false) {
//                if (gamepad1.y) {
//
//                    ArmIntake.setPower(1);
//
//                } else if (gamepad1.x) {
//
//                    ArmIntake.setPower(-1);
//                }
//
//            }
//            else    {
//
//                if(!gamepad1.y && !gamepad1.x){
//
//                    currentSpikeIntake=false;
//
//                }
//
//            }

            if (gamepad1.right_stick_button){

                horizontalInput = horizontalInput / 2;
                verticalInput = verticalInput / 2;
                rotationalInput = rotationalInput / 2;
            }

//            if (gamepad1.left_stick_button){
//
//                horizontalInput = horizontalInput * 2;
//                verticalInput = verticalInput * 2;
//                rotationalInput = rotationalInput * 2;
//
//            }




//            double current = ArmIntake.getCurrent(CurrentUnit.MILLIAMPS);
//            telemetry.addData("Milliamps", current);
//            telemetry.update();
//            if (current > 4500){
//                ArmIntake.setPower(0);
//                currentSpikeIntake = true;
//            }
//            if(currentSpikeIntake == true){
//
//                ArmIntake.setPower(0);
//
//            }
            robot.controlLoop();


        }
    }
}