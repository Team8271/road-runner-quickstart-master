/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "TonyStackRight2", group = "Test")
//@Disabled
public class TonyStackRight2 extends LinearOpMode {

    TonyRobotHardware robot = new TonyRobotHardware();

    private static final String TFOD_MODEL_ASSET = "PPDEC_8.tflite";
    private static final String[] LABELS = {
            "bd",
            "gh",
            "rw"
    };

    //this string is going to return a value based on the signal cone later in the code
    String signal = null;

    private static final String VUFORIA_KEY =
            "Ad/WJUD/////AAABmaHL2VlsskF7gs94CBhc85VMCMsnduT4r56lJ6R1ADz06l0nCXlkYHuEr/9MViHanSiKcefbD5RMEKuNSbMTOmC8JGbEkiQB5a+kE/JDCayLu/0cAj7+y4wkNo2v4YtJlr1YJ5HCLZ1Rzv007cx4S+NbSv3TSxZUQzomnBbZIc/3uLx5S0Sr3eood8gq7xRVTwXh0Rp9GJk+my8sz87vJyg+nZlWXa3q5WzuS0YRq2F5XMDMH1opYjN3Ub+0xFIZO82tBSBQfAMGLruFRyjQ7qpVgPra19wu8PldMmHoGHPdQgT+G6iAGCjClGpcnPtZMXw1VycsGRyjH4pBSH12J5HIheL9b/BTvvBwelC+0FeC";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        //Initialize hardware
        robot.init(hardwareMap);

        //initialize pods down
        SetPodsDown();

/**
 * Activate TensorFlow Object Detection before we wait for the start command.
 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
 **/

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }


/**************************************************************************
 *  Wait for the game to begin
 *  *******************************/

        telemetry.addData(">", "Pods down, ready to play");
        telemetry.update();
        waitForStart();

        getRawHeading();

        telemetry.addData("Hold Pos: ", robot.liftRHoldPos);
        telemetry.addData("Heading: ", getRawHeading());

        if (tfod != null) {
            //analyzeSignal returns the string "A,B or C" depending on what image was recognized
            signal = analyzeSignal();

            telemetry.addData("Zone: ", signal);
            telemetry.update();

/**
 * ********************Run desired zone code *****************************************************************
 * ****************REMEMBER POWER IS DECIMAL 0.0 TO 1.0  NOT 20 *************************************
 */
            //If returns squiggly lines
            if (signal.equals("A")) { //RedWaves go left parking zone1
                Zone1();
            }

            //If returns green hooks
            else if (signal.equals("B")) { //GreenHooks stay center parking zone2
                Zone2();
            }

            //If returns blue circles
            else if (signal.equals("C")){ //BlueDots go rightparking zone3
                Zone3();
            }
        }

        PodsUp(); //retract pods for teleOP
    }//END runOpMode


    /***********************************************
     *      METHODS FOR         !!!!!!
     *      AUTONOMOUS DRIVE,   !!!!!!
     *      ARM, & IMU          !!!!!!
     ***********************************************/

    //Default Drive Power
    double DRIVE_POWER = 0.5;

    //Wait timer for StopDrivingTime function
    public ElapsedTime waitTime = new ElapsedTime();


    /**   IMU    **/

    // read the raw (un-offset Gyro heading) directly from the IMU
    public double getRawHeading() {
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //Reset the "offset" heading back to zero
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        robot.headingOffset = getRawHeading();
        robot.robotHeading = 0;
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        robot.targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robot.robotHeading = getRawHeading() - robot.headingOffset;

        // Determine the heading current error
        robot.headingError = robot.targetHeading - robot.robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (robot.headingError > 180 && opModeIsActive())  robot.headingError -= 360;
        while (robot.headingError <= -180 && opModeIsActive()) robot.headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(robot.headingError * proportionalGain, -1, 1);
    }


    /***************************************************************
     * *********************DRIVE METHODS
     *  ***********************************************************/

    //region encoder drive
    public void StopDriving() throws InterruptedException
    {

        liftHold();

        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);

        robot.FLhold = robot.frontLeft.getCurrentPosition();
        robot.frontLeft.setPower((double)(robot.FLhold - robot.frontLeft.getCurrentPosition()) / robot.slopeVal);

        robot.FRhold = robot.frontRight.getCurrentPosition();
        robot.frontRight.setPower((double)(robot.FRhold - robot.frontRight.getCurrentPosition()) / robot.slopeVal);

        robot.BLhold = robot.backLeft.getCurrentPosition();
        robot.backLeft.setPower((double)(robot.BLhold - robot.backLeft.getCurrentPosition()) / robot.slopeVal);

        robot.BRhold = robot.backRight.getCurrentPosition();
        robot.backRight.setPower((double)(robot.BRhold - robot.backRight.getCurrentPosition()) / robot.slopeVal);

    }

    public void StopDrivingTime(long time)  throws InterruptedException
    {
        waitTime.reset();
        // the not running program was due to a time vs milliseconds() call
        while(waitTime.milliseconds() < time && opModeIsActive())
        {
            //keep the lift in current position
            liftHold();

            telemetry.addData("Stopped: ", time);
            telemetry.update();

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }
    }

    /**
     *
     * DRIVE Forward/Backward
     */

    public void DriveForwardEncoder(double power, int pos) throws InterruptedException
    {
        int ticks = pos * (int)robot.COUNTS_PER_INCH;
        int newLeftTarget = robot.leftEncoder.getCurrentPosition() + ticks;
        int newRightTarget = robot.rightEncoder.getCurrentPosition() + ticks;

        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);

        while((robot.leftEncoder.getCurrentPosition() < newLeftTarget && robot.rightEncoder.getCurrentPosition() < newRightTarget) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("Zone: ", signal);
            telemetry.addData("leftEncoderPos", robot.leftEncoder.getCurrentPosition());
            telemetry.addData("Target", newLeftTarget);
            telemetry.addData("rightEncoderPos", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("Target", newRightTarget);
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void DriveBackwardEncoder(double power, int pos) throws InterruptedException
    {
        int ticks = pos * (int)robot.COUNTS_PER_INCH;
        int newLeftTarget = robot.leftEncoder.getCurrentPosition() - ticks;
        int newRightTarget = robot.rightEncoder.getCurrentPosition() - ticks;

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);

        while((robot.leftEncoder.getCurrentPosition() > newLeftTarget && robot.rightEncoder.getCurrentPosition() > newRightTarget) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("Zone: ", signal);
            telemetry.addData("leftEncoderPos", robot.leftEncoder.getCurrentPosition());
            telemetry.addData("Target", newLeftTarget);
            telemetry.addData("rightEncoderPos", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("Target", newRightTarget);
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }
    /**
     * STRAFE METHODS
     */
    public void StrafeRightEncoder(double power, int pos) throws InterruptedException
    {
        //reset encoder
        //robot.backEncoder ?? how to reset it's not a motor like below
        robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // When tony goes RIGHT, the back encoder value increases, LEFT value decreases

        //set encoder targets
        //convert pos inches to ticks
        int ticks = pos * (int)robot.COUNTS_PER_INCH;
        //add/sub (depending on direction) ticks to the current encoder value
        // possibly use absolute values for a change in encoder vs pos/neg direction values??
        int newBackTarget = robot.backEncoder.getCurrentPosition() + ticks;

        //turn motors on for a RightStrafe
        robot.frontRight.setPower(-power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(-power);

        while((robot.backEncoder.getCurrentPosition() < newBackTarget) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("Target", newBackTarget/robot.COUNTS_PER_INCH);
            telemetry.addData("backEncoder", robot.backEncoder.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }
    public void StrafeLeftEncoder(double power, int pos) throws InterruptedException
    {
        //reset encoder THIS HAS BEEN CHANGED AND NEEDS TESTED
        //robot.backEncoder ?? how to reset it's not a motor
        robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // When tony goes RIGHT, the back encoder value increases, LEFT value decreases
        int ticks = pos * (int)robot.COUNTS_PER_INCH; //RETURNS TICKS TO ACHIEVE DESIRED MOVE IN INCHES

        int newBackTarget = robot.backEncoder.getCurrentPosition() - ticks;

        robot.frontRight.setPower(power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(power);


        while((robot.backEncoder.getCurrentPosition() > newBackTarget) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("Target", newBackTarget/robot.COUNTS_PER_INCH);
            telemetry.addData("backEncoder", robot.backEncoder.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    /**
     * Spin with BackEncoder
     */
    public void SpinRightEncoder(double power, double pos) throws InterruptedException
    {
        //reset encoder THIS HAS BEEN CHANGED AND NEEDS TESTED
        robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int start = robot.backEncoder.getCurrentPosition(); // Display initial value of encoder **should be zero after reset??

        // NOTE: a left spin makes encoder count go neg
        double ticks = pos * (int)robot.COUNTS_PER_INCH;  // calculates how many ticks it takes to rotate encoderWheel to pos
        // will need to determine encoder wheel travel needed to spin desired amount
        // Ex. travel for 90 degree turn?

        //since leftSpin encoder is neg, we need to subtract pos from current to get target
        double  newBackTarget = robot.backEncoder.getCurrentPosition() - ticks;

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);

        //current position should be zero, which is greater than newTarget
        while(robot.backEncoder.getCurrentPosition() > newBackTarget && opModeIsActive())
        {
            liftHold();
            telemetry.addData("start: ", start);
            telemetry.addData("Backtarget: ", newBackTarget);
            telemetry.addData("BackEncoder", robot.backEncoder.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();
    }

    public void SpinLeftEncoder(double power, int pos) throws InterruptedException
    {
        //reset encoder THIS HAS BEEN CHANGED AND NEEDS TESTED
        robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int start = robot.backEncoder.getCurrentPosition(); // Display initial value of encoder **should be zero after reset??

        // right spin encoder count increases, so add to create target
        int ticks = pos * (int)robot.COUNTS_PER_INCH;
        int newBackTarget = robot.backEncoder.getCurrentPosition() + ticks;

        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);

        while(robot.backEncoder.getCurrentPosition() < newBackTarget && opModeIsActive())
        {
            liftHold();
            telemetry.addData("target: ", newBackTarget);
            telemetry.addData("BackEncoder", robot.backEncoder.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();
    }

    /**      ARM & HAND  METHODS     **//*
     */
    //region Arm & Hand Functions
    public void openPalms() throws InterruptedException
    {
        robot.palmL.setPosition(0.5);
        robot.palmR.setPosition(0.5);
    }

    public void closePalms() throws InterruptedException
    {
        robot.palmL.setPosition(1);
        robot.palmR.setPosition(0);
    }

    public void swingArm(double power, int pos) throws InterruptedException {
        //sets arm's starting pos to 0
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armMotor.setTargetPosition(pos);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(power);
        while (robot.armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("swingTarget", pos);
            telemetry.addData("swingPos", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //return motor to regular mode
        //note might want to also set a minimal power to motor to make it hold. Since swingArmHold() will not work in auto
        //note: will not work if swingArm swings through to other side??
        robot.armMotor.setPower(0.2);
    }
    // DON'T BELIEVE THIS WILL WORK IN AUTO
    public void swingArmHold() {
        robot.armHoldpos = robot.armMotor.getCurrentPosition();
        robot.armMotor.setPower((double)(robot.armHoldpos - robot.armMotor.getCurrentPosition()) / robot.slopeVal);
    }
    // AGAIN, NOT SURE THIS WORKS WELL IN AUTO, AS THERE'S NO LOOP TO MONITOR POSITION
    public void liftHold() throws InterruptedException
    {
        robot.liftRHoldPos = robot.liftR.getCurrentPosition();

        //set BOTH of them to the lift L hold pos to avoid accidentally running them both at different values
        robot.liftL.setPower((double)(robot.liftRHoldPos - robot.liftR.getCurrentPosition()) / robot.slopeVal);
        robot.liftR.setPower((double)(robot.liftRHoldPos - robot.liftR.getCurrentPosition()) / robot.slopeVal);
    }

    public void LiftUp(double power, int pos)
    {
        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //NEVER go over 4,000
        while((robot.liftL.getCurrentPosition() < pos) && opModeIsActive())
        {
            robot.liftL.setPower(power);
            robot.liftR.setPower(power);

            // display progress
            telemetry.addData("liftEncoderPos: ", robot.liftL.getCurrentPosition());
            telemetry.update();
        }
        //update lift hold position
        robot.liftRHoldPos = pos;
        //Stop lift - may need some minimal power to hold
        robot.liftL.setPower(0);
        robot.liftR.setPower(0);
        robot.liftL.setPower(0.2);
        robot.liftL.setPower(0.2);
    }
    public void LiftDown(double power, int pos) // input pos will need to be a value less than current.
    {
        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while((robot.liftL.getCurrentPosition() > pos) && !robot.liftTouch.isPressed() && opModeIsActive())
        {
            //keep lift from over driving past bottom touch
            if(robot.liftTouch.isPressed()){
                //Stop lift - may need some minimal power to hold
                robot.liftL.setPower(0);
                robot.liftR.setPower(0);
                robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            else if(!robot.liftTouch.isPressed()) {
                robot.liftL.setPower(-power);
                robot.liftR.setPower(-power);

                // display progress
                telemetry.addData("liftEncoderPos: ", robot.liftL.getCurrentPosition());
                telemetry.update();
            }
        }
        //Stop lift - may need some minimal power to hold
        robot.liftL.setPower(0);
        robot.liftR.setPower(0);
        robot.liftL.setPower(0.2);
        robot.liftR.setPower(0.2);
        //hold stopped position


    }
    //endregion*//*

    public void SetPodsDown()
    {
        robot.podLeft.setPosition(robot.PodLDown);
        robot.podRight.setPosition(robot.PodRDown);
        robot.podBack.setPosition(robot.PodBDown);
    }
    public void PodsUp(){
        robot.podLeft.setPosition(robot.PodLup); //
        robot.podRight.setPosition(robot.PodRup); //
        robot.podBack.setPosition(robot.PodBup); //
    }
    private void initVuforia() {

        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }//endregion
    //region analyze signal

    /**
     * Change the wait time?
     * @return
     */
    public String analyzeSignal()
    {
        waitTime.reset();

        if (tfod != null) {
            //loop needed here with an escape after short waitTime in case no label is ID'd
            while (opModeIsActive() && waitTime.milliseconds() < 300) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getConfidence() > .70) {
                            if (recognition.getLabel().equals("bd")) {
                                //return blue circles park right
                                return "C";

                            } else if (recognition.getLabel().equals("gh")) {
                                //return green hook shapes park center
                                return "B";
                            } else if (recognition.getLabel().equals("rw")) {
                                // return red squiggly lines park left
                                return "A";
                            }
                        }
                    }
                }
            }
        }
        //if we get here then we failed to see anything
        return "C";
    }//end analyze

    //Depending on the return of the signal, we will call these three methods
    //**  NOTE these zones are not being used. Desired movement is coded directly into
    //**       the signal identification at the top (line: 142). They could be reinstated, but don't have to
    private void Zone1() throws InterruptedException
    {
        //our robot will always start with the same opening moves, AKA, AutoOpening
        AutoOpening();
        //StopDrivingTime(500);
        //StrafeLeftEncoder(0.4,11);
        //need to adjust based on where it is after AutoOpening
        //StrafeLeftEncoder(0.7, 20);


    }

    private void Zone2() throws InterruptedException
    {
        AutoOpening();
        //StopDrivingTime(500);
        //StrafeRightEncoder(0.4,11);
        //need to adjust based on where it is after AutoOpening
        //


    }

    private void Zone3() throws InterruptedException
    {
        AutoOpening();
        //StopDrivingTime(500);
        //StrafeRightEncoder(0.4,22);
        //park in zone 3
        //needs figured
        //StrafeRightEncoder(0.8, 20);

    }

    // Opening moves of our autonomous program for ALL zones
    private void AutoOpening() throws InterruptedException
    {
        //swingArmHold
        robot.armMotor.setPower(0.2);
        //closePalms();

        //drive 56"
            DriveForwardEncoder(0.4, 53);

        //strafe left
            StrafeLeftEncoder(0.4,11);
        //lift up
            LiftUp(1, 3850);
        //stop driving
            StopDrivingTime(500);
        // Go to high junction
            DriveForwardEncoder(0.4,2);
        // drop cone
            openPalms();
        //back up
            DriveBackwardEncoder(0.4,2);
        // strafe right
            StrafeRightEncoder(0.4,10);
        // turn
            SpinRightEncoder(0.29,9);
        // cone stack
            DriveForwardEncoder(0.4, 30);
        // lower lift
            LiftDown(0.5,300);
        // grab cone
            closePalms();
            StopDrivingTime(300);
        // lift up
            LiftUp(1,2236);
        // back up
            DriveBackwardEncoder(0.4,26);
        // turn right
            SpinRightEncoder(0.5,9);

        /**
        LiftUp(1, 2750);
        StrafeLeftEncoder(0.4, 11);
        StopDrivingTime(500);
        //forward 6
        DriveForwardEncoder(0.4, 3);
        //openPalms
        openPalms();
        StopDrivingTime(500);
        //backup 6in
        DriveBackwardEncoder(0.4,1);
        //lift down
        //DriveLift(0.3, 0);



        /**.
         * Testing the turn here
         */
        // SpinRightEncoder(0.4,200);


        //SpinRightEncoder(0.4, 10);
        //SpinLeftEncoder(0.4, 10);

        //LiftUp(1, 2750);
        //LiftDown(0.1, 2500);




    }

}