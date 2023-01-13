/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

public class TonyRobotHardware {

    /* Declare OpMode members. */
    private TonyComboOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private BasicParkTony myAutonomous = null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx frontLeft, backLeft, backRight, frontRight;
    public List<DcMotorEx> motors;

    //public DcMotorEx frontLeft = null;

    public DcMotor liftL = null;
    public DcMotor liftR = null;
    public DcMotor armMotor = null;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    //private DcMotor encoderAux;

    public Servo wrist = null;
    public Servo palmL = null;
    public Servo palmR = null;

    public Servo podLeft = null;
    public Servo podRight = null;
    public Servo podBack;

    public TouchSensor liftTouch = null;

    //IMU VARIABLES

    public BNO055IMU imu = null;

    Orientation angles;
    Acceleration gravity;

    public double          targetHeading = 0;
    public double          robotHeading  = 0;
    public double          headingOffset = 0;
    public double          headingError  = 0;

    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    //region Motor Hold Variables
    double armHoldMax   = 0;
    double armHoldMin   = 0;

    int armHoldpos;
    int liftLHoldPos;
    int liftRHoldPos;

    double slopeVal     = 1250;

    //various variables
    int liftLTarPos = 2200;
    int liftRTarPos = 2200;

    int armTarPos = 1500;

    HardwareMap hwMap = null;

    // *  Define a constructor that allows the OpMode to pass a reference to itself.
    public TonyRobotHardware() {
    }

    public void init(HardwareMap ahwMap)    {

        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotorEx.class, "FL");
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight = hwMap.get(DcMotorEx.class, "FR");
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hwMap.get(DcMotorEx.class, "BL");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hwMap.get(DcMotorEx.class, "BR");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftL = hwMap.get(DcMotorEx.class, "liftL");
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftR = hwMap.get(DcMotorEx.class, "liftR");
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // liftR.setMode(DcMotor.ZeroPowerBehavior.BRAKE);


        armMotor = hwMap.dcMotor.get("arm");

        armHoldpos = armMotor.getCurrentPosition();
        liftLHoldPos = liftL.getCurrentPosition();
        liftRHoldPos = liftR.getCurrentPosition();


        encoderLeft  = frontRight;
        encoderRight = frontLeft;
        //encoderAux  = backRight;

        //set drive motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftR.setDirection(DcMotor.Direction.FORWARD);

        //servos
        wrist = hwMap.servo.get("wrist");
        palmL = hwMap.servo.get("palmL");
        palmR = hwMap.servo.get("palmR");
        //fingerL = myOpMode.hardwareMap.get(Servo.class, "fingerL");
        //fingerR = myOpMode.hardwareMap.get(Servo.class, "fingerR");

        wrist.setPosition(1);
        palmL.setPosition(1);
        palmR.setPosition(0);
        //fingerL.setPosition(0.5);
        //fingerR.setPosition(0.5);

        liftTouch = hwMap.get(TouchSensor.class, "liftTouch");
        //GYRO VARIABLES
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ResetDriveEncoders();

    }

    public void ResetDriveEncoders()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
