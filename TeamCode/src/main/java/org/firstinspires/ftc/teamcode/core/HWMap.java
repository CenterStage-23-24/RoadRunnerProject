package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    // Drive Motors
    private final Motor leftFrontMotor;
    private final Motor leftBackMotor;
    private final Motor rightBackMotor;
    private final Motor rightFrontMotor;
    private final MecanumDrive mecanumDrive;

    // Mechanism Motors
    private final Motor linearSlidesRight;
    private final Motor linearSlidesLeft;
    private final Motor intakeMotor;
    //IMU
    private static IMU imu;
    public static double imuAngle;

    //Servos
    private final Servo outtakeServoLeft;
    private final Servo outtakeServoRight;
    private final CRServo axonServoLeft;
    private final CRServo axonServoRight;
    private final AnalogInput axonAnalogLeft;
    private final AnalogInput axonAnalogRight;
    private final Servo OdoRetractionLeft;
    private final Servo OdoRetractionRight;
    private final Servo OdoRetractionMiddle;

    //Sensors

    private final RevColorSensorV3 trayLeftCS;
    private final RevColorSensorV3 trayRightCS;
    private final DistanceSensor distanceSensorLeft;
    private final DistanceSensor distanceSensorRight;
    private HardwareMap hardwareMap;

    public HWMap(HardwareMap hardwareMap) {
        //Drive Motors
        this.hardwareMap = hardwareMap;
        rightFrontMotor = new Motor(hardwareMap, "RF", Motor.GoBILDA.RPM_435); //CH Port 0
        leftFrontMotor = new Motor(hardwareMap, "LF", Motor.GoBILDA.RPM_435);//CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = new Motor(hardwareMap, "LB", Motor.GoBILDA.RPM_435); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = new Motor(hardwareMap, "RB", Motor.GoBILDA.RPM_435);//CH Port 3. The left odo pod accesses this motor's encoder port.

        //Linear Slides Motors - ROBOT
        linearSlidesLeft = new Motor(hardwareMap, "LSL", Motor.GoBILDA.RPM_1150); //EH Port 2
        linearSlidesRight = new Motor(hardwareMap, "LSR", Motor.GoBILDA.RPM_1150);//EH Port 3

        /*
        //Linear Slides Motors - TEST BENCH
        linearSlidesLeft = new Motor(hardwareMap, "LSL", Motor.GoBILDA.RPM_312); //EH Port 2
        linearSlidesRight = new Motor(hardwareMap, "LSR", Motor.GoBILDA.RPM_312);//EH Port 3
         */

        // Intake Motor
        intakeMotor = new Motor(hardwareMap, "IM", Motor.GoBILDA.RPM_435); //EH Port 0

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0
        if(imu == null){
            imu = hardwareMap.get(IMU.class, "imu");
            initializeIMU();
            linearSlidesLeft.resetEncoder();
        }

        //Outtake Servos
        outtakeServoLeft = hardwareMap.get(Servo.class, "OSL"); //EH Port 4
        outtakeServoRight = hardwareMap.get(Servo.class, "OSR");//EH Port 5

        //ODO retraction Servos
        OdoRetractionLeft = hardwareMap.get(Servo.class, "ORL"); //CH Port 0
        OdoRetractionRight = hardwareMap.get(Servo.class, "ORR");//CH Port 1
        OdoRetractionMiddle = hardwareMap.get(Servo.class, "ORM");//CH Port 2

        //Linear Slides Servos
        axonServoLeft = new CRServo(hardwareMap, "ASL");//EH Port 0
        axonServoRight = new CRServo(hardwareMap, "ASR");//EH Port 2
        axonAnalogLeft = hardwareMap.get(AnalogInput.class, "AAL"); //EH Port 0
        axonAnalogRight = hardwareMap.get(AnalogInput.class, "AAR"); //EH Port 2

        axonServoLeft.setInverted(false);//Counterclockwise
        axonServoRight.setInverted(false);//Clockwise

        //Mapping Sensors
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "DSR");//CH Port 0
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DSL");//CH Port 2

        trayRightCS = hardwareMap.get(RevColorSensorV3.class, "TRCS");//EH Port 1
        trayLeftCS = hardwareMap.get(RevColorSensorV3.class, "TLCS");//EH Port 2

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        linearSlidesLeft.setInverted(false);
        linearSlidesRight.setInverted(true);

        //Set Motor Mode
        leftBackMotor.setRunMode(Motor.RunMode.RawPower);
        rightBackMotor.setRunMode(Motor.RunMode.RawPower);
        leftFrontMotor.setRunMode(Motor.RunMode.RawPower);
        rightFrontMotor.setRunMode(Motor.RunMode.RawPower);

        linearSlidesRight.setRunMode(Motor.RunMode.PositionControl);
        linearSlidesLeft.setRunMode(Motor.RunMode.PositionControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        //Mecanum Drive Initialization
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        mecanumDrive.setRightSideInverted(false);
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
    }


    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuAngle;
    }

    public static void initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);//This change is only for test bench it should normally be RIGHT and UP.
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
    }


    public DistanceSensor getDistanceSensorRight() {
        return distanceSensorRight;
    }

    public DistanceSensor getDistanceSensorLeft() {
        return distanceSensorLeft;
    }

    public RevColorSensorV3 getTrayRightCS() {
        return trayRightCS;
    }

    public RevColorSensorV3 getTrayLeftCS() {
        return trayLeftCS;
    }

    public Servo getOuttakeServoRight() {
        return outtakeServoRight;
    }

    public Servo getOuttakeServoLeft() {
        return outtakeServoLeft;
    }

    public CRServo getAxonServoRight() {
        return axonServoRight;
    }

    public CRServo getAxonServoLeft() {
        return axonServoLeft;
    }

    public AnalogInput getAxonAnalogRight() {
        return axonAnalogRight;
    }

    public AnalogInput getAxonAnalogLeft() {
        return axonAnalogLeft;
    }

    public Motor getLinearSlidesRight() {
        return linearSlidesRight;
    }

    public Motor getLinearSlidesLeft() {
        return linearSlidesLeft;
    }

    public Motor getIntakeMotor() {
        return intakeMotor;
    }

    public Motor getRightBackMotor() {
        return rightBackMotor;
    }

    public Motor getLeftBackMotor() {
        return leftBackMotor;
    }

    public Motor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public Motor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public int getOdoReadingLeft() {
        return rightBackMotor.getCurrentPosition();
    }

    public int getOdoReadingPerpendicular() {
        return leftBackMotor.getCurrentPosition();
    }

    public int getOdoReadingRight() {
        return leftFrontMotor.getCurrentPosition();
    }

    public Servo getOdoRetractionLeft() {
        return OdoRetractionLeft;
    }

    public Servo getOdoRetractionRight() {
        return OdoRetractionRight;
    }

    public Servo getOdoRetractionMiddle() {
        return OdoRetractionMiddle;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public IMU getImu() {
        return imu;
    }
}