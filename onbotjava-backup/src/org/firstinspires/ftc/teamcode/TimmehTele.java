package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TimmehTele{
    private DcMotorEx frontLeft, frontRight, backRight, backLeft;
    public TimmehTele (HardwareMap hw){
        frontRight = hw.get(DcMotorEx.class , "Timmy 1");
        backRight = hw.get(DcMotorEx.class , "Timmy 2");
        backLeft = hw.get(DcMotorEx.class , "Timmy 3");
        frontLeft = hw.get(DcMotorEx.class , "Timmy 4");
        
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // reverse Motors
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void TimmehTele(double drive, double turn, double strafe){
       frontLeft.setPower(drive + strafe + turn);
       backRight.setPower(drive - strafe + turn);
       backLeft.setPower(drive - strafe - turn);
       frontRight.setPower(drive + strafe - turn);
       
      
    }
}