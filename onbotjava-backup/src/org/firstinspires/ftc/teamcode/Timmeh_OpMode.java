package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp

public class Timmeh_OpMode extends OpMode{
    
    TimmehTele dt; 
    @Override 
    public void init(){
      dt = new TimmehTele(hardwareMap);  
    }
    @Override
    public void loop(){
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        dt.TimmehTele(drive, strafe, turn);
    }
    

    // todo: write your code here
}