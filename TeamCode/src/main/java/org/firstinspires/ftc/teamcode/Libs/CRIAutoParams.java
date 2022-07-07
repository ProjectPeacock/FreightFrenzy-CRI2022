package org.firstinspires.ftc.teamcode.Libs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

public class CRIAutoParams {

    public double forwardSpeed = 0;
    public double reverseSpeed = 0;
    public double forwardDistance = 0;
    public double hubDistance1 = 0;
    public double hubDistance2 = 0;
    public double hubDistance3 = 0;
    public double tseDistance = 0;
    public double extraDistance =0;
    public double tseReturnDist = 0;
    public double bucketAngle1= 0;
    public double bucketAngle2= 0;
    public double bucketAngle3= 0;
    public double arcTurnReturn = 0;
    public double warehouseParkDelay = 0;
    public double driveByTime=0;

    public double turnAngle = 65;
    public double parkDistance = 35;
    public double warehouseParkDistance = 0;
    public double turnError = 2;
    public double bucketAngle = 0.3;
    public int    hubFactor = 1;
    public double powerRight = 0;
    public double powerLeft = 0;
    public double arcTime =0;
    public double bonusDistance = 0;

    public double midTurnHubAngle = 0;
    public double midCarouselApproachAngle1 = 0;
    public double midCarouselApproachAngle2 = 0;
    public double midStorageParkTurn = 0;
    public double carTurnHubAngle = 0;
    public String paramsName = "undefined";

    public CRIAutoParams(){

    }
    public void initParams(boolean blueAlliance, int fieldPosition){

        if (fieldPosition == 1) {
            if(blueAlliance){
                blueWarehouse();
            } else {
                redWarehouse();
            }
        }

        if (fieldPosition == 2) {
            if(blueAlliance){
                blueMiddle();
            } else {
                redMiddle();
            }
        }

        if (fieldPosition == 3) {
            if(blueAlliance){
                blueCarousel();
            } else {
                redCarousel();
            }
        }
    }   // end AutoParams constructor

    private void redWarehouse() {
        this.paramsName = "redWarehouse";
        this.warehouseParkDelay = 13;   // Time consumed before parking in the warehouse
        this.arcTime = 1.3;               // length of time for the arc turn
        this.powerLeft = -0.2;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.5;         // power to put on the right side of the robot for arc turn
        this.arcTurnReturn = 15;
        this.forwardSpeed = 0.32;       // speed at which the robot moves
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 10;        // distance to back up from pushing the TSE out
        this.turnAngle = -50;            // angle to turn towards the shipping hub
        this.hubDistance1 = 5;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 6;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 7;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1
        this.parkDistance = 35;         // distance to travel to get in storage parking location
        this.extraDistance = 4;         // extra distance to back up from the hub after X_SCORE
        this.warehouseParkDistance = 30;   // distance to travel to get in warehouse parking location
        this.turnError = 2;             // error to use when turning
        this.hubFactor = 1;             // sets direction to rotate depending on where the hub is
        this.bonusDistance = 20;
    }   // end method redWarehouse

    private void redMiddle(){
        this.paramsName = "redMiddle";
        this.warehouseParkDelay = 27;   // Time consumed before parking in the warehouse
        this.arcTime = 1.9;               // length of time for the arc turn
        this.powerLeft = -0.8;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.2;         // power to put on the right side of the robot for arc turn
        this.forwardSpeed = 0.32;       // speed at which the robot moves
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 14;        // distance to back up from pushing the TSE out
        this.turnAngle = 50;            // angle to turn towards the shipping hub
        this.hubDistance1 = 8;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 6;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 7;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1
        this.parkDistance = 35;         // distance to travel to get in storage parking location
        this.arcTurnReturn = 20;
        this.extraDistance = 5;         // extra distance to back up from the hub after X_SCORE
        this.warehouseParkDistance = 100;   // distance to travel to get in warehouse parking location
        this.turnError = 2;             // error to use when turning
        this.hubFactor = 1;             // sets direction to rotate depending on where the hub is

        this.forwardDistance = 30.0;    //  if using the arcTurn to move a TSE out of the way,

        this.midTurnHubAngle = 35;        // angle to turn from start to approach the alliance hub
        this.midCarouselApproachAngle1 = 96; // angle after first crossing barrier to approach carousel
        this.midCarouselApproachAngle2 = 25; // last angle to turn to approach carousel
        this.midStorageParkTurn = 160;       // turn towards the storage for final approach
        this.carTurnHubAngle = 25;
        // use forward distance to position to score in the hub
    }   // end method redMiddle

    private void redCarousel(){
        this.paramsName = "redCarousel";
        this.warehouseParkDelay = 25;   // Time consumed before parking in the warehouse
        this.arcTime = 1.9;               // length of time for the arc turn
        this.powerLeft = -0.8;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.2;         // power to put on the right side of the robot for arc turn
        this.forwardSpeed = 0.32;       // speed at which the robot moves
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 14;        // distance to back up from pushing the TSE out
        this.turnAngle = 50;            // angle to turn towards the shipping hub
        this.hubDistance1 = 8;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 6;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 7;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1
        this.parkDistance = 35;         // distance to travel to get in storage parking location
        this.arcTurnReturn = 20;
        this.extraDistance = 5;         // extra distance to back up from the hub after X_SCORE
        this.warehouseParkDistance = 100;   // distance to travel to get in warehouse parking location
        this.turnError = 2;             // error to use when turning
        this.hubFactor = 1;             // sets direction to rotate depending on where the hub is

        this.forwardDistance = 30.0;    //  if using the arcTurn to move a TSE out of the way,
        // use forward distance to position to score in the hub
    }   // end method redCarousel


    private void blueWarehouse() {
        this.paramsName = "blueWarehouse";
        this.warehouseParkDelay = 13;   // Time consumed before parking in the warehouse
        this.arcTime = 1.6;               // length of time for the arc turn
        this.powerLeft = -0.55;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.2;         // power to put on the right side of the robot for arc turn
        this.arcTurnReturn = 5;
        this.forwardSpeed = 0.32;      // speed at which the robot moves
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 14;        // distance to back up from pushing the TSE out
        this.turnAngle = 50;            // angle to turn towards the shipping hub
        this.hubDistance1 = 8;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 6;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 8;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1
        this.parkDistance = -35;         // distance to travel to get in storage parking location
        this.extraDistance = 0;         // extra distance to back up from the hub after X_SCORE
        this.warehouseParkDistance = 30;   // distance to travel to get in warehouse parking location
        this.turnError = 2;             // error to use when turning
        this.hubFactor = -1;             // sets direction to rotate depending on where the hub is

    }   // end method blueWarehouse

    private void blueMiddle(){
        this.paramsName = "blueMiddle";
        this.warehouseParkDelay = 25;   // Time consumed before parking in the warehouse
        this.arcTime = 1;               // length of time for the arc turn
        this.arcTurnReturn = 15;
        this.forwardSpeed = 0.32;       // speed at which the robot moves forward
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 14;        // distance to back up from pushing the TSE out
        this.turnAngle = -50;            // angle to turn towards the shipping hub
        this.hubDistance1 = 6;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 8;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 7;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1

        this.extraDistance = 5;         // extra distance to back up from the hub after X_SCORE
        this.parkDistance = 35;         // distance to travel to get in storage parking location
        this.warehouseParkDistance = 100;   // distance to travel to get in warehouse parking location
        this.turnError = 3;             // error to use when turning
        this.hubFactor = -1;             // sets direction to rotate depending on where the hub is
        this.powerLeft = -0.2;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.7;         // power to put on the right side of the robot for arc turn

        this.midTurnHubAngle = -35;         // angle to turn from start to approach the alliance hub
        this.midCarouselApproachAngle1 = -100; // angle after first crossing barrier to approach carousel
        this.midCarouselApproachAngle2 = -25; // last angle to turn to approach carousel
        this.midStorageParkTurn = -160;       // turn towards the storage for final approach
        this.carTurnHubAngle = -25;

    }   // end method blueMiddle

    private void blueCarousel(){
        this.paramsName = "blueCarousel";
        this.warehouseParkDelay = 25;   // Time consumed before parking in the warehouse
        this.arcTime = 1;               // length of time for the arc turn
        this.arcTurnReturn = 15;
        this.forwardSpeed = 0.32;       // speed at which the robot moves forward
        this.reverseSpeed = -0.32;      // speed at which the robot moves in reverse
        this.tseDistance = 34;          // take off distance from the starting point to move TSE
        this.tseReturnDist = 4;        // distance to back up from pushing the TSE out
        this.turnAngle = -90;            // angle to turn towards the shipping hub
        this.hubDistance1 = 0;          // distance to move towards the hub if placing in level 1
        this.hubDistance2 = 0;          // distance to move towards the hub if placing in level 2
        this.hubDistance3 = 0;          // distance to move towards the hub if placing in level 3
        this.bucketAngle1 = -0.75;        // angle of the bucket if placing in level 1
        this.bucketAngle2 = -0.55;        // angle of the bucket if placing in level 1
        this.bucketAngle3 = -1.0;        // angle of the bucket if placing in level 1
        this.driveByTime = 0.5;

        this.extraDistance = 5;         // extra distance to back up from the hub after X_SCORE
        this.parkDistance = 35;         // distance to travel to get in storage parking location
        this.warehouseParkDistance = 100;   // distance to travel to get in warehouse parking location
        this.turnError = 2;             // error to use when turning
        this.hubFactor = -1;             // sets direction to rotate depending on where the hub is
        this.powerLeft = -0.2;          // power to put on the left side of the robot for arc turn
        this.powerRight = -0.7;         // power to put on the right side of the robot for arc turn

    }   // end method blueCarousel

}   // end AutoParams class
