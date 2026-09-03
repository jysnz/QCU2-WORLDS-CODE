#include "autonomous.hpp"
#include "functions.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "motors.hpp"

extern int currentAutonIndex;

void thirty_sec(){
    chassis.moveToPoint(0, -10, 3000);
    chassis.moveToPoint(0, 10, 3000);
    chassis.moveToPoint(0, -10, 3000);
    chassis.waitUntilDone();

    chassis.moveToPose(25, 25, 45, 3000);
    //Shoot the pin and cup
    drivetrainReset();

    chassis.moveToPoint(0, -10, 3000);
    chassis.waitUntilDone();
    chassis.moveToPose(25, 7, 75, 3000);
    chassis.waitUntilDone();
    //Get the pin and cup 
    
    

}

void runAutonomous() {
    // switch (currentAutonIndex) {
    //     case 0: // 2v2 Left
    //         leverReset();
    //         twoVtwo_left_red();
    //         break;
    //     case 1: 
    //         leverReset();
    //         twoVtwo_left_blue();
    //         break;
    //     case 2: // 2v2 Right Red
    //         leverReset();
    //         twoVtwo_right_red();
    //         break;
    //     case 3: // 2v2 Right Blue
    //         leverReset();
    //         twoVtwo_right_blue();
    //         break;
    //     case 4: // Skills
    //         leverReset();
    //         skills();
    //         break;
    //     case 5: // SkillsV1
    //         leverReset();
    //         skillsV1();
    //         break;
    //     case 6:
    //         // SoloAWP_Red_Right
    //          SoloAWP_Red_Right();
    //          break;
    //     default:
    //         break;
    // }
}

