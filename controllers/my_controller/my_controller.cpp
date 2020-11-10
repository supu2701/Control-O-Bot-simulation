#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/RangeFinder.hpp>



#define sam left_dist = left_sensor->getValue() > 69000,mright_dist = mright_sensor->getValue() > 69000,mleft_dist = mleft_sensor->getValue() > 69000,right_dist = right_sensor->getValue() > 69000,wheels[0]->setVelocity(left),wheels[1]->setVelocity(right),wheels[2]->setVelocity(left),wheels[3]->setVelocity(right)
#define turnright left = 4, right = 0
#define turnleft left = 0, right = 4
#define nochange left = 6, right = 6,flag=1

#define ran l = *(ds[2]->getRangeImage()) < 0.40, f = *(ds[1]->getRangeImage()) < 0.40, r = *(ds[0]->getRangeImage()) < 0.40
int sw;   //variable declaration for swap function
bool flag;
#define swa(a,b) sw=a,a=b,b=sw
#define TIME_STEP 32
using namespace webots;

double left = 0.0;
double right = 0.0;


//x, y is the current coordinates of the bot
//dirx stores +ve or -ve of the x direction
//diry stores +ve or -ve of the y direction
int dirx = 1, diry = 0, x = 1, y = 0;

// function to change direction
void changedir(int le, int fro, int ri) {
	
	//le, fro, ri check path in left, front and right direction
    //std::cout << "changedir" << std::endl;



//Conditions for changing directions according to the path detected
    if (!fro && le && ri)
        nochange, x += dirx, y += diry;

    else if (fro && !le && ri) {
        turnleft;
        if (diry == 0)x -= diry, y -= dirx, swa(dirx, diry), dirx *= -1, diry *= -1;
        else x += diry, y += dirx, swa(dirx, diry);
    }

    else if (fro && le && !ri) {
        turnright;
        if (dirx == 0)x -= diry, y -= dirx, swa(dirx, diry), dirx *= -1, diry *= -1;
        else x += diry, y += dirx, swa(dirx, diry);
    }


    else if (!fro && abs(5 - x) + abs(5 - y) >= abs(5 - x - dirx) + abs(5 - y - diry))
        nochange, x += dirx, y += diry;


    else if (!le && abs(5 - x) + abs(5 - y) >= abs(5 - x + diry) + abs(5 - y + dirx)) {
        turnleft;
        if (diry == 0)x -= diry, y -= dirx, swa(dirx, diry), dirx *= -1, diry *= -1;
        else x += diry, y += dirx, swa(dirx, diry);
    }

    else if (!ri) {
        turnright;
        if (dirx == 0)x -= diry, y -= dirx, swa(dirx, diry), dirx *= -1, diry *= -1;
        else x += diry, y += dirx, swa(dirx, diry);
    }

    else if (!fro)
        nochange, x += dirx, y += diry;

    else if (!le) {
        turnleft;
        if (diry == 0)x -= diry, y -= dirx, swa(dirx, diry), dirx *= -1, diry *= -1;
        else x += diry, y += dirx, swa(dirx, diry);
    }

}




int main() {
    Robot *robot = new Robot();
    bool l, r, f;

    DistanceSensor *left_sensor = robot->getDistanceSensor("left_sensor");
    DistanceSensor *mleft_sensor = robot->getDistanceSensor("mleft_sensor");
    DistanceSensor *mright_sensor = robot->getDistanceSensor("mright_sensor");
    DistanceSensor *right_sensor = robot->getDistanceSensor("right_sensor");


    left_sensor->enable(TIME_STEP);
    mleft_sensor->enable(TIME_STEP);
    mright_sensor->enable(TIME_STEP);
    right_sensor->enable(TIME_STEP);

    Motor *wheels[4];
    char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++) {
        wheels[i] = robot->getMotor(wheels_names[i]);
        wheels[i]->setPosition(INFINITY);
        wheels[i]->setVelocity(0.0);
    }

    RangeFinder *ds[3];  //for the three sensors at left, right and mid
    char dsNames[3][14] = {"right_range", "mid_range", "left_range"};

    
	//loop to find right, mid and left range
        for (int i = 0; i < 3; i++) {
        ds[i] = robot->getRangeFinder(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }

    bool fla = 0;


    while (robot->step(TIME_STEP) != -1) {

        // read sensors
        bool left_dist = left_sensor->getValue() > 69000;
        bool mleft_dist = mleft_sensor->getValue() > 69000;
        bool mright_dist = mright_sensor->getValue() > 69000;
        bool right_dist = right_sensor->getValue() > 69000;


        // std::cout << *(ds[0]->getRangeImage()) << std::endl;
        // std::cout << *(ds[1]->getRangeImage()) << std::endl;
        // std::cout << *(ds[2]->getRangeImage()) << std::endl;




        // std::cout << left_dist << std::endl;
        // std::cout << mleft_dist << std::endl;
        // std::cout << mright_dist << std::endl;
        // std::cout << right_dist << std::endl;
        // std::cout << "########################" << std::endl;


        //Conditions for setting speed during line following
        if (mright_dist == 0 && left_dist == 0 && right_dist == 0 && mleft_dist == 0)
            left = 10, right = 10;


        else if (left_dist == 0 && right_dist == 0) {
            if (mright_dist == 1)
                left = 10, right = 2;
            else left = 2, right = 10;
        }



        else if (x != 0 && x != 5 && y != 0 && y != 5) {
            ran;
            flag = 0;

            //std::cout << "tripath" << std::endl;
            changedir(l, f, r);   //function calling for changing direction

            while (!(left_dist == 0 && right_dist == 0 && mleft_dist == 0 && mright_dist == 0) && robot->step(TIME_STEP) != -1) {
                sam;
                if (flag)
                    if ((left_dist == 0 && right_dist == 0 && ( mleft_dist == 0 || mright_dist == 0)))break;
            }

            //std::cout << "changedirover" << std::endl;

        }
        else if (((dirx != 0 && y - dirx >= 0 && y - dirx <= 5) || (diry != 0 && x + diry >= 0 && x + diry <= 5)) && ((dirx != 0 && y + dirx >= 0 && y + dirx <= 5) || (diry != 0 && x - diry >= 0 && x - diry <= 5))) {
            ran;
            //std::cout << "leftright" << std::endl;
            changedir(l, 1, r);


            while (!(left_dist == 0 && right_dist == 0 && mleft_dist == 0 && mright_dist == 0) && robot->step(TIME_STEP) != -1)sam;
            // std::cout << "changedirover" << std::endl;

        }
        else if ((dirx != 0 && y - dirx >= 0 && y - dirx <= 5) || (diry != 0 && x + diry >= 0 && x + diry <= 5)) {
            ran;
            //std::cout << "left" << std::endl;
            changedir(l, f, 1);

            while (!(left_dist == 0 && right_dist == 0 && mleft_dist == 0 && mright_dist == 0) && robot->step(TIME_STEP) != -1)sam;

            // std::cout << "changedirover" << std::endl;

        }
        else if ((dirx != 0 && y + dirx >= 0 && y + dirx <= 5) || (diry != 0 && x - diry >= 0 && x - diry <= 5)) {
            ran;
            //std::cout << "right" << f << " " << r << std::endl;
            changedir(1, f, r);

            while (!(left_dist == 0 && right_dist == 0 && mleft_dist == 0 && mright_dist == 0) && robot->step(TIME_STEP) != -1)sam;

            // std::cout << "changedirover" << std::endl;

        }


        std::cout << "xy" << x << " " << y << std::endl;
        // std::cout << "dir" << dirx << " " << diry << std::endl;
        //std::cout << right << std::endl;
        if (x == 5 && y == 5) fla = 1;
        if (fla && (x != 5 || y != 5)) {
            //std::cout << "xypohocha" << x << " " << y << std::endl;
            left = 0;
            right = 0;
        }

        // Setting the final velocity of the wheels taking in account all conditions
        wheels[0]->setVelocity(left);
        wheels[1]->setVelocity(right);
        wheels[2]->setVelocity(left);
        wheels[3]->setVelocity(right);
        if (fla && (x != 5 || y != 5)) {
            //std::cout << "xypohocha" << x << " " << y << std::endl;
            left = 0;
            right = 0;
            break;

        }
        
    }

    delete robot;
    return 0;
}