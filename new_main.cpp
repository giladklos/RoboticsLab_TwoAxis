#include <iostream>
#include<math.h>
#include<cmath>
#include <conio.h>
#include <iostream>
#include <string>
using namespace std;
#define l1 16
#define l2 6
#define l3 15
#define l4 15
#define l5 6
#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77
#define dm 0.5

bool InBounds(float x, float y) {
    float d = x - l1;
    if ((pow(x, 2) + pow(y, 2)) > (pow((l2 + l3), 2)))
        return false;
    else if ((pow(x, 2) + pow(y, 2)) < (pow((l2 - l3), 2)))
        return false;
    else if ((pow(d, 2) + pow(y, 2)) > (pow((l4 + l5), 2)))
        return false;
    else if ((pow(d, 2) + pow(y, 2)) < (pow((l4 - l5), 2)))
        return false;
    else return true;
}
void opt(float ang1_1, float ang1_2, float ang2_1, float ang2_2, float last_ang1, float last_ang2, float &ang1_send, float &ang2_send){
    if(abs(ang1_1-last_ang1)<abs(ang1_2-last_ang1)?ang1_send=ang1_1:ang1_send=ang1_2);
    if(abs(ang2_1-last_ang2)<abs(ang2_2-last_ang2)?ang2_send=ang2_1:ang2_send=ang2_2);
    if(abs(ang1_1-last_ang1)<abs(ang1_2-last_ang1)?last_ang1=ang1_1:last_ang1=ang1_2);
    if(abs(ang2_1-last_ang2)<abs(ang2_2-last_ang2)?last_ang2=ang2_1:last_ang2=ang2_2);
}
bool inverse_kinematics(float x, float y, float &ang1_1, float &ang2_1, float &ang1_2, float &ang2_2)
{
    //write inverse kinematics function here:
    //every error - return true
    //if invers kinematic is  ok - return false
    if(InBounds(x,y)){
        /////////////////////////////////////////////////////
        //for theta 2
        float d1 = sqrt(pow(x, 2) + pow(y, 2));
        float d2 = sqrt(pow(x - l1, 2) + pow(y, 2));
        float beta2p = acos((pow(l2, 2) + pow(d1, 2) - pow(l3, 2)) / (2 * l2*d1));
        float beta2m = -acos((pow(l2, 2) + pow(d1, 2) - pow(l3, 2)) / (2 * l2*d1));
        float a2 = atan2(y, x);
        ang1_1 = a2 + beta2p;
        ang1_2 = a2 + beta2m;
        // for theta 5;
        float beta5p = acos((pow(l5, 2) + pow(d2, 2) - pow(l4, 2)) / (2 * l5*d2));
        float beta5m = -acos((pow(l5, 2) + pow(d2, 2) - pow(l4, 2)) / (2 * l5*d2));
        float a5 = atan2(y, (x - l1));
        ang2_1 = a5 + beta5p;
        ang2_2 = a5 + beta5m;
        return true;
    }
    //////////////////////////////////////////////////////
    return false;}


int main() {
    float x0=10;
    float y0=10;
    float ang1_1=0;
    float ang1_2=0;
    float ang2_1=0;
    float ang2_2=0;
    float last_ang1=0;
    float last_ang2=0;
    float ang1_send=0;
    float ang2_send=0;
int c=0;
while(1){
    switch((c=getch())) {
        case KEY_UP:
          y0=y0+dm;
        if(inverse_kinematics(x0,y0,ang1_1,ang2_1,ang1_2,ang2_2)) {
            opt(ang1_1, ang1_2, ang2_1,ang2_2, last_ang1, last_ang2,ang1_send,ang2_send);
        }
            break;
        case KEY_DOWN:
           y0=y0-dm;
            if(inverse_kinematics(x0,y0,ang1_1,ang2_1,ang1_2,ang2_2)) {
                opt(ang1_1, ang1_2, ang2_1, ang2_2, last_ang1, last_ang2, ang1_send, ang2_send);
            }
            break;
        case KEY_LEFT:
           x0=x0-dm;
            if(inverse_kinematics(x0,y0,ang1_1,ang2_1,ang1_2,ang2_2)) {
                opt(ang1_1, ang1_2, ang2_1, ang2_2, last_ang1, last_ang2, ang1_send, ang2_send);
            }
            break;
        case KEY_RIGHT:
           x0=x0-dm;
            if(inverse_kinematics(x0,y0,ang1_1,ang2_1,ang1_2,ang2_2)) {
                opt(ang1_1, ang1_2, ang2_1, ang2_2, last_ang1, last_ang2,ang1_send,   ang2_send);
            }
            break;
        default:
            cout << endl << "null" << endl;  // not arrow
            break;
    }

}
    return 0;
}
