#include "ros/ros.h"
//#include "inverse_kinematics/destination_topic.h"
#include<cmath>
#include<iostream>
#define _USE_MATH_DEFINES;
#include<math.h>
#include<array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sensor_msgs/JointState.h"
#include <string>
using namespace std;
using namespace Eigen;
//リンク設定
#define L1 0.5
#define L2 0.5
#define L3 0.5


array<double,3> inverse(double x,double y,double z,double a,double b,double c,double ang1,double ang2,double ang3)              //逆運動学
{

    int i=1;
    //角度行列
    MatrixXf ang(3,1);
    ang(0,0) = ang1;
    ang(1,0) = ang2;
    ang(2,0) = ang3;

    double px,py,pz;
    double J1,J2,J3,J4,J5,J6,J7,J8,J9;
    MatrixXf J(3,3);            //ヤコビ行列
    MatrixXf inJ(3,3);          //逆ヤコビ
    MatrixXf dP(3,1);           //⊿P配列
    MatrixXf dAng(3,1);         //⊿角度配列

    while(true)
    {
        px = a -x;       //⊿p＝目標値ー現在地
        py = b-y;
        pz = c-z;
        if(px<0.001 && px>-0.001 && py<0.001 && py>-0.001 && pz<0.001 && pz>-0.001)                 //⊿pが小さくなるとbreak
        {
            cout << "理想値x=" << a << ", y=" << b << ", c=" << c << endl;
            break;
        }
        //ヤコビ要素計算
        J1 = -L2*sin(ang(0,0))*sin(ang(1,0))-L3*sin(ang(0,0))*sin(ang(1,0)+ang(2,0));
        J2 = L2*cos(ang(0,0))*cos(ang(1,0))+L3*cos(ang(1,0))*cos(ang(1,0)+ang(2,0));
        J3 = L3*cos(ang(0,0))*cos(ang(1,0)+ang(2,0));
        J4 = L2*cos(ang(0,0)*sin(ang(1,0)))+L3*cos(ang(0,0))*sin(ang(1,0)+ang(2,0));
        J5 = L2*sin(ang(0,0))*cos(ang(1,0))+L3*sin(ang(0,0))*cos(ang(1,0)+ang(2,0));
        J6 = L3*sin(ang(0,0))*cos(ang(1,0)+ang(2,0));
        J7 = 0;
        J8 = -L2*sin(ang(1,0))-L3*sin(ang(1,0)+ang(2,0));
        J9 = -L3*sin(ang(1,0)+ang(2,0));
        //代入
        J << J1,J2,J3,
             J4,J5,J6,
             J7,J8,J9;

        inJ = J.inverse();
        //目標値までの距離÷10
        dP(0,0) = px/10;
        dP(1,0) = py/10;
        dP(2,0) = pz/10;
        //配列演算
        dAng = inJ*dP;
        //角度＋⊿角度
        ang += dAng;
        //順運動学
        x = L2*cos(ang(0,0))*sin(ang(1,0))+L3*cos(ang(0,0))*sin(ang(1,0)+ang(2,0));
        y = L2*sin(ang(0,0))*sin(ang(1,0))+L3*sin(ang(0,0))*sin(ang(1,0)+ang(2,0));
        z = L1+L2*cos(ang(1,0))+L3*cos(ang(1,0)+ang(2,0));
        double angle1 = ang(0,0)*180/M_PI;
        double angle2 = ang(1,0)*180/M_PI;
        double angle3 = ang(2,0)*180/M_PI;
        //出力
        cout << i << " " << "θ１=" << angle1 << "  θ2=" << angle2 << " θ3=" << angle3 <<"  x=" << x << "  y=" << y << " z=" << z << endl;
        i++;
    }
    return {ang(0,0),ang(1,0),ang(2,0)};

}

array<double,3> forward(double ang1, double ang2,double ang3)                       //順運動学
{
    double px = L2*cos(ang1)*sin(ang2)+L3*cos(ang1)*sin(ang2+ang3);
    double py = L2*sin(ang1)*sin(ang2)+L3*sin(ang1)*sin(ang2+ang3);
    double pz = L1+L2*cos(ang2)+L3*cos(ang2+ang3);
    return {px,py,pz};
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_joint_publisher");
  ros::NodeHandle nh;

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);
  sensor_msgs::JointState jsm;
  jsm.name.resize(3);
  jsm.name[0] = "joint1";
  jsm.name[1] = "joint2";
  jsm.name[2] = "joint3";

  jsm.position.resize(3);
  ros::Rate loop_rate(10);

  double ang1=30*M_PI/180,ang2=45*M_PI/180,ang3=45*M_PI/180;                  //初期角度設定
  array<double,3> result_f = forward(ang1,ang2,ang3);                         //順運動学呼び出し
  cout << "now::x=" << result_f[0] << " y="<< result_f[1] << " z=" << result_f[2] << endl;
  while(ros::ok())
  {
    cout << "目標値を入力してください。" << endl;
    //目標値入力
    double x,y,z;
    cout << "x=";
    cin >> x;
    cout << "y=";
    cin >> y;
    cout << "z=";
    cin >> z;

    array<double,3> result_i = inverse(result_f[0],result_f[1],result_f[2],x,y,z,ang1,ang2,ang3);           //逆運動学呼び出し
    array<double,3> result_f2 = forward(result_i[0] ,result_i[1],result_i[2]);
    //出力
    cout << "theta1=" << result_i[0] << " theta2=" << result_i[1] << " theta3=" << result_i[2] << endl;
    cout << "after::x=" << result_f2[0] << " y="<< result_f2[1] << " z=" << result_f2[2] <<  endl;
    ang1 = result_i[0];
    ang2 = result_i[1];
    ang3 = result_i[2];
    jsm.header.stamp = ros::Time::now();
    jsm.position[0] = ang1;
    jsm.position[1] = ang2;
    jsm.position[2] = ang3;
    joint_pub.publish(jsm);



    loop_rate.sleep();

  }
  return 0;
}
