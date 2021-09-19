#include <iostream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "Eigen/Geometry"
using namespace std;
bool readImuFile(ifstream &imufile,
                 double &time,
                 Eigen::Vector3d &mag,
                 Eigen::Vector3d &acc,
                 Eigen::Vector3d &gyr
                 );
void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,
                vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps);
sensor_msgs::Imu imuMsg;
sensor_msgs::ImagePtr img_0_msg;
sensor_msgs::ImagePtr img_1_msg;
ros::Publisher imu_pub;
ros::Publisher img_0_pub;
ros::Publisher img_1_pub;
string filePath="/media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_imu_gnss/recording_2021-02-25_14-16-43/";//imu 数据位置
string imgPath = "/media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_stereo_images_undistorted/recording_2021-02-25_14-16-43/";//图像数据位置
//imu 路径
string imu_path=filePath+"/imu.txt";//"dump_images/image_capturer_1/imu.txt";
//ifstream imufile(imu_path,ios::in);
string bagPath=filePath+"/test1.bag";

Eigen::Vector3d mag;
Eigen::Vector3d acc;
Eigen::Vector3d gyr;
double imu_time;
int main(int argc, char* argv[])
{
    if(argc<2) return false;
    filePath=argv[1];
    std::cout<<"imu and export bag filePath="<<filePath<<std::endl;
    imu_path=filePath+"/imu.txt";
    if(access(imu_path.c_str(), 0)!=0){std::cout<<"no imu data"<<std::endl;return -1;}
    ifstream imufile(imu_path,ios::in);
    bagPath=filePath+"/test1.bag";//输出rosbag
    imgPath=argv[2];
    string imgTimePath=imgPath+"/times.txt";
    if(access(imgTimePath.c_str(), 0)!=0){std::cout<<"no imgTimePath"<<std::endl;return -1;}
    std::cout<<"loading imu and img ......"<<std::endl;

    ros::init(argc,argv,"Tum");
    ros::NodeHandle n;
    imu_pub=n.advertise<sensor_msgs::Imu>("imu",10);
    img_0_pub=n.advertise<sensor_msgs::Image>("img0",10);
    img_1_pub=n.advertise<sensor_msgs::Image>("img1",10);

    vector<string> Img0FileName,Img1FileName;
    vector<double> imgTimeStamp;
    LoadImages(imgPath, Img0FileName, Img1FileName, imgTimeStamp);//导入图像

    bool endImu=0;
    int i=0;
    rosbag::Bag bag;
    bag.open(bagPath,rosbag::bagmode::Write);
    while(ros::ok())
    {
        if(!endImu)
        {
            endImu=readImuFile(imufile, imu_time, mag, acc, gyr);//读取IMU
            bag.write("imu",ros::Time(imu_time),imuMsg);;
        }
        if(imgTimeStamp[i]<imu_time && i<Img0FileName.size() && i<Img1FileName.size())
        {
            cv::Mat img0= cv::imread(Img0FileName[i], CV_LOAD_IMAGE_UNCHANGED);
            cv::Mat img1= cv::imread(Img1FileName[i], CV_LOAD_IMAGE_UNCHANGED);
//            cv::imshow("left_rbg",img0);
//            cv::imshow("right_rbg",img1);
//            cv::waitKey(1);
            img_0_msg=cv_bridge::CvImage(std_msgs::Header(),"mono8",img0).toImageMsg();
            img_1_msg=cv_bridge::CvImage(std_msgs::Header(),"mono8",img1).toImageMsg();
            img_0_msg->header.stamp=ros::Time(imgTimeStamp[i]);
            img_1_msg->header.stamp=ros::Time(imgTimeStamp[i]);
            img_0_pub.publish(img_0_msg);
            img_1_pub.publish(img_1_msg);
            bag.write("img0",ros::Time(imgTimeStamp[i]),img_0_msg);;
            bag.write("img1",ros::Time(imgTimeStamp[i]),img_1_msg);;
            i++;
        }
        if(endImu || i>=Img0FileName.size() || i>=Img1FileName.size())
        {
            break;
        }
        ros::Duration(0.0001).sleep();
        ros::spinOnce();
    }
    bag.close();
    std::cout << "write bag file in : "<<bagPath << std::endl;
    std::cout << "end!" << std::endl;
    return 0;
}

bool readImuFile(ifstream &imufile,double &time,Eigen::Vector3d &mag,Eigen::Vector3d &acc,Eigen::Vector3d &gyr)
{
    string lineStr_imu;
    std::getline(imufile,lineStr_imu);
    if(imufile.eof())
    {
        std::cout<<"END OF IMU FILE "<<std::endl;
        return 1;
    }
//    cout <<"  getline="<<lineStr_imu << endl;
    stringstream ss(lineStr_imu);
    // 按照逗号分隔
    vector<string> lineArray;
    string str;
    while (getline(ss, str, ' '))
        lineArray.push_back(str);
    double time_now = stod(lineArray[0])/1e9;
    if(time_now==time)return 0;
    time=time_now;
    Eigen::Quaterniond q_;
    gyr.x()=stod(lineArray[1]);
//    gyr.x()=gyr.x()*3.1415926/180;
    gyr.y()=stod(lineArray[2]);
//    gyr.y()=gyr.y()*3.1415926/180;
    gyr.z()=stod(lineArray[3]);
//    gyr.z()=gyr.z()*3.1415926/180;
    acc.x()=stod(lineArray[4]);
    acc.y()=stod(lineArray[5]);
    acc.z()=stod(lineArray[6]);

//    mag.x()=stod(lineArray[14]);
//    mag.y()=stod(lineArray[15]);
//    mag.z()=stod(lineArray[16]);
    //把imu数据压入队列
//    Eigen::Quaterniond q_imu(1,0,0,0);
//    imu_odom.add_imuPose(imu_time,acc,gyr);
    imuMsg.header.frame_id="world";
    imuMsg.header.stamp=ros::Time(time);
    imuMsg.linear_acceleration.x=acc.x();
    imuMsg.linear_acceleration.y=acc.y();
    imuMsg.linear_acceleration.z=acc.z();
    imuMsg.angular_velocity.x=gyr.x();
    imuMsg.angular_velocity.y=gyr.y();
    imuMsg.angular_velocity.z=gyr.z();
//    imuMsg.orientation.x=q_imu.x();
//    imuMsg.orientation.y=q_imu.y();
//    imuMsg.orientation.z=q_imu.z();
//    imuMsg.orientation.w=q_imu.w();
    if(1)//(pub_or_not)
        imu_pub.publish(imuMsg);
//    strArray.push_back(lineArray);
//    int64 num_str=stol(strArray[i][0]);//string 转数字
//    strnum.push_back(num_str);
//    cout<<"imu_time====="<<setprecision(17)<<imu_time<<endl;
    return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps)
{
    string timeFilePath=strPathToSequence+"/times.txt";
    ifstream TimeStream;
    TimeStream.open(timeFilePath.c_str());
    vector<string>imgNameList;
    while(!TimeStream.eof())
    {
        string strline;
        std::getline(TimeStream,strline);
        if(!strline.empty())
        {
            stringstream ss;
            ss<<strline;
            string imgName;
            ss>>imgName;
            imgNameList.push_back(imgName);
            string time_stamp;
            ss>>time_stamp;
//            std::cout<<time_stamp<<std::endl;
            vTimestamps.push_back(stod(time_stamp));
        }
    }

    string strPrefixLeft = strPathToSequence;

    const int nTimes = vTimestamps.size();
    vstrImageFilenames0.resize(nTimes);
    vstrImageFilenames1.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
//        stringstream ss;
//        ss << setfill('0') << setw(6) << i;
//        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageFilenames0[i]=strPathToSequence+"/undistorted_images/cam0/"+imgNameList[i]+".png";
        vstrImageFilenames1[i]=strPathToSequence+"/undistorted_images/cam1/"+imgNameList[i]+".png";  //vstrImageFilenames0是slam帧
        //cv::Mat left=cv::imread(vstrImageFilenames0[i],CV_LOAD_IMAGE_ANYDEPTH);
//        cv::Mat left_rbg;
//        cvtColor(left,left_rbg,CV_BayerBG2RGB,0);
        //cv::imshow("left",left);
//        cv::imshow("left_rbg",left_rbg);
        //cv::waitKey(1);
    }

}
