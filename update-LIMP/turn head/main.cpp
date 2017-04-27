#include "main.h"
#include <curses.h>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <librealsense/rs.hpp>
#include "detect.hpp"
#include <iostream>
#include <fstream>

using namespace std;

ofstream LED[4];


using namespace GaitManager ;
using namespace Robot;
using namespace std;
using namespace boost;

float degree;
float walk_distance;

void callback(const std_msgs::Float32& msg)  {

	degree = int(msg.data)%1000-180;
	walk_distance = int(msg.data/1000)/100.0;

	mWalk.X0 = walk_distance/4;
	if(degree>30)
	        mWalk.RobotYaw_New = 30;
	else if(degree<-30)
	        mWalk.RobotYaw_New = -30;
	else
	        mWalk.RobotYaw_New = degree;
	mWalk.StepCountTarget = 2;
	mWalk.PatternInit(); 
	mWalk.RobotState=mWalk.Walk_start;
}

//ros::Subscriber<std_msgs::Float32> sub1("deltaDegree",1, &callback);

int main(int argc, char **argv)
{
	char LEDIndex[4][50] = {"/sys/class/gpio/gpio2/value","/sys/class/gpio/gpio3/value","/sys/class/gpio/gpio4/value","/sys/class/gpio/gpio14/value"};
	for(int i=0;i<4;i++)
		LED[i].open(LEDIndex[i]);

	degree = -1000;
        ros::init(argc,argv,"ros_node1");
	//advertise();
        ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe("deltaDegree",1, &callback); 
        //ros::Subscriber sub2 = nh.subscribe("path",1, &pathCallback); 

  /*rs:: cam-control*/      
  rs::context ctx;
  if(ctx.get_device_count() == 0) return EXIT_FAILURE;
  rs::device * dev = ctx.get_device(0);
  rs::apply_depth_control_preset( dev,    RS_IVCAM_PRESET_DEFAULT);
  dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
  dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60);
  dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
  dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);

  dev->start();
 
  int sum = 0;
  int counter = 0;
  int lastTimestamp[2]={0};
  int currentTimestamp[2]={0};
  
  void *rgbData, *depthData;
 
 
  const float scale = dev->get_depth_scale();
  const float fx = dev->get_stream_intrinsics(rs::stream::infrared).fx;
  const float fx2 = dev->get_stream_intrinsics(rs::stream::infrared2).fx;
  const float baseline = dev->get_extrinsics(rs::stream::infrared2, rs::stream::infrared).translation[0];
  printf("scale: %f\nfx: %f\nfx2:%f\nbaseline: %f\n", scale, fx,fx2, baseline);
  rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
 
  char imgcounter = 'a';
  cv::Mat depthImg, rgbImg;
  detect de;
  calXYZ cal;

  /*according to the head yaw anf head pitch,correct the angle*/
  
  
  
  	if(cm730.Connect() == true)
	{
		Scan(&cm730);
		Parainit();
		stand();
		//pthread_t thread_t;
		//pthread_create(&thread_t, NULL, JY901_thread, NULL);
		pthread_t thread_LIPM;
		pthread_create(&thread_LIPM, NULL, LIPM_thread, NULL);
		 cout<<"init  mWalk.JointValue[19]:"<<mWalk.JointValue[19]<<endl;
  while (1&&cv::waitKey(30) !=27) 
    {//顯示圖像並且將圓形匹配出來
    dev->wait_for_frames();
    currentTimestamp[0] = dev->get_frame_timestamp(rs::stream::infrared);
    currentTimestamp[1] = dev->get_frame_timestamp(rs::stream::infrared2);
    if(currentTimestamp[0] != lastTimestamp[0]){
        sum += currentTimestamp[0]-lastTimestamp[0];
        if(counter++ > 100){
          printf("%d ms\n", sum/counter);
          counter = sum = 0;
        }
        lastTimestamp[0] = currentTimestamp[0];
        rgbData = (void *)dev->get_frame_data(rs::stream::infrared);//左邊的是不帶2的
        depthData=(void *)dev->get_frame_data(rs::stream::infrared2);//右邊的攝像頭
        rgbImg = cv::Mat(480, 640, CV_8U,rgbData).clone();//CV_8UC3
        depthImg = cv::Mat(480, 640, CV_8U,depthData).clone();
        //cv::cvtColor(rgbImg,rgbImg,CV_BGR2RGB);
        
       char in = cvWaitKey(30);
       
       vector<Vec3f> centre=de.centre( rgbImg,depthImg);
       vector<vector<float> > coo;
       int size =centre.size();//represent the size of centre ,if correct the value is 4
sleep(2);
        cv::imshow("left",rgbImg);
   	cv::imshow("right",depthImg);  
      
       while(size!=4&&cv::waitKey(30) !=27)
	{

           if(mWalk.JointValue[20]<4096.0-2000.0)
           	{
                mWalk.JointValue[20]+=2000.0;


                 sleep(1);
dev->wait_for_frames();
sleep(3);
 rgbData = (void *)dev->get_frame_data(rs::stream::infrared);//左邊的是不帶2的
        depthData=(void *)dev->get_frame_data(rs::stream::infrared2);//右邊的攝像頭
        rgbImg = cv::Mat(480, 640, CV_8U,rgbData).clone();//CV_8UC3
        depthImg = cv::Mat(480, 640, CV_8U,depthData).clone();
 cv::imshow("left",rgbImg);
    cv::imshow("right",depthImg);  

                centre=de.centre( rgbImg,depthImg);
 cout<<"mWalk.JointValue[20]:"<<mWalk.JointValue[20]<<endl;
 	        size=centre.size();
                if(size==4) break;
               }
         else 
		return -1;//down the head

       while(size!=4&&mWalk.JointValue[19]>200.0)
          {//turn left
           mWalk.JointValue[19]-=200.0;
           sleep(1);
dev->wait_for_frames();
sleep(3);
 rgbData = (void *)dev->get_frame_data(rs::stream::infrared);//左邊的是不帶2的
        depthData=(void *)dev->get_frame_data(rs::stream::infrared2);//右邊的攝像頭
        rgbImg = cv::Mat(480, 640, CV_8U,rgbData).clone();//CV_8UC3
        depthImg = cv::Mat(480, 640, CV_8U,depthData).clone();
 cv::imshow("left",rgbImg);
    cv::imshow("right",depthImg);     
     
	   centre=de.centre( rgbImg,depthImg);
 	   size=centre.size();
           cout<<"mWalk.JointValue[19]:"<<mWalk.JointValue[19]<<endl;
          }
        if(mWalk.JointValue[19]<=200.0&&size!=4)
          { mWalk.JointValue[19]=2048.0;

        while(size!=4&&mWalk.JointValue[19]<4096.0-200.0)
          {
           mWalk.JointValue[19]+=200;
           sleep(1);
dev->wait_for_frames();
sleep(3);
 rgbData = (void *)dev->get_frame_data(rs::stream::infrared);//左邊的是不帶2的
        depthData=(void *)dev->get_frame_data(rs::stream::infrared2);//右邊的攝像頭
        rgbImg = cv::Mat(480, 640, CV_8U,rgbData).clone();//CV_8UC3
        depthImg = cv::Mat(480, 640, CV_8U,depthData).clone();
         cv::imshow("left",rgbImg);
    cv::imshow("right",depthImg);  

           centre=de.centre( rgbImg,depthImg);
 	   size=centre.size();
cout<<"mWalk.JointValue[19]:"<<mWalk.JointValue[19]<<endl;
          }

        if(mWalk.JointValue[19]>=4096.0-200.0&&size!=4)
           mWalk.JointValue[19]=2048.0;
        else  if(size==4)
           {//body anf head 
            double angleStep=(double)(mWalk.JointValue[19]-2048.0)/4096.0*360.0/3;
            
 	    double angle[4] = {0.0, -angleStep , -angleStep , -angleStep};
            mWalk.start(0.00,4,angle); //walk  
 while(mWalk.RobotState!=mWalk.Walk_standed){
	    sleep(1);
	    }
                
            mWalk.JointValue[19]=2048.0;//-=angleStep/360.0*4096.0;
          
           }

	} //if(mWalk.JointValue[19]<=200.0&&size!=4)

        else  if(size==4)
           {//body anf head 
            double angleStep=(double)(2048.0-mWalk.JointValue[19])/4096.0*360.0/3.0;
            double angle[4] = {0.0, angleStep , angleStep , angleStep };
            mWalk.start(0.00,4,angle); //walk 
             while(mWalk.RobotState!=mWalk.Walk_standed){
	    sleep(1);
	    }
                 
            mWalk.JointValue[19]=2048.0;//+=angleStep*3.0/360.0*4096.0;
           
             
           }
       
	}//while(size!=4)
    
	if(centre.size()>=4)
        	  coo=de.resultXYZ( centre);
       cv::imshow("left",rgbImg);
    cv::imshow("right",depthImg);  
       
    if(coo.size()>0&&coo[0].size()>0)
        {
           
           int len=coo[0].size()-1;
           cout<<"len="<<len<<endl;
           pair<float,float> d_a=cal.calAngle(len,coo[0],coo[1],coo[2] );
           cout<<"dis:"<<d_a.first<<"    angle:"<<d_a.second/3.14*180.0<<endl;
           
           cout<<"*********************88"<<endl;
 
	   if(abs(d_a.second)>0.0)
          {
             cout<<"angle"<<d_a.second/3.14*180.0<<endl;
            double angleStep=(double)d_a.second/3.14*180.0/3.0;
           
            double angle[4] = {0.0, angleStep , angleStep , angleStep };
            mWalk.start(0.00,4,angle); //wal

   	    while(mWalk.RobotState!=mWalk.Walk_standed){
	    sleep(1);
	    }
         

          }
         else 
          return 0;      


        }// if(coo.size()>0&&coo[0].size()>0)
     



  

      }//   while (1&&cv::waitKey(30) !=27) 


		 
	}//if
}
	return 0;
}
void show_1()
{
	if(mWalk.StepPaused)
		mWalk.RobotState = mWalk.Action_UpDown;

	if(mWalk.UpDownCount == 3){
		mWalk.RobotState = mWalk.Walk_forward;
		mWalk.UpDownCount = 0;
	}
}

void* LIPM_thread(void* ptr)
{
	while(true)
	{
		// getchar();
		gettimeofday(&LIPM_start,NULL); 	double start_time = LIPM_start.tv_usec;
		mWalk.run();
		GyroBalance();//该修正需位于run之后
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 
		if(mWalk.StepCount ==mWalk.StepCountTarget){
			//停止，走完预定步数会自动停止，需要提前停止即执行此函数
			mWalk.stopWalking();
		}

		timing(start_time);
	}

}
void timing(int start_time)
{
	while(true){
		gettimeofday(&LIPM_end,NULL);
		TimeUsed = LIPM_end.tv_usec-start_time;
		if(TimeUsed<0)
			TimeUsed+=1000000;
		if(TimeUsed > mWalk.TimeStep*1000000+1000)
			printf("timing  Severe Error !!!!!!     %d\n",TimeUsed);
		if(TimeUsed>=mWalk.TimeStep*1000000)
			break;
	}
}

void LIPM_publish()
{
	std_msgs::Float64 msg_val;
	int temp;
	if(mWalk.StepRhythm == 1)
		odometer_y +=fabs(2*mWalk.F_cal);

	if(mWalk.StepRhythm != 0){
		msg_val.data=int((mWalk.CoM_x_RelaToP[mWalk.StepRhythm]+odometer_y)*1000);
		com_x_c.publish(msg_val);
		msg_val.data=int(mWalk.V0_x_RelaToP[mWalk.StepRhythm]*1000);
		com_y_c.publish(msg_val);
	}
	else{
		msg_val.data=int((mWalk.CoM_x_RelaToP[mWalk.RhythmCount]+odometer_y)*1000);
		com_x_c.publish(msg_val);
		msg_val.data=int(mWalk.V0_x_RelaToP[mWalk.RhythmCount]*1000);
		com_y_c.publish(msg_val);
	}
}

void advertise()
{
	ros::NodeHandle n;
	error_x =n.advertise<std_msgs::Float64>("error_x",1000);
	error_y =n.advertise<std_msgs::Float64>("error_y",1000);
	v_x_c =n.advertise<std_msgs::Float64>("v_x_c",1000);
	v_x_m =n.advertise<std_msgs::Float64>("v_x_m",1000);
	
	com_x_m =n.advertise<std_msgs::Float64>("com_x_m",1000);
	com_x_c =n.advertise<std_msgs::Float64>("com_x_c",1000);
	com_x_i =n.advertise<std_msgs::Float64>("com_x_i",1000);
	com_y_m =n.advertise<std_msgs::Float64>("com_y_m",1000);
	com_y_c =n.advertise<std_msgs::Float64>("com_y_c",1000);

	JY901X = n.advertise<std_msgs::Float64>("JY901X",1000);
	JY901Y = n.advertise<std_msgs::Float64>("JY901Y",1000);

	torsoroll = n.advertise<std_msgs::Float64>("torsoroll",1000);
	torsopitch = n.advertise<std_msgs::Float64>("torsopitch",1000);

	StepRhythm = n.advertise<std_msgs::Float64>("StepRhythm",1000);

	stepOK = n.advertise<std_msgs::Bool>("stepOK",1000);

}

void CoM_measure()
{
	for(id=9; id<=18; id++)
	{
		if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
			{mWalk.JointReadOK=true; error1[id] = idealvalue[id] - measuredvalue[id];}
		// else
			// {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!\n", id);}
	}

	double length;
	for(int i = 9;i<=18;i++)
		MeasuredAngle[i] = (measuredvalue[i] - JointOffset[i] - 2048)/AngleToValue;
	MeasuredAngle[9] = -MeasuredAngle[9];
	MeasuredAngle[10] = -MeasuredAngle[10];
	MeasuredAngle[13] = -MeasuredAngle[13];
	MeasuredAngle[14] = -MeasuredAngle[14];
	MeasuredAngle[17] = -MeasuredAngle[17];
	MeasuredAngle[18] = -MeasuredAngle[18];

	// CoM_x_caled = mWalk.CoM_x_RelaToP[mWalk.StepRhythm]; 
	CoM_x_caled = mWalk.CoM_x_RelaToW[mWalk.MeasureLagCircle];
	CoM_y_caled = mWalk.CoM_y_RelaToW[mWalk.MeasureLagCircle];
	CoM_H_caled = 0.176;
	CoM_x_m_old = CoM_x_m;
	
	// mWalk.StepState = mWalk.Lfoot_stance;
	if(mWalk.StepState == mWalk.Rfoot_stance)
	{
		length = Shank*cos(MeasuredAngle[R_ANKLE_PITCH]/Rad2Deg) + Thigh*cos(( -MeasuredAngle[R_KNEE] - MeasuredAngle[R_ANKLE_PITCH])/Rad2Deg);		
		CoM_H_m = length*cos(MeasuredAngle[R_ANKLE_ROLL]/Rad2Deg);		
		CoM_x_m =  length*sin(MeasuredAngle[R_ANKLE_ROLL]/Rad2Deg);
		CoM_y_m = Shank*sin(MeasuredAngle[R_ANKLE_PITCH]/Rad2Deg) - Thigh*sin(( -MeasuredAngle[R_KNEE] - MeasuredAngle[R_ANKLE_PITCH])/Rad2Deg);
		CoM_H_m = CoM_H_m + IKoffset_R_H;
		CoM_x_m = CoM_x_m + IKoffset_R_x;
		CoM_y_m = CoM_y_m + IKoffset_R_y;
		torsoRoll = MeasuredAngle[R_ANKLE_ROLL] - MeasuredAngle[R_HIP_ROLL];
		torsoPitch = MeasuredAngle[R_ANKLE_PITCH] - (-MeasuredAngle[R_KNEE] -(-MeasuredAngle[R_HIP_PITCH]));
	} 

	else if(mWalk.StepState == mWalk.Lfoot_stance)
	{
		length = Shank*cos(MeasuredAngle[L_ANKLE_PITCH]/Rad2Deg) + Thigh*cos(( -MeasuredAngle[L_KNEE] - MeasuredAngle[L_ANKLE_PITCH])/Rad2Deg);	
		CoM_H_m = length*cos(MeasuredAngle[L_ANKLE_ROLL]/Rad2Deg);
		CoM_x_m =  length*sin(MeasuredAngle[L_ANKLE_ROLL]/Rad2Deg);
		CoM_y_m = Shank*sin(MeasuredAngle[L_ANKLE_PITCH]/Rad2Deg) - Thigh*sin(( -MeasuredAngle[L_KNEE] - MeasuredAngle[L_ANKLE_PITCH])/Rad2Deg);
		CoM_y_m = - CoM_y_m;
		CoM_H_m = CoM_H_m + IKoffset_L_H;
		CoM_x_m = CoM_x_m + IKoffset_L_x;
		CoM_y_m = CoM_y_m + IKoffset_L_y;
		torsoRoll = MeasuredAngle[L_ANKLE_ROLL] - MeasuredAngle[L_HIP_ROLL];
		torsoPitch = -MeasuredAngle[L_ANKLE_PITCH] - (MeasuredAngle[L_KNEE] - MeasuredAngle[L_HIP_PITCH]);
	}
	mWalk.CoM_H_measured = CoM_H_m;
	mWalk.CoM_x_measured = CoM_x_m;
	mWalk.CoM_y_measured = CoM_y_m;
}

void GyroBalance()
{
	cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &gyroY, 0);

	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
	int dir[14]          = {   -1,        -1,          1,         1,         -1,            1,          -1,        -1,         -1,         -1,         1,            1,           1,           -1      };
 
	// 侧向为X 正向为Y 
	//调试 乘2处默认为乘4
	// mWalk.JointValue[R_HIP_ROLL] += (int)(dir[1] * (gyroX-512) * BALANCE_HIP_ROLL_GAIN*4); // R_HIP_ROLL
	mWalk.JointValue[R_KNEE] -= (int)(dir[3] * (gyroY-512) * BALANCE_KNEE_GAIN*2); // R_KNEE
	mWalk.JointValue[R_ANKLE_PITCH] -= (int)(dir[4] * (gyroY-512) * BALANCE_ANKLE_PITCH_GAIN*2); // R_ANKLE_PITCH
	// mWalk.JointValue[R_ANKLE_ROLL] -= (int)(dir[5] * (gyroX-512) * BALANCE_ANKLE_ROLL_GAIN*4); // R_ANKLE_ROLL

	// mWalk.JointValue[L_HIP_ROLL] += (int)(dir[7] * (gyroX-512) * BALANCE_HIP_ROLL_GAIN*4); // L_HIP_ROLL
	mWalk.JointValue[L_KNEE] -= (int)(dir[9] * (gyroY-512) * BALANCE_KNEE_GAIN*2); // L_KNEE
	mWalk.JointValue[L_ANKLE_PITCH] -= (int)(dir[10] * (gyroY-512) * BALANCE_ANKLE_PITCH_GAIN*2); // L_ANKLE_PITCH
	// mWalk.JointValue[L_ANKLE_ROLL] -= (int)(dir[11] * (gyroX-512) * BALANCE_ANKLE_ROLL_GAIN*4); // L_ANKLE_ROLL
}

void IdealToControl()
{
	for(id=1;id<=20;id++)
	{
		int n = id*5-5;
		int temp;

		//速度控制，动作会更顺滑，受负载时的偏差会更大 对步态影响不好
		// speedvalue = (mWalk.JointValue[id] - idealvalue[id]) / (mWalk.mTimeStep*0.001) *0.1285;//速度控制
		// speedvalue = fabs(speedvalue);
		// if(speedvalue >1023)
		// 	printf("overspeed   id = %d   speedvalue  = %d\n",id ,  speedvalue);
		// if(speedvalue >1023) //调试  1023是否为速度上限需要测试
		// 	speedvalue = 1023;
		// param[n+3]=speedvalue & 0xff;
		// temp = speedvalue & 0xff00;
		// param[n+4]=temp>> 8;

		idealvalue[id] = mWalk.JointValue[id] + JointOffset[id] + ControlOffset[id];
		// controlvalue[id] = idealvalue[id] + error1[id]/2;
		controlvalue[id] = idealvalue[id];
		param[n]=id;
		param[n+1]=controlvalue[id] & 0xff;
		temp = controlvalue[id] & 0xff00;
		param[n+2]=temp>> 8;
	}
}

void readerror()
{
	for(int i=0;i<21;i++)	error1[i] = 0;
	// if(mWalk.StepState == mWalk.Rfoot_stance)
	{
		for(id=9; id<=18; id+=2)
		{
			if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
				{mWalk.JointReadOK=true; error1[id] = idealvalue[id] - measuredvalue[id];}
			// else
				// {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!\n", id);}
		}
	} 
	// else if(mWalk.StepState == mWalk.Lfoot_stance)
	{
		for(id=10; id<=18; id+=2)
		{
			if(cm730.ReadWord(id, 36, &measuredvalue[id], 0) == CM730::SUCCESS)
				{mWalk.JointReadOK=true; error1[id] = idealvalue[id] - measuredvalue[id];}
			// else
				// {mWalk.JointReadOK=false;  printf("id  =  %d    read motor failed!!!!!!!!!     measured failed !!!!!!!!!!!!\n", id);}
		}
	}
}

void stand()
{
	int n;
	int value;
	int wGoalPosition, wDistance;

	n = 0;
	for(id=1; id<=20; id++)
	{
			wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
			wDistance=800;//设置所有舵机速度为800
			param[n++] = id;
			param[n++] = CM730::GetLowByte(wGoalPosition+JointOffset[id]);
			param[n++] = CM730::GetHighByte(wGoalPosition+JointOffset[id]);
			param[n++] = CM730::GetLowByte(wDistance);
			param[n++] = CM730::GetHighByte(wDistance);
			// printf("%d = %d\n",id ,wGoalPosition );
	}
	mWalk.JointValue[19] = 2048.0;// in the middle position
	mWalk.JointValue[20] = 2048.0;

	for(id=1;id<=20;id++)
	{
		n = id*5-5;
		value = mWalk.JointValue[id] + JointOffset[id] + ControlOffset[id];
		param[n]=id;
		param[n+1]=value & 0xff;
		int temp = value & 0xff00;
		param[n+2]=temp>> 8;
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 
	sleep(1);
}

void Parainit()
{
	id = 9;
	mJOINT.SetPGain(id,150);
	mJOINT.SetIGain(id,140);
	mJOINT.SetDGain(id,90);

	id = 10;
	mJOINT.SetPGain(id,150);
	mJOINT.SetIGain(id,140);
	mJOINT.SetDGain(id,90);

	// id = 17;
	// mJOINT.SetPGain(id,100);
	// mJOINT.SetIGain(id,90);
	// mJOINT.SetDGain(id,60);

	// id = 18;
	// mJOINT.SetPGain(id,100);
	// mJOINT.SetIGain(id,140);
	// mJOINT.SetDGain(id,90);

	// id = 11;
	// mJOINT.SetPGain(id,150);
	// mJOINT.SetIGain(id,140);
	// mJOINT.SetDGain(id,90);

	// id = 12;
	// mJOINT.SetPGain(id,150);
	// mJOINT.SetIGain(id,140);
	// mJOINT.SetDGain(id,90);
	// id = 15;
	// mJOINT.SetPGain(id,50);
	// mJOINT.SetIGain(id,20);
	// mJOINT.SetDGain(id,70);

	// id = 16;
	// mJOINT.SetPGain(id,50);
	// mJOINT.SetIGain(id,20);
	// mJOINT.SetDGain(id,70);

	
	for(int i=1;i<21;i++)
	{
		cm730.WriteByte(i,MX28::P_P_GAIN,mJOINT.GetPGain(i),0);
		cm730.WriteByte(i,MX28::P_I_GAIN,mJOINT.GetIGain(i),0);
		cm730.WriteByte(i,MX28::P_D_GAIN,mJOINT.GetDGain(i),0);
	}

	for(id=1;id<=20;id++)
	{
		int n = id*2-2;
		int value = 50;
		param[n]=id;
		param[n+1]=value;
	}
	cm730.SyncWrite(MX28::P_RETURN_DELAY_TIME, 2, JointData::NUMBER_OF_JOINTS - 1, param); 
	for(id=1;id<=20;id++)
	{
		int n = id*2-2;
		int value = 1;
		param[n]=id;
		param[n+1]=value;
	}
	cm730.SyncWrite(MX28::P_RETURN_LEVEL, 2, JointData::NUMBER_OF_JOINTS - 1, param); 
}

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) 
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
		JY901ReceiveOK = true;
	}
}

/*
void* JY901_thread(void* ptr)
{
	while(1)
	{
		char temp = serial.readLine();
		CopeSerialData(temp);

		//Angle[0]为Pitch, Angle[1]为Roll
		if(JY901ReceiveOK == true)
		{
			JY901ReceiveOK = false;
			JY901Pitch = stcAngle.Angle[1]/32768.0*180;
			JY901Roll = stcAngle.Angle[0]/32768.0*180;
			mWalk.JY901Pitch = JY901Pitch;
			mWalk.JY901Roll = JY901Roll;
			//printf("JY901 received\n");
			//std_msgs::Float64 msg_val;
			//msg_val.data=(stcAngle.Angle[1]/32768.0*180);
			//JY901X.publish(msg_val);
			//msg_val.data=(stcAngle.Angle[0]/32768.0*180);
			//JY901Y.publish(msg_val);
		}
	}
    return NULL;
}
*/



/*
	mWalk.X0 = 0.02;
	mWalk.DSPRatio = 0.2;
	mWalk.PatternInit(); 
	mWalk.RobotState=mWalk.Walk_start_slope;
	while(true)
	{
		getchar();
		gettimeofday(&LIPM_start,NULL); 	double start_time = LIPM_start.tv_usec;
		mWalk.run();
		GyroBalance();//该修正需位于run之后
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		if(mWalk.RobotState==mWalk.Walk_standed)
			break;
		if(mWalk.StepCount ==10)
			mWalk.RobotState=mWalk.Walk_stop;
		timing(start_time);
	}

	usleep(800000);
	mWalk.X0 = -0.02;
	mWalk.DSPRatio = 0.1;
	mWalk.Step_TC_y = 6.9;d
	mWalk.PatternInit(); 
	mWalk.RobotState=mWalk.Walk_start_slope;
	while(true)
	{
		getchar();
		gettimeofday(&LIPM_start,NULL); 	double start_time = LIPM_start.tv_usec;
		mWalk.run();
		GyroBalance();//该修正需位于run之后
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		if(mWalk.RobotState==mWalk.Walk_standed)
			break;
		if(mWalk.StepCount ==8)
			mWalk.RobotState=mWalk.Walk_stop;
		timing(start_time);
	}


void readIMU()
{
	cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Z_L, &gyroZ, 0);
	cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &gyroY, 0);
	cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &gyroX, 0);
	cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &accZ, 0);
	cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &accY, 0);
	cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &accX, 0);
}

void Kalman()
{
	if(kalmanStartFlag == false)
	{
		kalmanStartFlag = true;
		zeroValue[0]  = 512;
		zeroValue[1] = 512;
		zeroValue[2] = 512;
		zeroValue[3]  = 512;
		zeroValue[4] = 512;
		zeroValue[5] = 512;
		zeroValue[5]  = 512;

		kalmanX.setAngle(180); // Set starting angle
		kalmanY.setAngle(180);
		KalTimer = mymillis();
		printf("kalman started\n");
	}
	double dt =(double)( mymillis() - KalTimer ) / 1000;   //unit sec , Obtain  time interval dt
	KalTimer = mymillis();

	 double gyroXrate = - (gyroX - zeroValue[0]) / 0.992; // (gyroXadc-gryoZeroX)/Sensitivity - in quids - Sensitivity = 0.00333/3.3*1023=1.0323
	 gyroXangle += gyroXrate * dt; // Without any filter
	// printf("gyroXrate = %f  ",gyroXrate);
	 double gyroYrate = - (gyroY- zeroValue[1]) / 0.992; // (gyroXadc-gryoZeroX)/Sensitivity - in quids - Sensitivity = 0.00333/3.3*1023=1.0323
	 gyroYangle += gyroYrate *dt; // Without any filter
	 //printf("gyroYrate = %f \n",gyroYrate);
	 double accXval = accX - zeroValue[3];
	 double accYval = accY - zeroValue[4];
	 double accZval = accZ - zeroValue[5];

	// Convert to 360 degrees resolution
	 double accXangle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
	 double accYangle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;

	//You might have to tune the filters to get the best values 
	compAngleX = (0.98 * (compAngleX + (gyroXrate * dt))) + (0.02 * (accXangle));
	compAngleY = (0.98 * (compAngleY + (gyroYrate * dt))) + (0.02 * (accYangle));

	kalAngleX  = kalmanX.getAngle(accXangle, gyroXrate, dt);
	kalAngleY  = kalmanY.getAngle(accYangle, gyroYrate, dt);

}
// return ms
double mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}
void show()
{
	///////////////////////////////////////////////////////////////		
	mWalk.RobotState=mWalk.Walk_start;
	while(true)
	{
		printf("in start\n");
		gettimeofday(&start,NULL);
		mWalk.run();
		GyroBalance();
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		while(true){
			gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
			if(TimeUsed>=mWalk.mTimeStep*1000)break;
		}
		if((mWalk.StepPaused)&&(mWalk.StepCount == 3))
			{mWalk.StepCount=0; break;}
	}

	///////////////////////////////////////////////////////////////		
	for(int i=0;i<4;i++)
	{
		printf("in for\n");
		mWalk.RobotState=mWalk.Action_UpDown;			
		while(true)
		{
			gettimeofday(&start,NULL);
			mWalk.run();
			GyroBalance();
			IdealToControl();
			cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

			while(true){
				gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
				if(TimeUsed>=mWalk.mTimeStep*1000)break;
			}
			if(mWalk.UpDownCount == 2)
				{mWalk.UpDownCount=0; break;}
		}

		mWalk.RobotState=mWalk.Walk_forward;
		while(true)
		{
			gettimeofday(&start,NULL);
			mWalk.run();
			GyroBalance();
			IdealToControl();
			cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

			while(true){
				gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
				if(TimeUsed>=mWalk.mTimeStep*1000)break;
			}
		if((mWalk.StepPaused)&&(mWalk.StepCount == 1))
			{mWalk.StepCount=0; break;}
		}
	}

	///////////////////////////////////////////////////////////////		
	// mWalk.RobotState=mWalk.Walk_stop;
	// while(true)
	// {
	// 	// getchar();
	// 	printf("in  first stop\n");
	// 	gettimeofday(&start,NULL);
	// 	mWalk.run();
	// 	GyroBalance();
	// 	IdealToControl();
	// 	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

	// 	while(true){
	// 		gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
	// 		if(TimeUsed>=mWalk.mTimeStep*1000)break;
	// 	}
	// 	if(mWalk.RobotState==mWalk.Walk_stand)
	// 		break;
	// }

	// //改幅度
	// mWalk.RobotState=mWalk.Walk_start;
	while(true)
	{
		gettimeofday(&start,NULL);
		mWalk.run();
		GyroBalance();
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		while(true){
			gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
			if(TimeUsed>=mWalk.mTimeStep*1000)break;
		}
		if((mWalk.StepRhythm ==0)&&(mWalk.StepCount <= 6))
		{
			mWalk.RobotYaw_Temp=10;  //WayPoint_Yaw[WayPointCount];
			printf("in turn 10\n");
		}
		if(mWalk.StepCount > 8)
			{mWalk.StepCount = 0;  break;}

		if((mWalk.StepRhythm ==0)&&(mWalk.StepCount > 6))
		{
			mWalk.RobotYaw_Temp=0;  //WayPoint_Yaw[WayPointCount];
			printf("in turn 0\n");
		}
	}
	mWalk.RobotState=mWalk.Walk_stop;
	while(true)
	{
		printf("in second stop\n");
		gettimeofday(&start,NULL);
		mWalk.run();
		GyroBalance();
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		while(true){
			gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
			if(TimeUsed>=mWalk.mTimeStep*1000)break;
		}
		if(mWalk.RobotState==mWalk.Walk_stand)
			break;
	}

	mWalk.RobotState=mWalk.Action_Jump;		
	while(true)
	{
		// getchar();
		gettimeofday(&start,NULL);
		mWalk.run();
		GyroBalance();
		IdealToControl();
		cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param); 

		while(true){
			gettimeofday(&end,NULL);TimeUsed = end.tv_usec-start.tv_usec;if(TimeUsed<0)TimeUsed+=1000000;
			if(TimeUsed>=mWalk.mTimeStep*1000)break;
		}
		if(mWalk.JumpCount == 13)
			break;
	}
}
*/
