///*本文用來進行雙目測距的實現，通過R200的紅外攝像頭和RGB彩色攝像頭獲得的圖像進行距離的計算*/
#include<iostream>
#include<vector>
#include<math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;

class calXYZ{
public:
/*設置攝像頭參數計算,*/
float  H=250.0f+4.175f;//400.00f;//攝像頭距離地點的高度 
float  T=70.068;//78.18f;//兩個攝像頭之間的距離寬度
float  f=589.201294;//456.35427;//595.0f;//攝像頭的焦距
float  imgWeidth=640.0;
float  imgHeigth=480.0;
vector<float> z;
vector<float> x;
vector<float> y;

/*使用匹配到的特徵點進行距離的計算*/
/*返回向量的最後一個值是所有深度值的加和*/
/*float calZ(IpPairVec matches)
{
       //vector<float> z;//
       float sum=0.0;
       int size=matches.size();
       float depth=0; 
       for(int i=0;i<size;i++)
         {

               float xl=matches[i].second.x;
               cout<<"xl="<<xl<<"  ";
               float xr=matches[i].first.x;
               cout<<"xr="<<xr<<"  ";
               sum+=f*T/(xl-xr);
               cout<<"z="<<f*T/(xl-xr)<<endl;
               z.push_back(f*T/(xl-xr));
         }
  
       z.push_back(sum);
       depth=z[z.size()-1]/(float)(z.size()-1);
       //this->z=z;
       return abs(depth);
       //return z;

}
*/
vector<float> calZ(vector<pair<Point,Point>> matches)
{
       vector<float> z;
       float sum=0.0;
       int size=matches.size();
       float depth=0; 
       for(int i=0;i<size;i++)
         {

               float xl=matches[i].second.x;
               //cout<<"xl="<<xl<<"  ";
               float xr=matches[i].first.x;
               //cout<<"xr="<<xr<<"  ";
               sum+=abs(f*T/(xl-xr));
        //       cout<<"class:z="<<f*T/(xl-xr)<<endl;
               z.push_back(abs(f*T/(xl-xr)));
         }
  
       z.push_back(sum);
       depth=z[z.size()-1]/(float)(z.size()-1);
      // cout<<"depth"<<depth<<endl;
       //this->z=z;
       return z;
       //return z;

}




/*float calX(vector<float> z,IpPairVec matches)
{      
   int size=matches.size();
   float sum=0;
   for(int i=0;i<size;i++)
          {
  
   
	  if((float)matches[i].first.x<imgWeidth/2.0)
		x.push_back(-abs((imgWeidth/2.0-(float)matches[i].second.x)*z[i]/f-(T/2.0)));
	  else if((float)matches[i].first.x==imgWeidth/2.0)//若點映射在左邊攝像頭圖像的中心點的位置,則該物體距離重心點的                 
		x.push_back( -T/2.0);
	  else if((float)matches[i].first.x<imgWeidth/2.0+imgWeidth/4.0)
		x.push_back(-abs((T/2-((float)matches[i].first.x-imgWeidth/2.0)*z[i]/f)));
	  else if((float)matches[i].first.x==imgWeidth/2.0+imgWeidth/4.0)
		x.push_back(0.0);
	  else if((float)matches[i].second.x<imgWeidth/2.0)
		x.push_back(abs(T/2.0-(imgWeidth/2.0-(float)matches[i].second.x)*z[i]/f));
	  else if((float)matches[i].second.x==imgWeidth/2.0)
		x.push_back(T/2.0);
	  else if  ((float)matches[i].second.x>imgWeidth/2.0)
		x.push_back(abs(z[i]/f*((float)matches[i].first.x-imgWeidth/2.0)+(T/2)));
          sum+=x[x.size()-1];

         }//for(i)
        x.push_back(sum);
        //	this->x=x;
    return  sum/(float)(x.size()-1);


}

*/

vector<float> calX(vector<float> z,vector<pair<Point,Point> > matches)
{  vector<float> x;  
   int size=matches.size();
   float sum=0;
   for(int i=0;i<size;i++)
          {
       cout<<matches[i].first.x<<"   "<<matches[i].second.x<<endl;
  
   
	  if((float)matches[i].first.x<imgWeidth/2.0)
		x.push_back(-abs((imgWeidth/2.0-(float)matches[i].first.x)*z[i]/f)-(T/2.0));
	  else if((float)matches[i].first.x==imgWeidth/2.0)//若點映射在左邊攝像頭圖像的中心點的位置,則該物體距離重心點的                 
		x.push_back(-abs(T/2.0));
    
	  else if((float)matches[i].first.x>=imgWeidth/2.0&&((imgWeidth-(float)matches[i].first.x)>((float)matches[i].second.x)))//(float)matches[i].first.x<imgWeidth)//<imgWeidth/2.0+imgWeidth/4.0)
		x.push_back(-abs((T/2-((float)matches[i].first.x-imgWeidth/2.0)*z[i]/f)));
	  else if((float)matches[i].first.x+(float)matches[i].second.x==imgWeidth)//==imgWeidth/2.0+imgWeidth/4.0)表示在
		x.push_back(0.0);
	  else if((float)matches[i].second.x<imgWeidth/2.0&& (imgWeidth-(float)matches[i].first.x)<((float)matches[i].second.x))
		x.push_back(abs(T/2.0-(imgWeidth/2.0-(float)matches[i].second.x)*z[i]/f));
	  else if((float)matches[i].second.x==imgWeidth/2.0)
		x.push_back(abs(T/2.0));
	  else if  ((float)matches[i].second.x>imgWeidth/2.0)
		//x.push_back(abs(((float)matches[i].first.x-imgWeidth/2.0)*z[i]/f-(T/2)));
                x.push_back(abs(z[i]/f*((float)matches[i].second.x-imgWeidth/2.0)+(T/2)));
          else x.push_back(0.0);
                //     cout<<"class:x="<<x[x.size()-1]<<endl;
          sum+=x[x.size()-1];

         }//for(i)
        x.push_back(sum);
         	this->x=x;
  //     cout<<"x:"<<sum/(float)(x.size()-1)<<endl;
    
    return  x;//sum/(float)(x.size()-1);


}

/*float calY(vector<float> z,IpPairVec matches)
{
//在保持攝像頭是水平方向的情況下
   int size=matches.size();
   float sum=0;
   for(int i=0;i<size;i++)
  {
    if(matches[i].first.y<imgHeigth/2.0)//正的位置
       y.push_back(abs((imgHeigth/2.0-matches[i].first.y)*z[i]/f));  
    else if(matches[i].first.y==imgHeigth/2.0)
       y.push_back(0.0);
    else
       y.push_back(abs((matches[i].first.y-imgHeigth/2.0)*z[i]/f));  
    sum+=y[y.size()-1];   
}

 y.push_back(sum);
        //	this->x=x;
    return  sum/(float)(y.size()-1);



}*/
vector<float> calY(vector<float> z,vector<pair<Point,Point> > matches)
{
//在保持攝像頭是水平方向的情況下
   vector<float> y;
   int size=matches.size();
   float sum=0;
   for(int i=0;i<size;i++)
    {
     if(matches[i].first.y<imgHeigth/2.0)//正的位置
        y.push_back(abs((imgHeigth/2.0-matches[i].first.y)*z[i]/f));  
     else if(matches[i].first.y==imgHeigth/2.0)
        y.push_back(0.0);
     else
        y.push_back(-abs((matches[i].first.y-imgHeigth/2.0)*z[i]/f));  
     sum+=y[y.size()-1];   
     //cout<<"class:y="<<y[y.size()-1]<<endl;
      }

     y.push_back(sum);
    //this->x=x;
    //cout<<"y:"<<sum/(float)(y.size()-1)<<endl;
    return y;// sum/(fl`oat)(y.size()-1);





}

/*計算攝像頭是否和充電樁正對,x y z向量分別表示該物理點距離攝像頭中心的距離*/
 std::pair<float,float>    calAngle(int size,vector<float> x,vector<float> y,vector<float> z )
{//根據x方向和z方向的距離和夾角計算機器人應該移動的位置
 //分情況表示，若是連着的x方向相同，則向另一方移動，若是兩者的方向不同，則往另一個方向移動，
 //int size=x.szie();//對其的點的位置
  float dis=-65535.0;//首先表示距離
  float turnAngle=0.0;
 //for(int i=0;i<size;i++)  
 // cout<<"wode"<<endl;
if(size>=2)
   {//對於每個匹配點進行判斷，如是挑選兩個匹配對的話
      cout<<"wode"<<endl;

      if(x[0]<0&&x[1]<0)
      {//如果都在攝像頭左側的情況下
       float pL0x=x[0];//在現在把詞典當成是攝像頭的原點位置，此時右邊攝像頭的移動距離肯定比左邊攝像頭的移動距離大，所以此時左邊的點的x方向肯定發生了變化
       float pR1x=abs(abs(x[0])-abs(x[1]));//防線變化之後的第一個點的投影方向
       float pR1z=pR1x*pR1x+z[1]*z[1];//表示當前和攝像頭之間的絕對距離
       bool  flag=false;
       if(z[0]*z[0]>pR1z)//如果p2點和攝像頭的垂線即z值還是大於當前位置（當前位置是以p2點的垂線和攝像頭水平方向的焦點爲攝像頭原點即攝像頭中心的時候）的p1和攝像頭中心的距離，則說明移動的位置需要過p2點和攝像頭的垂直的交點，此時兩個點的橫座標都是負的，都在攝像頭的左邊位置，然而交點是落在垂線還靠x正方向的位置，所以兩個點的x方向統一變成了負的  如果此時的p2和攝像頭的的距離依然大於p1和攝像頭的距離的話，說明還是要向x正方向移動的，
          {
           int flag=1;
          //p2的座標需要由正變成負數
          //flag=true;//這個值的正負實際上取決於攝像頭和平面之間的夾角度數
          /*求出來的距離表示攝像頭在x方向將要移動的距離，此時兩個特徵點的座標都是正的，同號的*/
          dis=(abs(x[0]),abs(x[1]),abs(z[0]),abs(z[1]));//所求得的x是在x方向將要移動的距離，不是某個點的x座標，雖然x座標是從正的到負的，但是都是按照絕對值進行加減的
          turnAngle=angle(z,x,dis,flag);//要轉向的角度 
          /*接下來是旋轉角度*/
  
          }//說明還要集訓向x方向移動，如果條件不成立說明不用再向x正方向移動
       /**/
       
       else if(z[0]*z[0]==pR1z)//如果在這個地方
         {//表示p1點的投影實在攝像頭x軸方向時的z方向的長度 和
          int flag=1;
          dis=abs(x[0]);//表示
          turnAngle=angle(z,x,dis,flag);//要轉向的角度 
         } 
       else  if(z[0]*z[0]<pR1z)
       {//如果根本到不了p2點垂直的地方，便成爲
        int flag=-1;
        dis=(abs(x[0]),abs(x[1]),abs(z[0]),abs(z[1]));
        turnAngle=angle(z,x,dis,flag);//要轉向的角度 
       }
       
       
      }//if(x[0]<0&&x[1]<0)
 


     //  } 

     
     else if(x[0]>0&&x[1]>0)
       {//如果都在攝像頭右側的情況下,選擇右邊的點P2爲測試點，其投影的位置爲假設的攝像頭移動後的點的位置，此時攝像頭從當前點到測試點的x方向的距離是已知的，可以測出是在當前的時候，攝像頭是否應該繼續向x方向移動，如果需要繼續移動的話，則p2的在x上的投影方向發生了改變
       float pR0x=x[1];//在現在把詞典當成是攝像頭的原點位置，此時右邊攝像頭的移動距離肯定比左邊攝像頭的移動距離大，所以此時左邊的點的x方向肯定發生了變化
       float pL1x=abs(abs(x[1])-abs(x[0]));//防線變化之後的第一個點的投影方向
       float pL1z=pL1x*pL1x+z[0]*z[0];//表示當前和攝像頭之間的絕對距離
       //bool  flag=false ;
       if(z[1]*z[1]>pL1z)//如果p2點和攝像頭的垂線即z值還是大於當前位置（當前位置是以p2點的垂線和攝像頭水平方向的焦點爲攝像頭原點即攝像頭中心的時候）的p1和攝像頭中心的距離，則說明移動的位置需要過p2點和攝像頭的垂直的交點，此時兩個點的橫座標都是負的，都在攝像頭的左邊位置，然而交點是落在垂線還靠x正方向的位置，所以兩個點的x方向統一變成了負的  如果此時的p2和攝像頭的的距離依然大於p1和攝像頭的距離的話，說明還是要向x正方向移動的，
          {
           int flag=1;
          //p2的座標需要由正變成負數
          //flag=true;//這個值的正負實際上取決於攝像頭和平面之間的夾角度數
          /*求出來的距離表示攝像頭在x方向將要移動的距離，此時兩個特徵點的座標都是正的，同號的*/
          dis=(abs(x[0]),abs(x[1]),abs(z[0]),abs(z[1]));//所求得的x是在x方向將要移動的距離，不是某個點的x座標，雖然x座標是從正的到負的，但是都是按照絕對值進行加減的
          turnAngle=angle(z,x,dis,flag);//要轉向的角度 
          /*接下來是旋轉角度*/
  
          }//說明還要集訓向x方向移動，如果條件不成立說明不用再向x正方向移動
       /**/
       
       else if(z[1]*z[1]==pL1z)//如果在這個地方
         {
          int flag=1;
          dis=abs(x[1]);//表示
          turnAngle=angle(z,x,dis,flag);//要轉向的角度 
         } 
       else  if(z[1]*z[1]<pL1z)
       {//如果根本到不了p2點垂直的地方，便成爲
        int flag=-1;
        dis=(abs(x[0]),abs(x[1]),abs(z[0]),abs(z[1]));
        turnAngle=angle(z,x,dis,flag);//要轉向的角度 
       }
       
       
      }//else if(x[0]>0&&x[1]>0)

       else if(x[0]<0&&x[1]>0)
      {//如果分別在兩側的情況下，判斷是偏左還是偏右
     
      if(z[0]==z[1])
      { //如果兩邊的z值相同，說明只可以靠平移就可以到達位置 
         if(abs(x[0])>abs(x[1])) 
           dis=-abs(abs(x[0])-abs(x[1]))/2.0;//表示
          else
           dis=abs(abs(x[0])-abs(x[1]))/2.0;
          turnAngle=0.0;//要轉向的角度 
          
      }
    else if(z[0]*z[0]+x[0]*x[0]==z[1]*z[1]+x[1]*x[1])
   {//說明只通過旋轉就可以達到特定位置
     dis=0.0;
     int flag=1;
     double angleL1=0.0;
     double angleR1=0.0;
     if(x[0]==0.0)
        angleL1=3.14159/2.0;
     else 
        angleL1=atan((double)abs(z[0])/(double)abs(x[0])); 
     if(x[1]==0.0)
        angleR1=3.14159/2.0;
     else 
        angleR1=atan((double)abs(z[1])/(double)abs(x[1]));  
      angleL1>angleR1?( turnAngle=angleR1-angleL1):(turnAngle=angleR1-angleL1);
     //turnAngle=angle(z,x,dis,flag);//要轉向的角度 
          

   }
  else
    {//如果既傾斜又不在中間點(x0-x)*(x0-x)+z0*z0=(x1+x)*(x1+x)+z1*z1;

    // dis=(x[0]*x[0]-x[1]*x[1]+z[0]*z[0]-z[1]*z[1])/(2.0*(x[0]+x[1]));
     dis=disMiddle(abs(x[0]),abs(x[1]),abs(z[0]),abs(z[1]));
     turnAngle=angleMiddle( z, x ,   dis);
     /*double angleL1=0.0;
     double angleR1=0.0;
     if(abs(x[0])-dis==0.0)
        angleL1=3.14159/2.0;
     else 
        angleL1=atan((double)abs(z[0])/(double)abs(x[0]-dis)); 
     if(abs(x[1])+dis==0.0)
        angleR1=3.14159/2.0;
     else 
        angleR1=atan((double)abs(z[1])/(double)abs(x[1]+dis));  
     angleL1>angleR1?( turnAngle=angleR1-angleL1):(turnAngle=angleR1-angleL1);//表示
    */

    } 


      
      }//else if(x[0]<0&&x[1]>0)

    
     //if()


     
  }   //for 結束
   

    pair<float,float>   angle;
    angle.first=dis;
    angle.second=turnAngle;
    return angle;


}//end


float  disMiddle(float x0,float x1,float z0,float z1)//(x0-x)*(x0-x)+z0*z0=(x1+x)*(x1+x)+z1*z1;
{
float fenzi=x0*x0+z0*z0-x1*x1-z1*z1;
float fenmu=2*(x0+x1);
return abs(fenzi/fenmu);
 

}



float dis(float x0,float x1,float z0,float z1)
{//計算是否可以得到,其中x的傳參是絕對值(x-x1)*(x-x1)+z1*z1=(x-x0)*(x-x0)+z0*z0

float fenzi=x0*x0+z0*z0-x1*x1-z1*z1;
float fenmu=2*(x0-x1);
return abs(fenzi/fenmu);

}
double  angleMiddle(vector<float> z,vector<float> x ,float  dis)
{
     double angleL1=0.0;
     double angleR1=0.0;
     if(abs(x[0])-dis==0.0)
        angleL1=3.14159/2.0;
     else 
        angleL1=atan((double)abs(z[0])/(double)abs(abs(x[0])-dis)); 
     if(abs(x[1])+dis==0.0)
        angleR1=3.14159/2.0;
     else 
        angleR1=atan((double)abs(z[1])/(double)abs(abs(x[1])+dis));  
    return  (angleR1-angleL1)/2.0;//angleL1>angleR1?(angleR1-angleL1):(angleR1-angleL1);//表示璁

}
double angle(vector<float> z,vector<float> x ,float  dis,int flag)
{//條件是：在兩個點到攝像頭的距離相等的情況下纔去計算轉角，距離相等的情況需要不斷調節  在某個點的時候想要進行的旋轉角度計算,將距離傳進來之後，根據當前距離計算應當的.1表示移動之後的，0表示移動之前的
   

   double angle=0.0;
   double angleL1=0.0;
   double angleR1=0.0;
   if((((double)dis-(double)abs(abs(x[0]))))==0.0)
     angleL1=3.14159/2.0;
   else
     angleL1=atan((double)abs(z[0])/abs((double)dis-(double)abs(x[0]))); 
   if((((double)dis-(double)abs(x[1])))==0.0)
     angleR1=3.14159/2.0;
   else 
     angleR1=atan((double)abs(z[1])/abs((double)dis-(double)abs(x[1])));
  if(flag==1)
   return (angleL1>(3.14159-angleR1)?-1.0:1.0)*(abs)(3.14-angleR1-angleL1)/2.0;///3.14*180.0;//逆時針是正的角度，順時針的轉角是負的
  if(flag==-1)
   return (angleL1>angleR1?-1.0:1.0)*(abs)(angleR1-angleL1)/2.0;///3.14*180.0;//逆時針是正的角度，順時針的轉角是負的
return 0.0;
}




};
