#include <iostream>
#include <vector>

#include<iostream>
#include<string>
#include<stdio.h>
#include "calXYZ.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "colorRGB.h"
using namespace std;
using namespace cv;
//typedef Point3_<float> Point3f;

class  detect{
public:

vector<pair<Point,Point> > matches;

char * color;
void setColor(char* color)
{
this->color=color;
return ;

}
calXYZ cal;
detect(){}
detect(vector<pair<Point,Point> > matches)
{
this->matches=matches;
cal.matches=matches;
//init
}

bool isWhite(int j,int i,Mat img)
{
 const uint8_t* rowPtr = img.ptr<uint8_t>(j);
 uint8_t pixel=rowPtr[i];
if((int)pixel>=200||(i<640&&(int)rowPtr[i+1]>=200)||(i<640&&(int)rowPtr[i+2]>=200)||(i<640&&(int)rowPtr[i+3]>=200)||(i<640&&(int)rowPtr[i+4]>=200))
 return true;
return false;

}

/*掃描整幅圖像找到紅外的中心點，按照列掃描，找到每一類的中心點*/
/*Vec3f  irDetectCol(Mat img,int beginCol)
{
//vector<Vec3f> circle;
Vec3f p = Vec3f(-1.0,-1.0,-1.0);
int begin0=-1;
int end0=-1;
int begin1=-1;
int end1=-1;
for(int i=beginCol;i<640;i++)
  {


  for(int j=0;j<480;j++)
    {//
      
    const uint8_t* rowPtr = img.ptr<uint8_t>(j);
    uint8_t pixel=rowPtr[i];
   if(isWhite( j, i, img))//(int)pixel>200)//
   {
    if(begin0<0.0)//(int)pixel>250&&
       {begin0=i;p[0]=begin0;break;}
    if(begin0>=0.0&&end0<0.0)//(int)pixel>200&&
     { end0=i;break;}
    if(begin0>=0.0&&i>end0) 
    {
    // if(i>end0)
       end0=i;
      break;
    }  
}//if(pixel>(ushort)200)



else if(begin0>0.0&&end0>0.0&&abs(begin0-end0)>5)
       {
p[0]=(float)begin0;
p[1]=(float)end0;
p[2]= 255.0 ;
 
return p;
}
    }//for(j)

}//for(i)
p[1]=(float) end0;
p[2]=(float)-255;
return p;


}
*/

Vec3f  irDetectCol(Mat img,int beginCol)
{
//vector<Vec3f> circle;
Vec3f p = Vec3f(-1.0,-1.0,-1.0);
int begin0=-10;
int end0=-10;
int begin1=-10;
int end1=-1;
int i=0;
for(  i=beginCol;i<640;i++)
  {


  for(int j=0;j<480;j++)
    {//
      
   	 const uint8_t* rowPtr = img.ptr<uint8_t>(j);
   	 uint8_t pixel=rowPtr[i];
   	 if((int)pixel>250)//isWhite( j, i, img))//(
   	  if(begin0<0)//(int)pixel>250&&
   	    {
    	    begin0=i;p[0]=(float)begin0;break;
  	     }
     
   }//for(j)
if(begin0>0) break;
}//for(i)
//find the first col of white

if(i==640)  { return p;}


for(  i=begin0+1;i<640;i++)
  {
int count=0;

  for(int j=0;j<480;j++)
    {//
      
    const uint8_t* rowPtr = img.ptr<uint8_t>(j);
    uint8_t pixel=rowPtr[i];
    if((int)pixel<=250)//isWhite( j, i, img))//(
      count++;
     
     
    }//for(j)
if(count==480&&end0<0.0) {end0=i;p[1]=end0;}
if(end0>0) break;
}//for(i)


/*if(i==640)
  p[1]=(float)639;*/

//p[2]=(float)-255;
return p;


}




vector<Vec3f>  centreDetect(Mat img,int beginCol)
{// return the center of one point
vector<Vec3f> c;

int begin0=-1;
int end0=-1;
Vec3f p = Vec3f(-1.0,-1.0,-1.0); 
p= irDetectCol(img, beginCol);//the return value is the begin and end col of white

if(p[0]>0.0&&p[1]>0.0)
 c.push_back(p);
else return c;
Vec3f centre = Vec3f(-1.0,-1.0,-1.0);
int j=0;
//cout<<"eoder"<<endl;
for(  j=0;j<480;j++)
 {
  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
   
 for(int i=p[0];i<p[1]+1;i++)
   {
     uint8_t pixel=rowPtr[i];
 if((int)pixel>250)//isWhite( j, i, img))//
   {
if(begin0<0)
 { 
 	begin0=j;
	centre[0]=(float)begin0;
   	break; 

}
 
}


   }//for(i)
if(begin0>0.0) break;
 }//for(j)
if(j==480){/*c.push_back(centre);*/return c;  }
/**/
//cout<<"youleyigezhi"<<endl;
for(  j=centre[0];j<480;j++)
 {int count=0;
  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
   
 for(int i=p[0];i<p[1]+1;i++)
   {
     uint8_t pixel=rowPtr[i];
 if((int)pixel<=250)//isWhite( j, i, img))//
   {
 count++;
 
   }

}//for(i)
if(count==p[1]-p[0]+1&&end0<0.0)
   {
  end0=j;
  centre[1]=(float)end0;
  break;
   }

if(end0>0) break;
 }//for(j)
if(begin0>0&&end0>0)
c.push_back(centre);
//cout<<"c.size():"<<c.size()<<endl;
//if(c.size()>=2)
//cv::circle(img,cv::Point((c[0][0]+c[0][1])/2,(c[1][0]+c[1][1])/2),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
return c;
}//end






vector<Vec3f> centre(Mat img1)
{
      Vec3f p=Vec3f(-1.0,-1.0,-1.0);
      vector<Vec3f> centre;
      vector<Vec3f> centreL=  centreDetect( img1,0);//from the first col begin to scan the value 255,every point have the upleft and downright
      vector<Vec3f>  centreR;
       if(centreL.size()>=2)
       { 
         centreR=  centreDetect( img1,centreL[0][1]+1);//centreR contain two value (colbegin,colend) (rowbegin,rowend),the second in the first picture;
       }
   if(centreR.size()>=2)
      {
	p[1]=(centreL[1][0]+centreL[1][1])/2;//第幾行表示縱座標
	p[0]=(centreL[0][0]+centreL[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
	centre.push_back(p);
	p[1]=(centreR[1][0]+centreR[1][1])/2;//第幾行表示縱座標
	p[0]=(centreR[0][0]+centreR[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
 

       }
return centre;//

}





vector<Vec3f> centre(Mat &img1,Mat& img2)
{//the first is ir,the second is rgb
//return four points of the two images
      Vec3f p=Vec3f(-1.0,-1.0,-1.0);
      vector<Vec3f> centre;
      vector<Vec3f> centreL= centreDetect( img1,0);//from the first col begin to scan the value 255,every point have the upleft and downright
      vector<Vec3f>  centreR;
      if(centreL.size()>=2)
       { 
         centreR=  centreDetect( img1,centreL[0][1]+1);//centreR contain two value (colbegin,colend) (rowbegin,rowend),the second in the first picture;
       }

      if(centreR.size()>=2)
      {
	p[1]=(centreL[1][0]+centreL[1][1])/2;//第幾行表示縱座標
	p[0]=(centreL[0][0]+centreL[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
	centre.push_back(p);
	p[1]=(centreR[1][0]+centreR[1][1])/2;//第幾行表示縱座標
	p[0]=(centreR[0][0]+centreR[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
/*the two point of the first picture*/

       }


      vector<Vec3f>  centreRR;
      vector<Vec3f> centreRL= rgbCentreDetect( img2,0);
       if(centreRL.size()>=2)
       { 
       centreRR=  rgbCentreDetect(img2,centreRL[0][1]+1);

       }


      if(centreRR.size()>=2)
      {
	p[1]=(centreRL[1][0]+centreRL[1][1])/2;
	p[0]=(centreRL[0][0]+centreRL[0][1])/2;
	cv::circle(img2,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
	p[1]=(centreRR[1][0]+centreRR[1][1])/2;
	p[0]=(centreRR[0][0]+centreRR[0][1])/2;
	cv::circle(img2,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
/*the two point of the second picture*/
       }


return centre;//every value present a point
        
}












/*
vector<Vec3f>  centreDetect(Mat img,int beginCol)
{
vector<Vec3f> c;

int begin0=-640;
int end0=-640;
Vec3f p = Vec3f(-1.0,-1.0,-1.0); 
p= irDetectCol(img, beginCol);
c.push_back(p);
Vec3f centre = Vec3f(-1.0,-1.0,-1.0);
for(int j=0;j<480;j++)
 {
  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
   
 for(int i=p[0];i<p[1];i++)
   {
     uint8_t pixel=rowPtr[i];
 if(isWhite( j, i, img))//(int)pixel>200)//
   {
if(begin0<0&&end0<0)
 { begin0=j;
  centre[0]=(float)begin0;
break; }
if(begin0>0&&i>end0)
  { end0=i;break;}

   }

else if(begin0>0&&end0>0&&abs(begin0-end0)>5)
{
centre[0]=(float)begin0;
centre[1]=(float)end0;
centre[2]=255.0;
c.push_back(centre);
return c;
}


   }//for(i)

 }//for(j)

centre[1]=(float)end0;
centre[2]=255.0;
c.push_back(centre);
return c;
}//end
*/













vector<vector<float> > resultXYZ(vector<Vec3f> centre) 
{
vector<vector<float> > re;
vector<Vec3f> circle ;
vector<Vec3f> circleR;
if(centre.size()==4)
{
circle.push_back(centre[0]);
circle.push_back(centre[1]);
circleR.push_back(centre[2]);
circleR.push_back(centre[3]);
return resultXYZ( circle ,  circleR) ;
}
/*the circle and circleR represent the centre of the goal*/
else return re;
}





Vec3f  rgbDetectCol(Mat img,int beginCol)
{//rgb  image blue
//vector<Vec3f> circle;
	Vec3f p = Vec3f(-1.0,-1.0,-1.0);
	int begin0=-10;
	int end0=-10;
	int begin1=-10;
	int end1=-1;
	int i=0;
//if(this->color!="red"&&this->color!="green"&&this->color!="blue")
// {}


for(i=beginCol*3;i<640*3;i+=3)
  {


  for(int j=0;j<480;j++)
    {//
	
      	  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
	  colorRGB color(rowPtr[i],rowPtr[i+1],rowPtr[i+2]);
	//cout<<"111111111111"<<endl;
	/* bool flagRGB=color.isThisColor(this->color);
	   bool flagHSV=color.isThisColorbyHSV(this->color);*/
	  bool  flagHSV= color.isGreen();
 	  //flagHSV=color.isThisColorbyHSV(this->color);
 	//cout<<"rgbDetectCol:"<<this->color<<endl;
	 if(flagHSV) 
           if(begin0<0) 
	{
	/*cout<<"1  color:("<<(int)color.r<<",  "<<(int)color.g<<",  "<<(int)color.b<<endl;;
	cout<<"1  hsv:("<<color.HSV[0]<<",  "<<color.HSV[1]<<",  "<<color.HSV[2]<<endl;	
	*/
	begin0=i/3;
	p[0]=(float)begin0;
	break;
	}  //delete//
	//if(color.isThisColor(this->color)){}//set the color and paramter
   	 
	/* uint8_t pixelR=rowPtr[i];
	 uint8_t pixelG=rowPtr[i+1];
	 uint8_t pixelB=rowPtr[i+2];
         int GR=0,GB=0;
	 GR=(int)pixelG-(int)pixelR;
 	 GB=(int)pixelG-(int)pixelB;
   	 if((int)pixelG>200&&GR>180&&GB>180)//isWhite( j, i, img))//(
   	  if(begin0<0)//(int)pixel>250&&
   	    {
    	    begin0=i;p[0]=(float)begin0;break;
  	     }//here
     */
   }//for(j)
if(begin0>0) break;
}//for(i)
//find the first col of white

if(i==640*3)  { return p;}


for(  i=begin0*3+1;i<640*3 ;i+=3)
  {
	int count=0;

  for(int j=0;j<480;j++)
    {//
	
 	 const uint8_t* rowPtr = img.ptr<uint8_t>(j);
	 colorRGB color(rowPtr[i],rowPtr[i+1],rowPtr[i+2]);
	 //cout<<"222222222222"<<endl;
	bool  flagHSV= color.isGreen();
	//flagHSV=color.isThisColorbyHSV(this->color);
	/* bool flagRGB=color.isThisColor(this->color);
	 bool flagHSV=color.isThisColorbyHSV(this->color);*/
 	 if(!flagHSV)  {count++;}  //delete//
	else {
		/*cout<<"2 color:("<<(int)color.r<<",  "<<(int)color.g<<",  "<<(int)color.b<<endl;
		cout<<"2  hsv:("<<color.HSV[0]<<",  "<<color.HSV[1]<<",  "<<color.HSV[2]<<endl;	
	*/
}
   	/* uint8_t pixelR=rowPtr[i];
	 uint8_t pixelG=rowPtr[i+1];
	 uint8_t pixelB=rowPtr[i+2];
         int BR=0,BG=0;
	 BR=(int)pixelB-(int)pixelR;
 	 BG=(int)pixelB-(int)pixelG;
     if(!(int)pixelB>200&&BG>180&&BR>180)//isWhite( j, i, img))//(
      count++;*/
     //here
     
    }//for(j)
if(count==480&&end0<0.0) {end0=i/3;p[1]=end0;}
if(end0>0) break;
}//for(i)


/*if(i==640)
  p[1]=(float)639;*/

//p[2]=(float)-255;
return p;


}





vector<Vec3f>  rgbCentreDetect(Mat img,int beginCol)
{// return the centre of one point


vector<Vec3f> c;
int begin0=-1;
int end0=-1;
Vec3f p = Vec3f(-1.0,-1.0,-1.0); 
p= rgbDetectCol(img, beginCol);//the return value is the begin and end col of white
//cout<<"p:("<<p[0]<<","<<p[1]<<","<<p[2]<<")";
if(p[0]>0.0&&p[1]>0.0)
 c.push_back(p);
else return c;
Vec3f centre = Vec3f(-1.0,-1.0,-1.0);
int j=0;
//cout<<"eoder"<<endl;
for(j=0;j<480;j++)
 {
  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
  for(int i=p[0]*3;i<(p[1]+1)*3;i+=3)
   {
	//const uint8_t* rowPtr = img.ptr<uint8_t>(j);
	  colorRGB color(rowPtr[i],rowPtr[i+1],rowPtr[i+2]);
//cout<<"333333333333"<<endl;
	bool  flagHSV= color.isGreen();
	//flagHSV=color.isThisColorbyHSV(this->color);
	 /*bool flagRGB=color.isThisColor(this->color);
	 bool flagHSV=color.isThisColorbyHSV(this->color);*/
 	 if(flagHSV) 
         if(begin0<0){ 
	/*cout<<"3  color:("<<(int)color.r<<",  "<<(int)color.g<<",  "<<(int)color.b<<endl;
	cout<<"3  hsv:("<<color.HSV[0]<<",  "<<color.HSV[1]<<",  "<<color.HSV[2]<<endl;	
	*/
	begin0=j;
        centre[0]=(float)begin0;
	 break;  }//delete//
	
/* uint8_t pixelR=rowPtr[i];
	 uint8_t pixelG=rowPtr[i+1];
	 uint8_t pixelB=rowPtr[i+2];
 	 int BR=(int)pixelB-(int)pixelR;
 	 int BG=(int)pixelB-(int)pixelG;
     if((int)pixelB>200&&BG>180&&BR>180)//isWhite( j, i, img))//
	{
	if(begin0<0)
 	   { 
 		begin0=j;
		centre[0]=(float)begin0;
   		break; 

	   }
 
	}
*/

   }//for(i)


if(begin0>0.0) break;
 }//for(j)
if(j==480){/*c.push_back(centre);*/return c;  }
/**/
//cout<<"youleyigezhi"<<endl;
for(  j=centre[0];j<480;j++)
 {int count=0;
  const uint8_t* rowPtr = img.ptr<uint8_t>(j);
   
 for(int i=p[0]*3;i<(p[1]+1)*3;i+=3)
   {
  	const uint8_t* rowPtr = img.ptr<uint8_t>(j);
	  colorRGB color(rowPtr[i],rowPtr[i+1],rowPtr[i+2]);
//cout<<"4444444444444"<<endl;
	bool  flagHSV= color.isGreen();
	//flagHSV=color.isThisColorbyHSV(this->color);
	/* bool flagRGB=color.isThisColor(this->color);
	 bool flagHSV=color.isThisColorbyHSV(this->color);*/
 	 if(!flagHSV) count++;//delete//
	 else 
	{
	/*cout<<"4 color:("<<(int)color.r<<",  "<<(int)color.g<<",  "<<(int)color.b<<endl;
	cout<<"4  hsv:("<<color.HSV[0]<<",  "<<color.HSV[1]<<",  "<<color.HSV[2]<<endl;		
	*/}	
	/* uint8_t pixelR=rowPtr[i];
	 uint8_t pixelG=rowPtr[i+1];
	 uint8_t pixelB=rowPtr[i+2];
 	 int BR=(int)pixelB-(int)pixelR;
 	 int BG=(int)pixelB-(int)pixelG;

	 if(!((int)pixelB>200&&BG>180&&BR>180))//isWhite( j, i, img))//
  	 {
	 	count++;
 
  	 }*/

	}//for(i)
	if(count==p[1]-p[0]+1&&end0<0.0)
 	  {
	  end0=j;
	  centre[1]=(float)end0;
	  break;
	   }

	if(end0>0) break;
 }//for(j)
if(begin0>0&&end0>0)
	c.push_back(centre);
//cout<<"c.size():"<<c.size()<<endl;
//if(c.size()>=2)
//cv::circle(img,cv::Point((c[0][0]+c[0][1])/2,(c[1][0]+c[1][1])/2),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
return c;
}//end


vector<Vec3f> rgbCentre(Mat img1)
{
      Vec3f p=Vec3f(-1.0,-1.0,-1.0);
      vector<Vec3f> centre;
      vector<Vec3f> centreL=  rgbCentreDetect( img1,0);//from the first col begin to scan the value 255,every point have the upleft and downright
      vector<Vec3f>  centreR;
       if(centreL.size()>=2)
       { 
         centreR=  rgbCentreDetect( img1,centreL[0][1]+1);//centreR contain two value (colbegin,colend) (rowbegin,rowend),the second in the first picture;
       }
	if(centreR.size()>=2)
      {
	p[1]=(centreL[1][0]+centreL[1][1])/2;//第幾行表示縱座標
	p[0]=(centreL[0][0]+centreL[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
	centre.push_back(p);
	p[1]=(centreR[1][0]+centreR[1][1])/2;//第幾行表示縱座標
	p[0]=(centreR[0][0]+centreR[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
 

       }
return centre;//

}





vector<Vec3f> rgbCentre(Mat img1,Mat img2)
{
//return four points of the two images
      Vec3f p=Vec3f(-1.0,-1.0,-1.0);
      vector<Vec3f> centre;
      vector<Vec3f> centreL=  rgbCentreDetect( img1,0);//from the first col begin to scan the value 255,every point have the upleft and downright
      vector<Vec3f>  centreR;
       if(centreL.size()>=2)
       { 
         centreR=  rgbCentreDetect( img1,centreL[0][1]+1);//centreR contain two value (colbegin,colend) (rowbegin,rowend),the second in the first picture;
       }

      if(centreR.size()>=2)
      {
	p[1]=(centreL[1][0]+centreL[1][1])/2;//第幾行表示縱座標
	p[0]=(centreL[0][0]+centreL[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0);   
	centre.push_back(p);
	p[1]=(centreR[1][0]+centreR[1][1])/2;//第幾行表示縱座標
	p[0]=(centreR[0][0]+centreR[0][1])/2;//第幾列表示橫座標
	cv::circle(img1,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
/*the two point of the first picture*/

       }


      vector<Vec3f>  centreRR;
      vector<Vec3f> centreRL=  rgbCentreDetect( img2,0);
       if(centreRL.size()>=2)
       { 
       centreRR=  rgbCentreDetect(img2,centreRL[0][1]+1);

       }


      if(centreRR.size()>=2)
      {
	p[1]=(centreRL[1][0]+centreRL[1][1])/2;
	p[0]=(centreRL[0][0]+centreRL[0][1])/2;
	cv::circle(img2,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
	p[1]=(centreRR[1][0]+centreRR[1][1])/2;
	p[0]=(centreRR[0][0]+centreRR[0][1])/2;
	cv::circle(img2,cv::Point(p[0],p[1]),3, cv::Scalar(0, 0, 0), -1, 8, 0); 
	centre.push_back(p);
/*the two point of the second picture*/
       }


return centre;//every value present a point
        
}











/*返回值是一個圓心半徑的數組每個數組裏面有三個值分別表示(x,y,r)*/
  vector<Vec3f>  circleDetect(Mat img)
    {//檢測Mat圖像上的圓形並且將其圓心半徑的三維數組給出來
   vector<Vec3f> circles;// circles; 
    
   GaussianBlur(img, img, Size(7, 7), 2, 2);
 
   HoughCircles(img, circles,CV_HOUGH_GRADIENT,1.5, 10, 200, 100, 0, 0); 
   int len=circles.size()-1;
   for(int i=len;i>0;i--)
     if(abs(circles[i][0]-circles[i-1][0])<20||abs(circles[i][1]-circles[i-1][1])<20)
       {
        Vec3f p=circles[circles.size()-1];
        circles[circles.size()-1]=circles[i];
        circles[i]=p;

        circles.pop_back();
         if(i!=circles.size())
           i++;
        }
   if(circles.size()!=0)
     for(int i=0;i<circles.size();i++)
     circle(img, Point(circles[i][0],circles[i][1]), 3, Scalar(255, 255, 255), -1, 8, 0);   

 /*CvFont font; 
    double hscale = 0.5;
    double vscale = 0.4;
    int linewidth = 0.5;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX |CV_FONT_ITALIC,hscale,vscale,0,linewidth);
    CvScalar textColor =cvScalar(0,0,255);
    char num='a'; 
   // char num='a'; 
   for(int i=0;i<circles.size();i++)
         {
          CvPoint center =cvPoint(cvRound(circles[i][0]), cvRound(circles[i][1]));
          cvPutText(img, &num, center, &font,cvScalar(255,255,255));
          num++;
         }
   */
return circles;
    }


vector<vector<float> > resultXYZ(vector<Vec3f> circle , vector<Vec3f> circleR) 
{     
       float T=0.0;
       vector<vector<float> > re;
       Point3f p(-640.0,-640.0,-640.0);
       int len=circle.size();
       int lenR=circleR.size();
       //cout<<"circle&&R"<<endl;
       if(len==2&&lenR==2)
       {//如果到檢測到了圓心
       vector<pair<Point,Point> > matches;//存放的是相互匹配的點對，一共兩對，每隊中是一對左右圖像上相互匹配的圖像
       Point left,right;
       int first,second;
       if(circle[0][0]<circle[1][0])   
         first=0;
       else first=1;
        if(circleR[0][0]<circleR[1][0])   
         second=0;
       else second=1;
       

       left.x=circle[first][0];
       left.y=circle[first][1];
       //cout<<"left:"<<left<<endl;;
       right.x=circleR[second][0];
       right.y=circleR[second][1];
       //cout<<"right:"<<right<<endl;;
       matches.push_back(std::pair<Point,Point>(left,right));
       
       left.x=circle[1-first][0];
       left.y=circle[1-first][1];
       //cout<<"left:"<<left<<endl;;
       right.x=circleR[1-second][0];
       right.y=circleR[1-second][1];
       //cout<<"right:"<<right<<endl;;
       matches.push_back(std::pair<Point,Point>(left,right));
       cout<< "matches[0]:("<< matches[1].first.x<<" ,"<<matches[1].first.y<<"),("<<matches[1].second.x<<","<<matches[1].second.y<<")"<<endl;
       cout<< "matches[1]:("<< matches[0].first.x<<" ,"<<matches[0].first.y<<"),("<<matches[0].second.x<<","<<matches[0].second.y<<")"<<endl;
       //cout<<"matches.size():"<<matches.size()<<endl;

	

	cout<< "changed  matches[0]:("<< matches[1].first.x<<" ,"<<matches[1].first.y<<"),("<<matches[1].second.x<<","<<matches[1].second.y<<")"<<endl;
        cout<< "changed matches[1]:("<< matches[0].first.x<<" ,"<<matches[0].first.y<<"),("<<matches[0].second.x<<","<<matches[0].second.y<<")"<<endl; 	
 	/*change matches*/

	
       vector<float> z=cal.calZ(matches);//matches
     cal.T=0.0;



       if(z.size()>0)
   

       // for(int i=0;i<z.size()-1;i++)
      {
       p.z=z[0];
       vector<float>  x=cal.calX( z);//,matches
  
       vector<float> y=cal.calY( z);//,matches   
       p.x=x[0];
       p.z=z[0];    
       p.y=y[0];
       cout<<"p[0"<<"]:"<<p<<endl;
       re.push_back(x);
       re.push_back(y);
       re.push_back(z);
//p=Point3f(re[0][0],re[1][0],re[2][0]);
       p.x=x[1];
       p.z=z[1];    
       p.y=y[1];
       cout<<"p[1]:"<<p<<endl;

}


       }

   // else {cout<<"lenL:"<<len<<"  lenR"<<lenR<<endl;}

 return re;//p;

}



}; 
