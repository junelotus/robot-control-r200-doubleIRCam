#include "colorRGB.h"
#include <iostream>
#include <vector>
using namespace std;
const colorRGB colorRGB::red(255,0,0);
const colorRGB colorRGB::green(0,255,0);
const colorRGB colorRGB::blue(0,0,255);

bool colorRGB::isRed(colorRGB color)
{

if(color==colorRGB::red)
 return true;
return false;

}

bool colorRGB::isRed(vector<float > HSV)
{

if(HSV[0]>=-30&&HSV[0]<30)
 return true;
return false;

}

bool colorRGB::isGreen()
{
int r=(int)this->r;
int g=(int)this->g;
int b=(int)this->b;
if(g-r>100&&g-b>100&&g>150)
	return true;
return false;
}



bool colorRGB::isGreen(colorRGB color)
{

if(color==colorRGB::green)
 	return true;
return false;

}

bool colorRGB::isGreen(vector< float >  HSV)
{

if(HSV[0]>=90&&HSV[0]<150)
 return true;
return false;

}

bool colorRGB::isGreen(float h)
{
if(h>=90&&h<150)
 return true;
return false;

}

bool colorRGB::isBlue(colorRGB color)
{

if(color==colorRGB::blue)
 return true;
return false;

}

bool colorRGB::isBlue(vector<float> HSV)
{

if(HSV[0]>=210&&HSV[0]<270)
 return true;
return false;

}

/*qingse  150~210;yellow 30~90 pinhong 270~330*/


bool colorRGB::isThisColor(char* name)
{
if(name=="red"&&isRed(*this))//||name=="green"||name=="blue")
 return true;
if(name=="green"&&isGreen(*this))//||name=="green"||name=="blue")
 return true;
if(name=="blue"&&isBlue(*this))//||name=="green"||name=="blue")
 return true;
return false;

}


vector<float> colorRGB::RGBToHSV()
{

vector<float> HSV(3,0.0);

float r=(float)(int)this->r;
float g=(float)(int)this->g;
float b=(float)(int)this->b;
float h=0.0,s=0.0,v=0.0;

v=(r>g?r:g);
float min=(r>g?g:r);
v=v>b?v:b;
HSV[2]=v;

min=min<b?min:b;


if(v!=0.0)
 HSV[1]=(v-min)/v;
else HSV[1]=0.0;

if(v==r)
 HSV[0]=60.0*(g-b)/(v-min);
if(v==g)
 HSV[0]=120.0+60.0*(b-r)/(v-min);
if(v==b)
 HSV[0]=240.0+60.0*(r-g)/(v-min);

if(HSV[0]<0.0)
 HSV[0]+=360.0;
this->HSV=HSV;//set the HSV of the RGB

return HSV;




}


bool colorRGB::isThisColorbyHSV(char* name)
{	
	vector<float> HSV=RGBToHSV();
	
	if(name=="red"&&isRed(this->HSV))//||name=="green"||name=="blue")
	 return true;
	if(name=="green"&&isGreen(this->HSV))//||name=="green"||name=="blue")
	 return true;
	if(name=="blue"&&isBlue(this->HSV))//||name=="green"||name=="blue")
	 return true;
	return false;		


}
