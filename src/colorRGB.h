#pragma once 
#include <iostream>
#include <vector>
using namespace std;

class colorRGB{

public :
	uint8_t r;
	uint8_t g;
	uint8_t b;
	vector<float> HSV;
	static const colorRGB red;
	static const colorRGB green;
	static const colorRGB blue;
	colorRGB(){}
	colorRGB(uint8_t r,uint8_t g,uint8_t b):r(r),g(g),b(b){}
	bool isRed(colorRGB color);
	bool isRed(vector<float> HSV);
	bool isGreen();
	bool isGreen(colorRGB color);
	bool isGreen(vector<float> HSV);
	bool isGreen(float h);
	bool isBlue(colorRGB color);
	bool isBlue(vector<float> HSV);
	bool isThisColor(char* name);
	bool isThisColorbyHSV(char* name);
	
	vector<float> RGBToHSV();
	
	bool operator==(colorRGB color0)
	{//scale

	if(this->r>200&&abs(this->g-color0.g)>180&&abs (this->b-color0.b)>180)
 		return true;

	else 
 		return false;

}




};

