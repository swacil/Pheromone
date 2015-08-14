#include "CRawImage.h"

static unsigned char sth[] = {66,77,54,16,14,0,0,0,0,0,54,0,0,0,40,0,0,0,128,2,0,0,224,1,0,0,1,0,24,0,0,0,0,0,0,16,14,0,18,11,0,0,18,11,0,0,0,0,0,0,0,0,0,0};

CRawImage::CRawImage(int wi,int he)
{
	width =  wi;
	height = he;
	bpp= 3;
	size = bpp*width*height;
	data = (unsigned char*)calloc(size,sizeof(unsigned char));
	memset(header,0,122);
	memcpy(header,sth,122);
	header[18] = width%256;
	header[19] = width/256;
	header[22] = height%256;
	header[23] = height/256;
	header[2] = (size+122)%256;
	header[3] = ((size+122)/256)%256;
	header[4] = ((size+122)/256/256)%256;
	header[5] = ((size+122)/256/256)/256;
	header[34] = (size)%256;
	header[35] = ((size)/256)%256;
	header[36] = ((size)/256/256)%256;
	header[37] = ((size)/256/256)/256;
	header[10] = 122;
	numSaved = 0;
	ownData = true;
}

CRawImage::CRawImage(unsigned char *datai,int wi,int he)
{
	ownData = false;
	width =  wi;
	height = he;
	bpp= 3;
	size = bpp*width*height;
	data = datai; 
	memset(header,0,122);
	memcpy(header,sth,122);
	header[18] = width%256;
	header[19] = width/256;
	header[22] = height%256;
	header[23] = height/256;
	header[2] = (size+54)%256;
	header[3] = ((size+54)/256)%256;
	header[4] = ((size+54)/256/256)%256;
	header[5] = ((size+54)/256/256)/256;
	header[34] = (size)%256;
	header[35] = ((size)/256)%256;
	header[36] = ((size)/256/256)%256;
	header[37] = ((size)/256/256)/256;
	header[10] = 122;
	numSaved = 0;
}

void CRawImage::generate(CPheroField *phero,int chan)
{
	float *phe = phero->data;
	chan = chan%7;
//	int chana = chan%3;
	for (int i = 0;i<size;i+=3){
		 data[i]=data[i+1]=data[i+2]=fmin(phe[i/3],255);
	}
	/*if (chan > 2){
		chana = (chan+1)%3;
		for (int i = 0;i<size;i+=3){
				if (phe[i/3] > 0)  data[i+chana]=fmin(phe[i/3],255);
		}
	}
	if (chan > 6){
		chana = (chan+2)%3;
		for (int i = 0;i<size;i+=3){
			if (phe[i/3] > 0) data[i+chana]=fmin(phe[i/3],255);
		}
	}*/
}

int CRawImage::getSaveNumber()
{
	char name[100];
	FILE* file = NULL;
	do{
		sprintf(name,"%04i.bmp",numSaved++);
		file = fopen(name,"r");
	}
	while (file != NULL);
	numSaved--;
	return numSaved;
}

CRawImage::~CRawImage()
{
	if (ownData) free(data);
}

void CRawImage::swap()
{
  unsigned char* newData = (unsigned char*)calloc(size,sizeof(unsigned char));
  int span = width*bpp;
  for (int j = 0;j<height;j++){
	  memcpy(&newData[span*j],&data[span*(height-1-j)],span);
	  for (int i = 0;i<width;i++){
		  char a = newData[(width*j+i)*3]; 
		  newData[(width*j+i)*3] = newData[(width*j+i)*3+2];
		  newData[(width*j+i)*3+2] = a; 
	  }
  }
  memcpy(data,newData,size);
  free(newData);
}

void CRawImage::saveBmp(const char* inName)
{
	FILE* file = fopen(inName,"wb");
	swap();
	fwrite(header,54,1,file);
	fwrite(header,54,1,file);
	fwrite(header,14,1,file);
	fwrite(data,size,1,file);
	swap();
	printf("Saved size %ix%i - %i.\n",size,width,height);
	fclose(file);
}

void CRawImage::saveBmp()
{
	char name[100];
	sprintf(name,"images/%06i.bmp",numSaved++);
	saveBmp(name);
}

bool CRawImage::loadBmp(const char* inName)
{
	FILE* file = fopen(inName,"rb");
	if (file!=NULL)
	{
		fread(data,54,1,file);
		bpp = 3;
		memcpy(header,data,54);
		int headerWidth = header[18]+header[19]*256;
		int headerHeight = header[22]+header[23]*256;
		if (ownData && (headerWidth != width || headerHeight != height)){
			free(data);
			height = headerHeight;
			width = headerWidth;
			size = height*width*bpp;
			data = (unsigned char*)calloc(size,sizeof(unsigned char));
		}
		int offset = header[10]+header[11]*256;
		fread(data,offset-54,1,file);
		fread(data,size,1,file);
		fclose(file);
		swap();
		return true;
	}
	return false;
}

void CRawImage::plotCenter()
{
	int centerWidth = 20;
	unsigned char color[] = {255,150,150};
	for (int i = -centerWidth;i<centerWidth;i++){
		for (int j =0;j<3;j++){
			data[(width*(height/2+i)+width/2-centerWidth)*3+j] = color[j];
			data[(width*(height/2+i)+width/2+centerWidth)*3+j] = color[j];
			data[(width*(height/2-centerWidth)+width/2+i)*3+j] = color[j];
			data[(width*(height/2+centerWidth)+width/2+i)*3+j] = color[j];
		}
	}
}

void CRawImage::plotLine(int x,int y) {
	int base;
	if (y < 0 || y > height-1) y = height/2;
	if (x < 0 || x > width-1) x = width/2;
	for(int i=0; i < width;i++) {
		if (i == width/2) i++;
		base = (width*y+i)*3;
		data[base+0] = 255;
		data[base+1] = 0;
		data[base+2] = 255;
	}

	for(int j=0;j<height;j++) {
		const int bidx = ((width*j)+x)*3;
	if (j == height/2) j++;
		data[bidx+0] = 255;
		data[bidx+1] = 255;
		data[bidx+2] = 0;
	}
}

void CRawImage::displayCalibration()
{
	int radius = 40;
	int pos = 0;
	int iix,iiy;
	int oX = 0;
	int oY = 0;
	int xx[] = {radius+oX,radius+oX,width-radius-oX,width-radius-oX};
	int yy[] = {radius+oY,height-radius-oY,radius+oY,height-radius-oY};
	for (int corner = 0;corner<4;corner++)
	{
		int x = xx[corner]; 
		int y = yy[corner]; 
		for (int ix = -radius;ix<radius+1;ix++)
		{
			iix = ix +x;
			for (int iy = -radius;iy<radius+1;iy++)
			{
				iiy = iy +y;
				if (iix >= 0 && iix < width && iiy >=0 && iiy < height)
				{
					pos = 3*(iix+iiy*width);
					float dist = sqrt(ix*ix+iy*iy);
					if (dist < radius && dist > radius*0.75 || dist < radius * 0.25) data[pos]=data[pos+1]=data[pos+2]=255;
				}
			}
		}
	}
}

void CRawImage::displayRobotFull(int x, int y,int phi,int id)
{
	int radius = 80;
	int iix,iiy;
	int pos = 0;
	for (int ix = -radius;ix<radius+1;ix++)
	{
		iix = ix +x;
		for (int iy = -radius;iy<radius+1;iy++)
		{
			iiy = iy +y;
			if (iix >= 0 && iix < width && iiy >=0 && iiy < height)
			{
				pos = 3*(iix+iiy*width);
				float dist = sqrt(ix*ix+iy*iy);
				if (dist < radius) data[pos]=data[pos+1]=data[pos+2]=id;
			}
		}
	}
}

void CRawImage::displayRobot(int x, int y,int phi,int id)
{
	int radius = 40;
	int pos = 0;
	int iix,iiy;
	for (float i  = 0;i<6.28;i+=0.01)
	{
		iix = x+radius*sin(i);
		iiy = y+radius*cos(i);

		if (iix >= 0 && iix < width && iiy >=0 && iiy < height)
		{
			pos = 3*(iix+iiy*width);
			data[pos]=data[pos+1]=data[pos+2]=128;
		}
	}
}

/** pocita jas obrazku:
  *  upperHalf == true, pocita se jen z horni poloviny obrazku
  *  upperHalf == false, pocita jen ze spodni poloviny obrazku
  */
double CRawImage::getOverallBrightness(bool upperHalf) {
	int step = 5;
	int sum,num,satMax,satMin,pos;
	sum=num=satMax=satMin=0;
	int limit = 0;
	if (upperHalf) limit = 0; else limit=height/2;
	for (int i = limit;i<height/2+limit;i+=step){
		for (int j = 0;j<width;j+=step){
			pos = (i*width+j)*bpp;
			if (data[pos] >= 250 && data[pos+1] >=250 && data[pos+2] >= 250) satMax++;  
			if (data[pos] <= 25 && data[pos+1] <=25 && data[pos+2] <= 25) satMin++;
			sum+=data[pos] + data[pos+1] + data[pos+2];
			num++;
		}
	}
	return (sum/num/bpp) + (satMax-satMin)*100.0/num;
}




