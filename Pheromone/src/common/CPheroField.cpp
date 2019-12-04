#include "CPheroField.h"

CPheroField::CPheroField(int wi,int he,float evapor,float diffuse,float influ)
{
	evaporation = evapor;
	diffusion = diffuse;
	influence = influ;
	lastX = (int*)calloc(MAX_ID,sizeof(float));
	lastY = (int*)calloc(MAX_ID,sizeof(float));
	width =  wi;
	height = he;
	size = width*height;
	data = (float*)calloc(size,sizeof(float));
         
}

CPheroField::~CPheroField()
{
	free(lastX);
	free(lastY);
	free(data);
}

void CPheroField::clear()
{
	memset(data,0,size*sizeof(float));
}

void CPheroField::addTo(int x, int y,int id,int num,int radius)
{
	id = id%MAX_ID;
	int dx = x-lastX[id];
	int dy = y-lastY[id];
	if (fabs(dx) > 100 || fabs(dy) > 100) {
		lastX[id] = x;
		lastY[id] = y;
		dx = dy = 0; 
	}
	int sx,ex,sy,ey;
	float ix,iy;
	if (fabs(dx) > fabs(dy)){
		if (dx > 0){
			sx = lastX[id];
			ex = x;
			iy = lastY[id];
		}else{
			sx = x;
			ex = lastX[id];	
			iy = y;
		}
		float f = (float)dy/(float)dx;
		for (int ix = sx;ix<=ex;ix++)
		{
			add(ix,iy,id,num,radius);
			iy+=f;
		}
	}else{
		if (dy > 0){
			sy = lastY[id];
			ey = y;
			ix = lastX[id];
		}else{
			sy = y;
			ey = lastY[id];
			ix = x;
		}
		float f = (float)dx/(float)dy;
		for (int iy = sy;iy<=ey;iy++)
		{
			add(ix,iy,id,num,radius);
			ix+=f;
		}

	}

	lastX[id] = x;
	lastY[id] = y;
}


float CPheroField::get(int x, int y)
{
	if (x > 0 && y >0 && x<width && y<height) return data[x+y*width];
	return -1;
}
void CPheroField::circle(int x, int y,int id,int num,int radius)
{
    id = id%MAX_ID;
    int pos = 0;
    int iix,iiy;
    for (int ix = -radius;ix<radius+1;ix++){
		iix = ix +x;
		for (int iy = -radius;iy<radius+1;iy++){
			iiy = iy +y;
			if ((ix*ix)+(iy*iy)<=radius*radius){
				pos = (iix+iiy*width);
				data[pos] += num;
			}
		}
	}
}

//MSc Project
//shareGradCircle draws a circle with a small blur on the rim of the circle. radius refers to the radius of the circle including blurry part and radius2 refers to the radius of the circle without blur. 
// radius > radius2
void CPheroField::sharpGradCircle(int x, int y,int id,int num,int radius, int radius2)
{
    id = id%MAX_ID;
    int pos = 0;
    int iix,iiy;
    for (int ix = -radius;ix<radius+1;ix++){
		iix = ix +x;
		for (int iy = -radius;iy<radius+1;iy++){
			iiy = iy +y;
			if ((ix*ix)+(iy*iy)<=radius*radius){
                            if ((ix*ix)+(iy*iy)<=radius2*radius2){
				pos = (iix+iiy*width);
				data[pos] += num;
                            }
                            else {
                                pos = (iix+iiy*width);
                                data[pos] += -num/(radius-radius2) * (sqrt((ix*ix)+(iy*iy))-radius);
                            }
			}
		}
	}
}

void CPheroField::Gracircle(int x, int y,int id,int num,int radius)
{
    id = id%MAX_ID;
    int pos = 0;
    int iix,iiy;
    int lastradius=0;
    int kr=0;
    for (int kp=0;kp<radius+1;kp=kp+1)
    {
      kr=num*1.82-kp*1.5;
      for (int ix = -kp;ix<kp+1;ix++){
		iix = ix +x;
		for (int iy = -kp;iy<kp+1;iy++){
			iiy = iy +y;
			if ((ix*ix)+(iy*iy)<=kp*kp && (ix*ix)+(iy*iy)>=lastradius*lastradius){
				pos = (iix+iiy*width);
				data[pos] = kr;
			}
		}
	}
     lastradius=kp;
    }
}
//

void CPheroField::rectangle(int x, int y,int id,int num,int a,int b)
{
    id = id%MAX_ID;
    int pos = 0;
    int iix,iiy;
    for (int ix = -a;ix<a+1;ix++){
		iix = ix +x;
		for (int iy = -b;iy<b+1;iy++){
			iiy = iy +y;
			if (ix >= -(a/2) && ix < (a/2) && iy >= -(b/2) && iy < (b/2)){
				pos = (iix+iiy*width);
                                data[pos] += num;
			}
		}
	}
}
    
void CPheroField::add(int x, int y,int id,int num,int radius)
{
	id = id%MAX_ID;
	int pos = 0;
	int iix,iiy;
	for (int ix = -radius;ix<radius+1;ix++){
		iix = ix +x;
		for (int iy = -radius;iy<radius+1;iy++){
			iiy = iy +y;
			if (iix >= 0 && iix < width && iiy >=0 && iiy < height){
				pos = (iix+iiy*width);
				data[pos] += num*(1-fmin(sqrt(ix*ix+iy*iy)/radius,1));
			}
		}
	}
	lastX[id] = x;
	lastY[id] = y;
}
void CPheroField::diff(int x, int y)
{
    int kernelSize = x;
    int sigma = y;
    cv::Mat mat = cv::Mat(height,width,CV_32F,data);
    cv::GaussianBlur( mat, mat, cv::Size( kernelSize, kernelSize ), sigma, sigma );
}
    
void CPheroField::wind(float x,float y)
{
    float windX = x;
    float windY = y;
    if (diffusion > 0.0){
                float timex = timer.getTime();
		//create an openCV structure
                //float* dataWind = (float*)calloc(size,sizeof(float));// An array containing wind speed
                //float v = 0.5; // coefficient multiplied to the wind matrix
                float* gradX = (float*)calloc(size,sizeof(float)); //Gradient of pheromone in x direction
                float* gradY = (float*)calloc(size,sizeof(float)); //Gradient of pheromone in y direction
		cv::Mat mat = cv::Mat(height,width,CV_32F,data);
                cv::Mat matX = cv::Mat(height,width,CV_32F,gradX); // grad X matrix
                cv::Mat matY = cv::Mat(height,width,CV_32F,gradY); // grad Y matrix
                //cv::Mat matWind = cv::Mat(height,width,CV_32F,dataWind); // Wind matrix containing how fast the pheromone 
                cv::Mat kernelX = (cv::Mat_<float>(1,3) << -1.0, 0.0, 1.0);
                cv::Mat kernelY = (cv::Mat_<float>(3,1) << -1.0, 0.0, 1.0);
                cv::filter2D(mat,matX,-1,kernelX,cv::Point_<int>(-1,-1),0.0);
                cv::filter2D(mat,matY,-1,kernelY,cv::Point_<int>(-1,-1),0.0);
                
                //matX = 0.5 * matX
                //matY = 0.5 * matY
                
                for (int j = 0; j < width; j++) {
                    for (int i = 0; i < height; i++) {
                        if (mat.at<float>(i,j) == 0) {
                            mat.at<float>(i,j) = 0; //pixel with 0 is not moving
                        }
                        else {
                            mat.at<float>(i,j) -= (windX*matX.at<float>(i,j)+windY*matY.at<float>(i,j))*timex/1000000.0;
                        }
                    }
                }
                for (int j = 0; j < width; j++) {
                for (int i = 0; i < height; i++) {
                        if (mat.at<float>(i,j) > 255) {
                            mat.at<float>(i,j) = 255 ; //if any data value is greater than the maximum value, it is limited to the maximum value.
                        }
                        
                    }
                }
                
		//perform blur
		cv::GaussianBlur( mat, mat, cv::Size( 3, 3 ), 1, 1 ); //11 3 for max vel circle experiment //3 1.1 maxrec
                
                
                //add the shifted matrix and remaining matrix so that total amount of pheromone is same.
                //deallocate the memory
                free(gradX);
                free(gradY);
                //free(dataTemp2);
		/*float diffuse = pow(2,-timex/1000000.0/diffusion);
		for (int i = width+1;i<size;i++){
			if (i%width == 0) i++;
			diffH = (data[i-1] - data[i])/2;
			data[i] += diffH*(1-diffuse);
			diffV = (data[i-width] - data[i])/2;
			data[i] += diffV*(1-diffuse);
			data[i-1] -= diffH*(1-diffuse);
			data[i-width] -= diffV*(1-diffuse);
		}*/
                timer.reset();
                timer.start();
	}
}
    
void CPheroField::recompute()
{
	float timex = timer.getTime();
	float decay = pow(2,-timex/1000000.0/evaporation);
	//int diffV,diffH; // declaration of diffusion constants for each axe
	for (int i = 0;i<size;i++) data[i]=data[i]*decay;
	
	//printf("Recompute took %.0f %f\n",timer.getTime()-timex,diffuse);
	timer.reset();
	timer.start();
}
void CPheroField::measure(int x, int y,int z)
{
    if (data[x+y*width] <= z ) {
        int pos = 0;
        int iix,iiy;
        for (int ix = -10;ix<11;ix++){
		iix = ix +x;
		for (int iy = -10;iy<11;iy++){
			iiy = iy +y;
			if ((ix*ix)+(iy*iy)<=100){
				pos = (iix+iiy*width);
				data[pos] = 0;
			}
		}
	}
    }
}
