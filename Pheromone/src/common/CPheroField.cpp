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
void CPheroField::wind(int x,int y)
{
    int windX = x;
    int windY = y;
    if (diffusion > 0.0){
		//create an openCV structure
                float* dataWind = (float*)calloc(size,sizeof(float));// An array containing wind speed
                float v = 0.5; // coefficient multiplied to the wind matrix
                float* dataTemp = (float*)calloc(size,sizeof(float)); //Temporary data to apply wind effect and diffusion seperately
                float* dataTemp2 = (float*)calloc(size,sizeof(float)); //Temporary data of 1 - wind matrix
		cv::Mat mat = cv::Mat(height,width,CV_32F,data);
                cv::Mat matTemp = cv::Mat(height,width,CV_32F,dataTemp); //  Temporary matrix which contains some proportion of the pheromone data and it will be multiplied by the wind matrix. (only some of pheromone is moving by wind)
                cv::Mat matWind = cv::Mat(height,width,CV_32F,dataWind); // Wind matrix containing how fast the pheromone will be shifted
                cv::Mat one = cv::Mat::ones(height,width,CV_32F); // a matrix size of arena containing ones.
                cv::Mat RestWind = cv::Mat(height,width,CV_32F,dataTemp2); // a matrix of (1 - wind) - the proportion of pheromone will be diffused
		bool over = false;
                for (int j = 0; j < width; j++) {
                    for (int i = 0; i < height; i++) {
                        if (mat.at<float>(i,j) == 0) {
                            matWind.at<float>(i,j) = 0; //pixel with 0 is not moving
                        }
                        else {
                            matWind.at<float>(i,j) = 2-pow(2,(mat.at<float>(i,j)/512));//1-(0.9/255)*(mat.at<float>(i,j));2-pow(3,(mat.at<float>(i,j)/512));// tried (1/mat.at<float>(i,j) but it is not giving the effect I expected. This part is what I am still working on
			   // if (matWind.at<float>(i,j) > 1) matWind.at<float>(i,j) = 1;
			  //  if (matWind.at<float>(i,j) < 0) matWind.at<float>(i,j) = 0;
                        }
                    }
                }
                // multiply the scalar v to matWind
                matWind = matWind*v;
                
                // subtract 1 by matWind so that mat preserves non-shifted pheromone without any change in the amount
                RestWind = one - matWind;
                
                // multiply mat by matWind so that the amount of pheromone will be moved is fixed
                matTemp = mat.mul(matWind);
                
                // the rest of pheromone remain.
                mat = mat.mul(RestWind);
                
		//perform blur
		cv::GaussianBlur( mat, mat, cv::Size( 11, 11 ), 3, 3 );
                
                //matSplit = v*mat; // Spliting mat into two with the wind coefficient (how much pheromone will be moved)
                //mat = (1-v)*mat; // diffused pheromone from the original position - scalar v multiplication
                cv::Mat trans = (cv::Mat_<double>(2,3) << 1, 0, windX, 0, 1, windY); // transformation matrix
                warpAffine(matTemp,matTemp,trans,matTemp.size()); //warp
                
                //add shifted matrix and remaining matrix so that total amount of pheromone is same.
		mat = mat + matTemp;
                 for (int j = 0; j < width; j++) {
                    for (int i = 0; i < height; i++) {
                        if (mat.at<float>(i,j) > 255) {
                            mat.at<float>(i,j) = 255 ; //if any data value is greater than the maximum value, it is limited to the maximum value.
                        }
                        
                    }
                }
                //deallocate the memory
                free(dataWind);
                free(dataTemp);
                free(dataTemp2);
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
