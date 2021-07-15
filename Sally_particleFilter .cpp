//Name:Sally Manasrah ID:206644
/*
The program assume that the robot environment 
is a circle area with origin x,y=(0,0) and radius 10
the circle has two inner circles , the first one is 
at (x,y)=(0,0) and radius 5, the second one is at 
(x,y)=(0,0) and radius 2.5.
The robot sense the distance far a way from the origin and return 
sensor information indicate his position either in circle 1 (c1 which has radius 2.5) , or 
in circle 2 (which has radius 5), or circle 3( which has radius 10). 
The particle number is 500 and each particle weight is being updated depend on sensor information(z),
and the particle position is being updated depend on robot movement (dist).
Each time the program run , request from the user to enter the movement or rotation value , when 
we need to just move it forword we enter value 0 for Theta , and when we need it to just rotate
we enter a value 0 for dist(distance), the program return the robot location without particle filter, 
and also mean x , mean y ,var x , var y , in order to compare these values with the robot location without the 
 algorithm.
*/
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <string>
#include <vector>
#include <random>

using namespace std;
int size=500;//number of particles
double meanX=0,meanY=0;

//random function used in the Constructor to Initilaize particles locations
double uniform() {
      return (double)rand() / RAND_MAX;
   }
   
class Particle
{
    private:
    int particleNO;
    static int current_no;
    double x;
    double y;
    double angle;
    double weight;
    
    public:

    // Setter
    
    void setX(double x) {
     this->x =x;
    }
    void setY(double y) {
     this->y =y;
    }
    void setAngle(double a) {
     this->angle = a;
    }
    void setWeight(double w) {
      this->weight = w;
    }
    void setNo(int n) {
      this->particleNO = n;
    }
    // Getter
	
    double getX() {
      return x;
    }
    double getY() {
      return y;
    }
    double getAngle() {
      return angle;
    }
    double getWeight() {
      return weight;
    }
	int getNo() {
      return particleNO;
    }
//Constructor for particles initialization
Particle()
   {  
      particleNO=current_no++;
	  //Generate random point(particles) inside the circle which has radius 10 
      double theta = 2 * 3.14159265358979323846264 * uniform();
      double r = sqrt(uniform());
      x=0 + r * 10 * cos(theta);//radius equal 10 , and the center x,y is (0,0)
      y=0 + r * 10 * sin(theta);
      angle=std::fmod(rand(),3.14);//random theta 
      weight= 1.0/(size*1.0);//set weight to (1/number of particles), in order to have equal weight in the initialization
      
   }  
//Update location(x,y) when robot move forward
void moveForward(double dist) {
       
       double xNew=this->getX()+dist;
       this->setX(xNew);
   }
//Update theta when robot rotate
void rotate(double angle){
      angle=angle*(3.14/180);
      double angleNew=std::fmod((this->getAngle()+angle),3.14);
      this->setAngle(angleNew);
   }
   
//Update particles weight depend on sensor information
void update_Weight(string z){
    //Define sensor probabilities
    double C1GivenC1=0.7;
    double C2GivenC1=0.15;
	double C3GivenC1=0.15;
	
	double C1GivenC2=0.15;
    double C2GivenC2=0.7;
	double C3GivenC2=0.15;
	
	double C1GivenC3=0.15;
    double C2GivenC3=0.15;
	double C3GivenC3=0.7;
	
    double xLOC=this->getX();
    double yLOC=this->getY();
    
	double circle1Radius=2.5;
	double circle2Radius=5.0;
	double circle3Radius=10.0;
	double center_x=0;
	double center_y=0;
	
	//update weight of particles depends on sense function return value 
    if (z=="c1")
       if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle1Radius * circle1Radius)
       this->setWeight(C1GivenC1);
       else if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle2Radius * circle2Radius)
       this->setWeight(C1GivenC2);
       else if ((xLOC - center_x) * (xLOC - center_x) +
        (yLOC - center_y) * (yLOC - center_y) <= circle3Radius * circle3Radius)
       this->setWeight(C1GivenC3);
       else this->setWeight(0);
    else if (z=="c2")
       if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle1Radius * circle1Radius)
       this->setWeight(C2GivenC1);
       else if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle2Radius * circle2Radius)
       this->setWeight(C2GivenC2);
       else if ((xLOC - center_x) * (xLOC - center_x) +
        (yLOC - center_y) * (yLOC - center_y) <= circle3Radius * circle3Radius)
       this->setWeight(C2GivenC3);
       else this->setWeight(0);
    else if (z=="c3")
       if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle1Radius * circle1Radius)
       this->setWeight(C3GivenC1);
       else if ((xLOC - center_x) * (xLOC - center_x) +
       (yLOC - center_y) * (yLOC - center_y) <= circle2Radius * circle2Radius)
       this->setWeight(C3GivenC2);
       else if ((xLOC - center_x) * (xLOC - center_x) +
        (yLOC - center_y) * (yLOC - center_y) <= circle3Radius * circle3Radius)
       this->setWeight(C3GivenC3);
       else this->setWeight(0);
    else 
       this->setWeight(0);
   }
};
int Particle::current_no = 0;

//Sense function depend on robot location 
string sense (Particle& p){
    double xLOC=p.getX();
    double yLOC=p.getY();
	//Determine robot location ( circle1 , circle2 or circle3 )
    //by Compare radius of circle with the distance of
    //the robot from circle center 
    if ((xLOC - 0) * (xLOC - 0) +
        (yLOC - 0) * (yLOC - 0) <= 2.5 * 2.5)
        return "c1";
    else
        if ((xLOC - 0) * (xLOC - 0) +
        (yLOC - 0) * (yLOC - 0) <= 5 * 5)
        return "c2";
    else   
        if ((xLOC - 0) * (xLOC - 0) +
        (yLOC - 0) * (yLOC - 0) <= 10 * 10)
        return "c3";
    else 
        return "null";
}

//Resampling with replacement biased on weight depend on this technique https://www.sebastiansylvan.com/post/importancesampling/ 
void Resample(vector<Particle>& input, vector<Particle>& outputs, int inputN, int outN)
{
    float sumWeights = 0.0f;
    for (int i = 0; i < inputN; ++i)
    {
        sumWeights += input[i].getWeight();
    }
    
    float sampleWidth = sumWeights / outN;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> rnd(0, sampleWidth);
    int outputSampleIx = -1;
    float weightSoFar = -rnd(generator);
    for (int i = 0; i < outN; ++i)
    {   
        // How far is this sample from the origin (minus offset)?       
        float sampleDist = i*sampleWidth;

        // Find which sample to output. Just walk up the samples until the sum
        // of the weights is > to the distance of the current sample
		
        while (sampleDist >= weightSoFar && outputSampleIx + 1 < inputN)
        {
            weightSoFar += input[++outputSampleIx].getWeight();
        }           
        outputs[i] = input[outputSampleIx]; 
    }
}

//Particle filter algorithm which take the particles set, motion information(distance , theta) and sensor information (z)  
vector<Particle> particleFilter(vector<Particle>& p, double dist,double theta ,string z)
{   
    vector<Particle>pNew (size) ;
    float m=0;//total weight
    int n=0;//number of particles
    
    //Sampling with replacement function
    Resample(p,pNew,size,size);
    
    //Update Particles location and weight
    for(int i = 0; i < size; i++) {
        pNew[i].moveForward(dist);
        pNew[i].update_Weight(z);
        n++;
        m=m+pNew[i].getWeight();
    }
    //Particles Weights Normalization
    for(int i = 0; i < size; i++) {
       double oldWeight= pNew[i].getWeight();
       double newWeight=(oldWeight*1.0)/(m*1.0);
       pNew[i].setWeight(newWeight);
    }
    return pNew;
}
//Calculate mean of x,mean of y
void CalculateMean(vector<Particle>& p,double& meanX,double& meanY)
    {
        double sumX = 0;
        double sumY = 0;
        for(int i = 0; i < size; i++)
        {
            sumX += p[i].getX();
            sumY += p[i].getY();
            
        }
        meanX= (sumX / size);
        meanY= (sumY / size);
    }
    
//calculate Variance of x,Variance of y 
void CalculateVariane(vector<Particle> p,double& varX,double& varY)
    {   
        CalculateMean(p,meanX,meanY);
        double tempX = 0;
        double tempY = 0;
        for(int i = 0; i < size; i++)
        {
             tempX += (p[i].getX() - meanX) * (p[i].getX() - meanX) ;
             tempY += (p[i].getY() - meanY) * (p[i].getY() - meanY) ;
        }
        varX= (tempX*1.0) / size;
        varY= (tempY*1.0) / size;
    }
//main function
int main()
{    
    srand(time(0));
    //Initilaize robot location
    Particle robot;
    robot.setX(0);robot.setY(0);robot.setAngle(1.57);robot.setWeight(1);
    
    double dist,theta;//Motion Information (U)
    string z;//Sensor Information
    double varX,varY;
    vector<Particle> particles(size);//Set of  particles 
    vector<Particle> newParticles(size);//Set of the updated particles after filtering

    while (1){
    cout<<"Enter number of meters,please "<<endl;
    cin>>dist;//enter 0 if we just need to rotate
    cout<<"Enter Theta of movement"<<endl;
    cin>>theta;// enter 0 if we just need  to move forward
    robot.moveForward(dist);//dist is setting to 0 when we want robot to rotate
    robot.rotate(theta);//Theta is setting to 0 when we want robot to move forward
    z=sense(robot);//return sensor information
    cout <<"Robot location Without particle filter:"<<endl<<"X = "<<robot.getX()<<", Y = "<<robot.getY()
    <<", angle = "<<robot.getAngle()<<endl;//Robot location without using particleFilter algorithm
      newParticles=particleFilter(particles,dist,theta,z);
      particles=newParticles; 
      CalculateVariane(particles,varX,varY);
      cout<<"X mean = "<<meanX<<","<<"Y mean = "<<meanY<<endl;
      cout<<"X variance = "<<varX<<","<<"Y variance = "<<varY<<endl;
      if(0<=varX && varX<=0.1){
          if(0<=varY && varY<=0.1){
              cout<<"Robot found";
              cout<<"Robot (x,y) coordination is :="<<"("<<meanX<<","<<meanY<<")";
              break;
          }
      }
    }
    
    return 0;
}



