/**
 * driver - Sample application for calculating steering and acceleration commands.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <math.h>

#include "core/io/ContainerConference.h"
#include "core/data/Container.h"
#include "core/data/Constants.h"
#include "core/data/control/VehicleControl.h"
#include "core/data/environment/VehicleData.h"

#include "GeneratedHeaders_Data.h"

#include "Driver.h"

namespace msv {

        using namespace std;
        using namespace core::base;
        using namespace core::data;
        using namespace core::data::control;
        using namespace core::data::environment;
        //------------------overtaking and lane following ------------------------ //
int dState=0;
int state=0;
int count;
int parkingState=0;
double desiredSteeringWheelAngle ; 
int sim=1;
bool obstacleFront=false;
//checking turn to left 
bool startTurningToLeft=false;

// counter 1 
int count1=0;
// counter 2
int count2=0;
// the distance for calculating travel path
double distance;


        //------------------------- parking -------------------------------//
double pi = 3.14159265;
        double Rs1; //first steer angle radius of curvature
        double Rs2; //second steer angle radius of curvature
        float xd;  // distance the car needs to drive from point a before turning stearing wheel
        
        //Measure of gap used with IR sensors
        float gapSum;
        float gapStart;
        float gapEnd;

        // counters to adjust movement length
        float counter=0; // used as counter to measure the distance traveled and
        int trajCounter=0; // used to count the trajectory in the first curve
        
        //controlling states
        
        
        int trigger = 0; //used to trigger states, with no go back
        int parking1=0;

        Driver::Driver(const int32_t &argc, char **argv) :
            ConferenceClientModule(argc, argv, "Driver") {
        }

        Driver::~Driver() {}

        void Driver::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void Driver::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        // This method will do the main data processing job.
        ModuleState::MODULE_EXITCODE Driver::body() {

            while (getModuleState() == ModuleState::RUNNING) {
                // In the following, you find example for the various data sources that are available:

                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(Container::VEHICLEDATA);
                VehicleData vd = containerVehicleData.getData<VehicleData> ();
                cerr << "Most recent vehicle data: '" << vd.toString() << "'" << endl;

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(Container::USER_DATA_0);
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
                cerr << "Most recent sensor board data: '" << sbd.toString() << "'" << endl;

                // 3. Get most recent user button data:
                Container containerUserButtonData = getKeyValueDataStore().get(Container::USER_BUTTON);
                UserButtonData ubd = containerUserButtonData.getData<UserButtonData> ();
                cerr << "Most recent user button data: '" << ubd.toString() << "'" << endl;

                // 4. Get most recent steering data as fill from lanedetector for example:
                Container containerSteeringData = getKeyValueDataStore().get(Container::USER_DATA_1);
                SteeringData sd = containerSteeringData.getData<SteeringData> ();
                cerr << "Most recent steering data: '" << sd.toString() << "'" << endl;
        double IR_FR = sbd.getValueForKey_MapOfDistances(0);    //ir front right
        double IR_R = sbd.getValueForKey_MapOfDistances(1);   //ir rear rear
                double IR_RR = sbd.getValueForKey_MapOfDistances(2);    //ir rear right
                double US_FC = sbd.getValueForKey_MapOfDistances(3);  //ultra sonic front cener
                //double US_FR = sbd.getValueForKey_MapOfDistances(4);    //ultra sonic front right
                double US_RR = sbd.getValueForKey_MapOfDistances(5);    //ultra sonic rear right


                // Design your control algorithm here depending on the input data from above.



                // Create vehicle control data.
                VehicleControl vc;
                distance= vd.getAbsTraveledPath();

                // With setSpeed you can set a desired speed for the vehicle in the range of -2.0 (backwards) .. 0 (stop) .. +2.0 (forwards)
          //      vc.setSpeed(0.4);
                if(parkingState==1){
                   


                //Algorithm below used for parellel parking, - source of algorithm used stated in documentation
                double cl;  //car length
                cl = 4;
                double wl;  //length between front and back car wheel axels
                wl = 2.65;
                double ww;  //width between front or back car wheels
                ww = 1;
                double wfbf;   //front wheel to front bumper
                wfbf = 1;
                double fcr;  //full circle radious when car wheels at max angle
                fcr = 5.25;
                double ow;  //object width
                ow = 3.5;
                double parkspace;   //calculates the space needed to park
                parkspace = cl + sqrt((pow (fcr, 2) - pow(wl, 2))+(2+ pow(ww, 2))-pow((sqrt(pow(fcr, 2)- pow(wl, 2)-ow)), 2)-wl-wfbf);
                
                Rs1 = (wl*(tan(25*pi/180.0)))+ww/2;     //first steer angle radius of curvature
                Rs2 = (wl*(tan(25*pi/180.0)))+ww/2;     //second steer angle radius of curvature
                
                double aq1;    // radius of the curvature corresponding to the first steering angle
                double aq2;    // radius of the curvature corresponding to the second steering angle
                float aaqq;   // (aq2-aq1) 
                 //  aq = cw +xc;       
                aq1 = Rs1 - ww/2;
                aq2 = Rs2 + ww/2;
                aaqq = (aq2-aq1); // Used to get the frst distanceo of trajectory
                xd = (pow((Rs1*Rs2),2)-pow((Rs1),2))+5.6; //distance from start of gap and to whre the car is "paralell" with next object after end of gap
              
                /*
                State 0 = counting when when gap is detected/start of gap
                State 1 = counting when when gap is ending/end of gap
                State 2 = checks if gap doesnt have an object
                State 3 = if no object detected in parking gap, its going to state 4, else stay on state 3 and continue forward
                State 4 = implement trajectory path. Stop when IR rear sensor gets to close to object.
                State 5 = Adjust car in parking slot, by using heading and IR rear
                */

                // speed to 1 and lanefollowing when parking1(state) is equal 0
                if(sim){
                if (parking1 ==0){    
                    vc.setSpeed(1.0);
                  //  desiredSteeringWheelAngle = sd.getExampleData();
                  // vc.setSteeringWheelAngle(desiredSteeringWheelAngle); // lane following
                } if(IR_RR<0 && state==0){                //state is 1 when is_rr is bigger than 0
                    gapStart = vd.getAbsTraveledPath();    //start calculating the traveled path
                    state=1;
                } if(IR_RR>0 && state==1 && trigger ==0){
                    gapEnd = vd.getAbsTraveledPath();      //stop calculating the traveled path
                }
                    gapSum = gapEnd - gapStart;  
                  if(gapSum > xd && xd > parkspace && trigger ==0 ){       // if travel path gapSum is bigger than xd path and if xd path
                    vc.setSpeed(0.0);
                    state =2;
                } if(state ==2 && US_RR>0 && trigger ==0){                                     //in state 3,
                    vc.setSpeed(1.0);
                    counter++;
                } if(state ==2 && (US_RR<0 || US_RR >7) && IR_FR <0 && trigger ==0 ){                                     //in state 3,
                    vc.setSpeed(0.0);
                    counter=143;
                    state=3;
                } if(state ==2 && (US_RR<0 || US_RR >7) && IR_RR >0 && IR_FR >0  && trigger ==0){                                     //in state 3,
                    vc.setSpeed(0.0);
                    parking1=1; // the speed on state 0 will be of
                    counter=158;
                    trigger=2;
                    state=4;
                } if(state ==3 && counter > 140 && trigger<2){      
                    trigger=1;
                    vc.setSpeed(1.0);
                    counter++;
                } if(state ==3 && counter > 163 && trigger==1){      
                    vc.setSpeed(0.0);
                    parking1=1;
                    trigger=2;
                    state=4;
                } if(state ==4 && counter >150 && trigger ==2){                                     //in state 3,
                    vc.setSteeringWheelAngle(26);              //the car wheels turn max to the right,
                    vc.setSpeed(-0.5);
                    trajCounter++;
                } if(state ==4  &&  (trajCounter/100)  >= aaqq && trigger ==2){
                    vc.setSteeringWheelAngle(-26);
                    vc.setSpeed(-0.4);
                } if(state == 4 && (IR_R > 0.9 && IR_R <=1.9) && trigger == 2){
                    trigger =3; // increase triggerger to not go back to state less then 5
                    vc. setSpeed(0.0);
                    state =5;
                } if(state == 5 && vd.getHeading() >= 0.09 && trigger==3 && (US_FC > 1.6 || US_FC <0)){
                    vc.setSteeringWheelAngle(27 );
                    vc.setSpeed(0.2);   
                } else if (state==5 &&   vd.getHeading() <= -0.09 && trigger==3 && (US_FC > 1.6 || US_FC <0)) {
                    vc.setSteeringWheelAngle(-27 );
                    vc.setSpeed(0.2);
                } else if(state == 5 && trigger>2  &&  (IR_R <0 || IR_R>1.7)){
                    vc.setSpeed(-0.2);
                    trigger=4;                       
                } else if(state == 5 && trigger==4 && (IR_R > 1.5 && IR_R <=1.9)){
                    vc.setSpeed(0);
                }
                        
                }else{
                        if (parking1 ==0){  
                   vc.setSpeed(1);
                    desiredSteeringWheelAngle = 7;
               vc.setSteeringWheelAngle(desiredSteeringWheelAngle); // lane following
                } 
                
                if(state==0 && trigger==0){
					//vc.setSteeringWheelAngle(-160);
					trigger=1;
					state=1;
				 }
				  if((IR_FR >3 && IR_FR <15)  && state==1 && trigger==1){

					state=2;
				    trigger=2;
				 }
                
                if(IR_RR>30 && state==2 && trigger==2){                //state is 1 when is_rr is bigger than 0
                    gapStart = vd.getAbsTraveledPath();    //start calculating the traveled path
                    counter++;                    
                } if(IR_RR < 16 && state==2 && trigger ==2){
                    gapEnd = vd.getAbsTraveledPath();
                          //stop calculating the traveled path
                }
                 if((IR_FR <35 || IR_RR<35) && trigger ==2 && counter > 30){       
                    parking1=1;
                    vc.setSpeed(-2);
                    counter++;
                } 

                  if(counter > 34 && trigger ==2){       
                    counter++;
                    parking1=1;
                    vc.setSpeed(0.0);
                    
          
   
                 // """TIMER"""" 
                }  if(counter > 80 && trigger ==2 ){      
                    vc.setSpeed(0.0);
                    trigger=3;
                    state =3;
                
             
        
                
                } if(state ==3 && trajCounter < 35 && trigger ==3){                                     
                   trajCounter++;
                    vc.setSteeringWheelAngle(45);              
                    vc.setSpeed(-2);
                    
                }   

                // """TIMER""""
                 if(state==3 && trajCounter > 34 && trigger <5 ){      
                    trajCounter++;
                    vc.setSpeed(0.0);
                    trigger=4;
                
             
        
                
                } 

                 
                if(state ==3 && (IR_R >34) &&  trajCounter >=34   && trigger <6){
                    trajCounter++;
                    trigger=5;
                    vc.setSteeringWheelAngle(-55);
                    vc.setSpeed(-2);
                
                   
                }    // """TIMER""""
                 if(state==3  && trajCounter > 59 && trigger <7 ){      
                    vc.setSpeed(1);
                    trigger=6;
                    trajCounter++;
                    vc.setSpeed(0);
                    
                

                }  
                 if(state > 2 && (IR_R <40 || IR_R >100) && trajCounter >83 && trigger <8){
                         vc.setSpeed(0);
                       state=4;
                       trigger=7;
                   
                   vc.setSteeringWheelAngle(0);
                   // trigger =5; // increase trigger to not go back to state less then 5
                 }

                 if(state == 4 && IR_R >33 && trajCounter >80 && trigger <8){
                       vc.setSpeed(-2);
                   vc.setSteeringWheelAngle(0);
                   state=5;
                   // trigger =5; // increase trigger to not go back to state less then 5
                 } if(state==5 && trigger==7){
                    vc.setSpeed(1);
                    vc.setSpeed(0);

                 }
                }
             
                //Printouts
                std::cout << "Parking Space "<< parkspace << std::endl;
                std::cout << "Heading "<< vd.getHeading() << std::endl;
                std::cout << "STATE "<< state << std::endl;
           
   
   
}else{
    
if(sd.getIntersectionFound()>0)
dState=1;
        if(sim){
                //  When car is moving in a distance less than 90 or bigger than 150 we are in straight
                if((distance<90 || distance>150) && dState==0){
                // lane following when US_FC is not detecting or has distance that is less then 10 and when IR_FR is not detecting
                if((sbd.getValueForKey_MapOfDistances(3)<0 || sbd.getValueForKey_MapOfDistances(3)>10 )  && obstacleFront==false && startTurningToLeft==false) {
                        cout<< "tracking" << endl;
                        vc.setSpeed(3);
                        desiredSteeringWheelAngle=sd.getExampleData();
                       
                //Obstacle detected from front when 0<US_FC<10. 
                } else if(sbd.getValueForKey_MapOfDistances(3)>0 && sbd.getValueForKey_MapOfDistances(3)<10 && obstacleFront==false && startTurningToLeft==false) {
                        cout<< "Changing boolean"<<endl;
                        obstacleFront=true;
                        state=1;

                //changing boolean turning to the left to true in this statement
                }else if(state==1 && obstacleFront==true) { // check if obstacle is found in front
                        cout<< "obstacle found" << endl;
                        startTurningToLeft=true;
                        obstacleFront=false;
                
                // in this statement car start to turn to the left with -20 degree and count2 increase until it reaches 100
                }else if(state==1 && startTurningToLeft==true && count2<100){
                        cout<< "turning to left" << endl;
                        desiredSteeringWheelAngle=-20;
                        vc.setSpeed(1);
                        count2++;

                // end of state1 
                }else if(state==1){
                        state=2;

                //checking whether US_FR IR_FR or IR_RR is detecting obstacle, then switch to follow the left lane
                }else if(state ==2 && (sbd.getValueForKey_MapOfDistances(4)>0 || sbd.getValueForKey_MapOfDistances(2)>0 || sbd.getValueForKey_MapOfDistances(0)>0 )){
                        cout<< "lane following in left side" << endl;
                        vc.setSpeed(2);
                        desiredSteeringWheelAngle=sd.getExampleData();

                //checking IR_FR and US_FR are not detecting, the car has passed the obstacle and start to steer back with 15 degrees to follow 
                // the right lane again when the counter reaches 50.
                }else if(state ==2 && sbd.getValueForKey_MapOfDistances(0)<0 && sbd.getValueForKey_MapOfDistances(4)<0 && count1<50){
                        cout<< "turning back to right lane" << endl;
                        vc.setSpeed(2);
                        desiredSteeringWheelAngle=15;
                        count1++;
                        
                // return back to state 0 when the car has completely passed the obstacle.
                }else{
                       count1=0;
                       obstacleFront=false;
                       startTurningToLeft=false;
                       state=0;
                       count2=0;
                }
//__________________________________________________________________________________________________________________________________________________________________________________________________________________
                // when the car is moving in a distance bigger than 90 and less than 150 we are in curve
                //(we had to make this if condition as in curve the lane following was not recognizing the vanishing point)
                // in line 189, we only added a small steering to the left with 15 degrees before adjusting it with lane following after the front detection
                }else if ((distance>90 && distance<150) && dState==0){
                if((sbd.getValueForKey_MapOfDistances(3)<0 || sbd.getValueForKey_MapOfDistances(3)>10 ) && sbd.getValueForKey_MapOfDistances(0)<0 && obstacleFront==false && startTurningToLeft==false) {
                    cout<< "tracking" << endl;
                    vc.setSpeed(2);
                    desiredSteeringWheelAngle=sd.getExampleData();
                       
                }else if(sbd.getValueForKey_MapOfDistances(3)>0 && sbd.getValueForKey_MapOfDistances(3)<10 && obstacleFront==false && startTurningToLeft==false) {
                        cout<< "Changing boolean"<<endl;
                        obstacleFront=true;
                        state=1;

                }else if(state==1 && obstacleFront==true) {
                        cout<< "obstacle found" << endl;
                        startTurningToLeft=true;
                        obstacleFront=false;
                
                }else if(state==1 && startTurningToLeft==true &&count2<100){
                        cout<< "turning to left" << endl;
                        desiredSteeringWheelAngle=-20;
                        vc.setSpeed(1);
                        count2++;

                }else if (state==1){
                        state=2;
                
                }else if (state ==2 && (sbd.getValueForKey_MapOfDistances(4)>0 || sbd.getValueForKey_MapOfDistances(2)>0 || sbd.getValueForKey_MapOfDistances(0)>0 )){
                         cout<< "turn to adjust curve" << endl;
                        vc.setSpeed(2);
                        desiredSteeringWheelAngle=15;

                }else if (state ==2 && (sbd.getValueForKey_MapOfDistances(4)>0 || sbd.getValueForKey_MapOfDistances(2)>0 || sbd.getValueForKey_MapOfDistances(0)>0 )){
                         cout<< "lane following in left side" << endl;
                        vc.setSpeed(2);
                        desiredSteeringWheelAngle=sd.getExampleData();

                }else if(state ==2 && sbd.getValueForKey_MapOfDistances(0)<0 && sbd.getValueForKey_MapOfDistances(4)<0 && count1<20){
                        cout<< "turning back to right lane" << endl;
                        vc.setSpeed(2);
                        desiredSteeringWheelAngle=15;
                        count1++;
                }else{
                    count1=0;
                    obstacleFront=false;
                    startTurningToLeft=false;
                    state=0;
                    count2=0;
                }
            }
        }else{
             if(US_FC>70 && state==0){
    cout << "                               1"<<endl;
    vc.setSpeed(1.0);
    //desiredSteeringWheelAngle=sd.getExampleData();
        state=1;
    }
    else if(state==1 && US_FC>70){
         cout << "                               2"<<endl;
        vc.setSpeed(1.0);
        //desiredSteeringWheelAngle=sd.getExampleData();
        
    }

    else if (US_FC<70 && state==1){
         cout << "                               3"<<endl;
    vc.setSpeed(1.0);
    desiredSteeringWheelAngle=-17;
      count2++;
state=2;
    }else if((US_RR >20 || (IR_RR<25 && IR_RR>18)) && state==2 && count2<5){
         cout << "                               4"<<endl;
        vc.setSpeed(1.0);
    desiredSteeringWheelAngle=-17;
    count2++;
     cout << "                       count is "<< count2<< endl;
  
    }

    else if (US_RR <40 && state==2){
    cout << "                                 changing degree to 20"<<endl;
    vc.setSpeed(1.0);
    desiredSteeringWheelAngle=40;
    state=3;
    }else if(state==3){
        vc.setSpeed(1.0);
    desiredSteeringWheelAngle=40;
  

    }else if (state==1 && IR_RR >23 && count1 <5){
        count1++;
        vc.setSpeed(1.0);
    desiredSteeringWheelAngle = -20;
}else{
    vc.setSpeed(1.0);
    desiredSteeringWheelAngle = 0;
}
        }
            if(sim)
            vc.setSteeringWheelAngle(desiredSteeringWheelAngle* Constants::DEG2RAD);
            else
            vc.setSteeringWheelAngle(desiredSteeringWheelAngle);
            
            if(dState==1){
                cout<<"in intersection" << endl;
                    vc.setSpeed(0.0);
                
                if(sim)
                vc.setSteeringWheelAngle(desiredSteeringWheelAngle* Constants::DEG2RAD);
                else
                vc.setSteeringWheelAngle(desiredSteeringWheelAngle);
                
                count++;
                if(count > 200){
                        sd.setIntersectionFound(0.0);
                        count=0;
                        dState=0;
                }

            }
    cout<<"distance " << distance << endl; 
}
           
                // You can also turn on or off various lights:
                vc.setBrakeLights(false);
                vc.setLeftFlashingLights(false);
                vc.setRightFlashingLights(true);

                // Create container for finally sending the data.
                Container c(Container::VEHICLECONTROL, vc);
                // Send container.
                getConference().send(c);
            }

            return ModuleState::OKAY;
        }
} // msv

