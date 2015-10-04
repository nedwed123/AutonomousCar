/**
 * lanedetector - Sample application for detecting lane markings.
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

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ctime>
#include <iomanip> 


#include "core/macros.h"
#include "core/base/KeyValueConfiguration.h"
 #include "core/data/TimeStamp.h"
#include "core/data/Container.h"
#include "core/data/image/SharedImage.h"
#include "core/io/ContainerConference.h"
#include "core/wrapper/SharedMemoryFactory.h"
 #include "core/wrapper/Time.h"
#include "core/wrapper/TimeFactory.h"


#include "tools/player/Player.h"

#include "GeneratedHeaders_Data.h"

#include "LaneDetector.h"

namespace msv {

    using namespace std;
    using namespace core::base;
    using namespace core::data;
    using namespace core::data::image;
    using namespace tools::player;
    using namespace cv;
double vp2leftLength;
double vp2rightLength;
int threshold_ratio = 170;
int state=0;
SteeringData sd;
double difference;
double value;
int intersectionFound=0;
int sim=1; // if real track change to 0
int simLength=351;
int simFactor=175;
int trackLength=380;
int trackFactor=190;
int vanishingpoint=300;

    //int dynamicYaxis = 1; // to move lines on the y axis

    LaneDetector::LaneDetector(const int32_t &argc, char **argv) : ConferenceClientModule(argc, argv, "lanedetector"),
        m_hasAttachedToSharedImageMemory(false),
        m_sharedImageMemory(),
        m_image(NULL),
        m_debug(false) {}

    LaneDetector::~LaneDetector() {}

    void LaneDetector::setUp() {
        // This method will be call automatically _before_ running body().
        if (m_debug) {

            
            // Create an OpenCV-window.
            cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
            cvMoveWindow("WindowShowImage", 300, 100);
        }
    }

    void LaneDetector::tearDown() {
        // This method will be call automatically _after_ return from body().
        if (m_image != NULL) {
            cvReleaseImage(&m_image);
        }

        if (m_debug) {
            cvDestroyWindow("WindowShowImage");
        }
    }

    bool LaneDetector::readSharedImage(Container &c) {
        bool retVal = false;

        if (c.getDataType() == Container::SHARED_IMAGE) {
            SharedImage si = c.getData<SharedImage> ();

            // Check if we have already attached to the shared memory.
            if (!m_hasAttachedToSharedImageMemory) {
                m_sharedImageMemory
                        = core::wrapper::SharedMemoryFactory::attachToSharedMemory(
                                si.getName());
            }

            // Check if we could successfully attach to the shared memory.
            if (m_sharedImageMemory->isValid()) {
                // Lock the memory region to gain exclusive access. REMEMBER!!! DO NOT FAIL WITHIN lock() / unlock(), otherwise, the image producing process would fail.
                m_sharedImageMemory->lock();
                {
                    const uint32_t numberOfChannels = 3;
                    // For example, simply show the image.
                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
                    }

                    // Copying the image data is very expensive...
                    if (m_image != NULL) {
                        memcpy(m_image->imageData,
                               m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * numberOfChannels);
                    }
                }

                // Release the memory region so that the image produce (i.e. the camera for example) can provide the next raw image data.
                m_sharedImageMemory->unlock();

                // Mirror the image.
                if(sim)
         cvFlip(m_image, 0, -1);

                retVal = true;
            }
        }
        return retVal;
    }

void MyLine( Mat img, Point start, Point end, Scalar color)
{
  int thickness = 1.5;
  int lineType = 8;
  line( img,
        start,
        end,
        color,
        thickness,
        lineType );
}



//Fuction returns type Vec3b of pixel intensity (blue,green,red).
Vec3b getintensity(Mat img, int y, int x){
    Vec3b intensity = img.at<Vec3b>(y,x);
    return intensity;
    }

bool findWhite(Mat img, int y, int x){
    Vec3b intensity = img.at<Vec3b>(y,x);
    int blue = intensity.val[0];
    int green = intensity.val[1];
    int red = intensity.val[2];
    if(blue == 255 && green == 255 && red == 255){
        //std::cout << "intensity "<< intensity << std::endl;
        return true;
    }
return false;
}

int getRightLine(Mat img, int yAxis, int count, int &rightLineLength){
    if(count>640)
       
        return rightLineLength;
    
    if(findWhite(img, yAxis, count)==true){
        //std::cout << "white pixel at count"<< count << std::endl;
        rightLineLength = count;
        //std::cout << "white pixel at value"<< rightLineLength << std::endl;
        return rightLineLength;
    
        }else {
            count++;
            getRightLine(img, yAxis, count, rightLineLength);
            //std::cout << "white pixel"<< std::endl;
        } 
return rightLineLength;
}


int getLeftLine(Mat img, int yAxis, int count, int &leftLineLength){
     if(count<1)
        return leftLineLength;
    if(findWhite(img, yAxis, count)==true){
        //std::cout << "white pixel at count"<< count << std::endl;
        leftLineLength = count;
        //std::cout << "white pixel at value"<< leftLineLength << std::endl;
        return leftLineLength;
        }else {
            count--; 
            getLeftLine(img, yAxis, count, leftLineLength);

            //std::cout << "white pixel"<< std::endl;
        }
        return leftLineLength;
}

void drawLine(Mat atom_image, int line1line, int count, int &line1leftLineLength, int &line1rightLineLength){
    
    getRightLine(atom_image, line1line, count, line1rightLineLength);
    //MyLine( atom_image, Point( count, line1line ), Point( line1rightLineLength, line1line),Scalar( 255, 0, 0)); //Blue line 
    //std::cout << "line1 right Length "<< line1rightLineLength - count << std::endl;
    
    getLeftLine(atom_image, line1line, count, line1leftLineLength);
    //MyLine( atom_image, Point( count, line1line ), Point( line1leftLineLength, line1line),Scalar( 255, 0, 0)); //other clour line 
    //std::cout << "line1 left Length "<<  count - line1leftLineLength << std::endl;

    
}


//bool findintersection(Mat img){
  //  Vec3b intensity = img.at<Vec3b>(y,x);
//}
   
    // You should start your work in this method.
    void LaneDetector::processImage() {
       
        // Example: Show the image.
        if (m_debug) {
            if (m_image != NULL) {
                //cvShowImage("WindowShowImage", m_image);
                cvWaitKey(10);
            }
        }
  
       Mat src,dst,color_dst,atom_image,edge,blured;
       src=m_image;
       //atom_image=src;
       blur(src , blured, Size(3,3) );
       cvtColor( blured, color_dst, COLOR_RGB2GRAY );

    if(sim)
     Canny( color_dst, dst, 50, 170, 3);
    else
     Canny(color_dst, dst, 50, threshold_ratio, 3);

    dst.convertTo(edge, CV_8U);
    // threshold( color_dst, dst, threshold_value, max_BINARY_value,threshold_type );
    cvtColor( edge, atom_image, COLOR_GRAY2RGB );

   if(!sim){
    Mat roi = edge( Rect(0,220,639,259) );
    //imshow("exposure",roi);
    cv::Mat mask;    //image is CV_8UC1
    cv::inRange(roi, 255, 255, mask);
    int whiteCount = cv::countNonZero(mask);
    cout << "No: white pixels " << whiteCount << endl;
    cout<< "Threshold: " << threshold_ratio<<endl;
    if(whiteCount < 1600) // 2138
        threshold_ratio--;
    else if(whiteCount > 3000)
        threshold_ratio++;
        else
           threshold_ratio = 170;
   }
   
        //TODO: Start here.


        // 1. Do something with the image m_image here, for example: find lane marking features, optimize quality, ...

    
    //Greyscale
    
    if(state==0){
        sd.setIntersectionFound(0.0);
    int count = 320;//middle of right lane/heading

    //Draw Heading Line
    Point pStart(count,500), pEnd(count,50);
    MyLine(atom_image,pStart,pEnd,Scalar(180,255,100));

    //line1lines

    int line7line = 225;
    int line7leftLineLength =0;
    int line7rightLineLength =0;
    drawLine(atom_image, line7line, count, line7leftLineLength, line7rightLineLength);

    int line8line = 235;
    int line8leftLineLength =0;
    int line8rightLineLength =0;
    drawLine(atom_image, line8line, count, line8leftLineLength, line8rightLineLength);

    int line6line = 260;
    int line6leftLineLength =0;
    int line6rightLineLength =0;
    drawLine(atom_image, line6line, count, line6leftLineLength, line6rightLineLength);

    //TopLines 1
    int topline = 275;
    int topleftLineLength =0;
    int toprightLineLength =0;
    drawLine(atom_image, topline, count, topleftLineLength, toprightLineLength);

    int line4line= 295;
    int line4leftLineLength = 0;
    int line4rightLineLength = 0;
    drawLine(atom_image, line4line, count, line4leftLineLength, line4rightLineLength);
    
    int line2line= 310;
    int line2leftLineLength = 0;
    int line2rightLineLength = 0;
    drawLine(atom_image, line2line, count, line2leftLineLength, line2rightLineLength);

    int interLine= 340;
    int interLineleft = 0;
    int interLineright = 0;
    drawLine(atom_image, interLine, count, interLineleft, interLineright);

     int inter1Line= 330;
    int inter1Lineleft = 0;
    int inter1Lineright = 0;
    drawLine(atom_image, inter1Line, count, inter1Lineleft, inter1Lineright);

    
    int line5line= 349;
    int line5leftLineLength = 0;
    int line5rightLineLength = 0;
    drawLine(atom_image, line5line, count, line5leftLineLength, line5rightLineLength);

    int line3line= 395;
    int line3leftLineLength = 0;
    int line3rightLineLength = 0;
    drawLine(atom_image, line3line, count, line3leftLineLength, line3rightLineLength);

    int line1line = 478;
    int line1leftLineLength = 0;
    int line1rightLineLength = 0;
    drawLine(atom_image, line1line, count, line1leftLineLength, line1rightLineLength);

    

//--------------------------------------------------------------------------------------------------------


    //1-8   
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line8leftLineLength, line8line),Scalar( 255, 255, 255)); //other clour line 

    //1-7   
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line7leftLineLength, line7line),Scalar( 255, 255, 255)); //other clour line 
    //1-top   
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( topleftLineLength, topline),Scalar( 255, 255, 255)); //other clour line 
    //1-2
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line2leftLineLength, line2line),Scalar( 255, 255, 255));
    //1-3
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line3leftLineLength, line3line),Scalar( 255, 255, 255)); //other clour line 
    //1-4
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line4leftLineLength, line4line),Scalar( 255, 255, 255)); //other clour line 
    //1-5
    MyLine(atom_image, Point( line1leftLineLength, line1line ), Point( line5leftLineLength, line5line),Scalar( 255, 255, 255)); //other clour line 

    //2-3
    MyLine(atom_image, Point( line2leftLineLength, line2line ), Point( line3leftLineLength, line3line),Scalar( 255, 255, 255)); //other clour line 
    //2-top
    MyLine(atom_image, Point( line2leftLineLength, line2line ), Point( topleftLineLength, topline),Scalar( 255, 255, 255)); //other clour line 
    //2-6
    MyLine(atom_image, Point( line2leftLineLength, line2line ), Point( line6leftLineLength, line6line),Scalar( 255, 255, 255)); //other clour line 
    
    //3-4connect 
    MyLine(atom_image, Point( line3leftLineLength, line3line ), Point( line4leftLineLength, line4line),Scalar( 255, 255, 255));
    MyLine(atom_image, Point( line5leftLineLength, line5line ), Point( topleftLineLength, topline),Scalar( 255, 255, 255));

    //VPointLines2
    int VPointLines2 = 350;
    int VP2leftLineLength =0;
    int VP2rightLineLength =0;
    
   
    getRightLine(atom_image, VPointLines2, count, VP2rightLineLength);
    MyLine( atom_image, Point( count, VPointLines2 ), Point( VP2rightLineLength, VPointLines2),Scalar( 0, 0, 255)); //other clour line 
    //std::cout << "mid right Length "<<  VP2rightLineLength - count<< std::endl;

    getLeftLine(atom_image, VPointLines2, count, VP2leftLineLength);
    MyLine( atom_image, Point( count, VPointLines2 ), Point( VP2leftLineLength, VPointLines2),Scalar( 0, 0, 255)); //other clour line 
    //std::cout << "mid left Length "<<  count - VP2leftLineLength << std::endl;


    //Length of vp2 lines 
    vp2leftLength = count - VP2leftLineLength;
    cout<<"vanishingpoint Left Length2 " << vp2leftLength << endl;

    vp2rightLength = VP2rightLineLength - count;
    cout<<"vanishingpoint Right Length2 " << vp2rightLength << endl;
    cout<< "Total "<< vp2leftLength+vp2rightLength << endl;
    


if(sim){
    if(interLineright==321 && interLineleft==319 && vp2rightLength>100 )
        state=1; 
    else if(inter1Lineright==321 && inter1Lineleft==319 && vp2rightLength>100 )
        state=1;
}else{
    if(vp2leftLength>210 && vp2rightLength>250 && intersectionFound<1)
      state=1;
    }   



   /*if(vp2rightLength>250)
       vp2rightLength=175;*/
    //Angles of line1 lines
    double vp2LAngle = atan(vp2leftLength/vanishingpoint) * 180.0 / CV_PI; //was 250 but adjust for turn 
    //cout<<"L Angle is " << vp2LAngle << endl;
    double vp2RAngle = atan(vp2rightLength/vanishingpoint) * 180.0 / CV_PI;
    //cout<<"R Angle is " << vp2RAngle << endl;
    //cout << "difference is " << vp2RAngle-vp2LAngle << endl;
    imshow( "atom_window", atom_image );

    double tangent=((350.0-309.0)/(480.0-423.0));

    
    if(sim){
  if((vp2rightLength>300 || vp2rightLength <0) && intersectionFound<1){
               

            value=(value-(tangent/13)); //was 7//
          
           difference=value;
}else if(vp2leftLength<vp2rightLength && vp2leftLength + vp2rightLength > simLength )
       difference = vp2RAngle-(vp2LAngle/(vp2leftLength/simFactor));
else if(vp2rightLength<vp2leftLength && vp2leftLength + vp2rightLength > simLength)
    difference = (vp2RAngle/(vp2rightLength/simFactor))-vp2LAngle;

    else if( vp2leftLength + vp2rightLength < simLength )
       difference = (vp2RAngle*(vp2leftLength/simFactor))- vp2LAngle;
       
    else
        difference= vp2RAngle-vp2LAngle;

if((vp2leftLength+vp2rightLength>500 && vp2leftLength+vp2rightLength<550 ) && intersectionFound==1){
        intersectionFound=2;
    }

    if(intersectionFound>1 && (vp2leftLength+vp2rightLength<320))
        intersectionFound=0;
  
    if(vp2rightLength>310 && intersectionFound > 0){
        
    difference=0;
    }  
    
}
else{
    if((vp2rightLength>500 || vp2rightLength <0) && intersectionFound<1){

            value=(value-(tangent/13)); //was 7//
           difference=value;

}else if(vp2leftLength<vp2rightLength && vp2leftLength + vp2rightLength > trackLength ) //354
       difference = vp2RAngle-(vp2LAngle/(vp2leftLength/trackFactor));
else if(vp2rightLength<vp2leftLength && vp2leftLength + vp2rightLength > trackLength)
    difference = (vp2RAngle/(vp2rightLength/trackFactor))-vp2LAngle; 

    else if( vp2leftLength + vp2rightLength < trackLength )
       difference = (vp2RAngle*(vp2leftLength/trackFactor))- vp2LAngle;
       
    else
        difference= vp2RAngle-vp2LAngle;
  
    if(vp2rightLength>310 && intersectionFound>0)
      difference=0;
      if(vp2rightLength<180 && vp2leftLength<180 && intersectionFound>0)
      intersectionFound=0;
 
}

    value=difference;
    std::cout<<"difference: "<< difference <<'\n';
    cout<<"intersection "<< intersectionFound<<endl;
    cout<<"interline " << interLineleft<<endl;
    cout<<"interline 1" << inter1Lineleft << endl;
    if(sim){
    if(difference>5)
        difference+=0.5; // adjustment for right curves
    sd.setExampleData(difference);
    }else{
        sd.setExampleData(difference*(2.4));
    }
   
    }
    if (state==1){
        sd.setIntersectionFound(1.0);
        state=0;
        intersectionFound=1;
        
    }
     
    

        // 2. Calculate desired steering commands from your image features to be processed by driver.



        // Here, you see an example of how to send the data structure SteeringData to the ContainerConference. This data structure will be received by all running components. In our example, it will be processed by Driver. To change this data structure, have a look at Data.odvd in the root folder of this source.
       
    

        // Create container for finally sending the data.
        Container c(Container::USER_DATA_1, sd);
        // Send container.
        getConference().send(c);
    }

    // This method will do the main data processing job.
    // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
    ModuleState::MODULE_EXITCODE LaneDetector::body() {
        // Get configuration data.
        KeyValueConfiguration kv = getKeyValueConfiguration();
        m_debug = kv.getValue<int32_t> ("lanedetector.debug") == 1;

        Player *player = NULL;
/*
        // Lane-detector can also directly read the data from file. This might be interesting to inspect the algorithm step-wisely.
        core::io::URL url("file://recorder.rec");

        // Size of the memory buffer.
        const uint32_t MEMORY_SEGMENT_SIZE = kv.getValue<uint32_t>("global.buffer.memorySegmentSize");

        // Number of memory segments.
        const uint32_t NUMBER_OF_SEGMENTS = kv.getValue<uint32_t>("global.buffer.numberOfMemorySegments");

        // If AUTO_REWIND is true, the file will be played endlessly.
        const bool AUTO_REWIND = true;

        player = new Player(url, AUTO_REWIND, MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS);

*/
        // "Working horse."
        while (getModuleState() == ModuleState::RUNNING) {
            bool has_next_frame = false;

            // Use the shared memory image.
            Container c;
            if (player != NULL) {
                // Read the next container from file.
                c = player->getNextContainerToBeSent();
            }
            else {
                // Get the most recent available container for a SHARED_IMAGE.
                c = getKeyValueDataStore().get(Container::SHARED_IMAGE);
            }                

            if (c.getDataType() == Container::SHARED_IMAGE) {
                // Example for processing the received container.
                has_next_frame = readSharedImage(c);
            }

            // Process the read image.
            if (true == has_next_frame) {
                processImage();
            }
        }

        OPENDAVINCI_CORE_DELETE_POINTER(player);

        return ModuleState::OKAY;
    }

} // msv
