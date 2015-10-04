/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <stdio.h>
#include <unistd.h>

#include <stdlib.h>
  #include <sys/time.h>



#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>

#include <errno.h>




#include "core/base/KeyValueConfiguration.h"
#include "core/data/Container.h"
#include "core/data/TimeStamp.h"
#include "core/data/control/VehicleControl.h"
#include "OpenCVCamera.h"
#include "core/data/environment/VehicleData.h"
#include "GeneratedHeaders_Data.h"

#include "Proxy.h"

namespace msv {

int valIr1;
int valIr2;
int valIr3;
int valUs1;
int valUs2;
int valUs3;
int valWheelE;

string convertedAngle;


int wd;
int fd;
struct termios oldR;
struct termios oldW;
const char *MODEMDEVICE ;
string readings;
double distance;
string userInput="512060,";

void close(int x);

bool open1(const char* x,int y);

void connect(string x,int y);

void write(string text);
string read();
    
    using namespace std;
    using namespace core::wrapper;
    using namespace core::base;
    using namespace core::data;
    using namespace tools::recorder;
using namespace core::data::control;

    Proxy::Proxy(const int32_t &argc, char **argv) :
	 ConferenceClientModule(argc, argv, "proxy"),
    m_recorder(NULL),
    m_camera(NULL)
        //arduino(ArduinoBaundRate::B9600bps)
    {}

    Proxy::~Proxy() {
    }

    void Proxy::setUp() {
      msv::connect("/dev/ttyACM0",1); // connect to arduino reading from
      msv::connect("/dev/ttyACM0",2); // connect to arduino sending to


	    // This method will be call automatically _before_ running body().
        if (getFrequency() < 20) {
            cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
        }

        // Get configuration data.
        KeyValueConfiguration kv = getKeyValueConfiguration();
	
	
        // Create built-in recorder.
        const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
        if (useRecorder) {
            // URL for storing containers.
            stringstream recordingURL;
            recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS() << ".rec";
            // Size of memory segments.
            const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
            // Number of memory segments.
            const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
            // Run recorder in asynchronous mode to allow real-time recording in background.
            const bool THREADING = true;

            m_recorder = new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING);
        }

        // Create the camera grabber.
        const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
        string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
        std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
        const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
        const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
        const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
        const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");

        if (TYPE.compare("opencv") == 0) {
            m_camera = new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP);
        }

        if (m_camera == NULL) {
            cerr << "No valid camera type defined." << endl;
        }
    }

    void Proxy::tearDown() {
	    // This method will be call automatically _after_ return from body().
        OPENDAVINCI_CORE_DELETE_POINTER(m_recorder);
        OPENDAVINCI_CORE_DELETE_POINTER(m_camera);
    }

    void Proxy::distribute(Container c) {
        // Store data to recorder.
        if (m_recorder != NULL) {
            // Time stamp data before storing.
            c.setReceivedTimeStamp(TimeStamp());
            m_recorder->store(c);
        }

        // Share data.
        getConference().send(c);
    }

    // This method will do the main data processing job.
    ModuleState::MODULE_EXITCODE Proxy::body() {
     
        uint32_t captureCounter = 0;
        SensorBoardData sensorBoardData;
        core::data::environment::VehicleData vd;
        while (getModuleState() == ModuleState::RUNNING) {

            // Capture frame.
            if (m_camera != NULL) {
                core::data::image::SharedImage si = m_camera->capture();

                Container c(Container::SHARED_IMAGE, si);
                distribute(c);
                captureCounter++;
            }
	           Container containerVehicleControl = getKeyValueDataStore().get(Container::VEHICLECONTROL);
             VehicleControl vc = containerVehicleControl.getData<VehicleControl> ();
             
             cerr << "Speed data: " << vc.getSpeed() << endl;
             cout << "Angle : " << vc.getSteeringWheelAngle()<<endl;
            // TODO: Here, you need to implement the data links to the embedded system
            // to read data from IR/US.

             int angle=60;
             int angleFromDriver= (int)vc.getSteeringWheelAngle(); // receive steeringaAngle from VehicleControl

             //convert the angle for arduino
             if(angleFromDriver <0)
              angleFromDriver*=-1;
            else
               angleFromDriver*=-1;
             angle+=angleFromDriver;
             stringstream ss;
              ss << angle;
              if(angle<100 && angle >10)
             convertedAngle="0"+ss.str();
           else if(angle <10 && angle>-1)
            convertedAngle="00"+ss.str();
           else
            convertedAngle=ss.str();
          

          // send different values depending on drivers speed
            if(vc.getSpeed()>0)
              userInput="600"+convertedAngle+",";
            else if(vc.getSpeed()<1 && vc.getSpeed()>-1)
              userInput="512"+convertedAngle+",";
            else if(vc.getSpeed()<-1)
              userInput="200"+convertedAngle+",";
              
              
              cout<<userInput<<endl;

              if(wd!=-1 && wd!=0)
             msv::write(userInput); // write to arduino


      if(fd!=-1 && fd!=0){

  readings=msv::read(); // read from arduino
	
  cout<< "readings are "<< readings << endl;
  
  int length=atoi(readings.substr(0,2).c_str());
  unsigned int finalLength=length+5; // check length
  

  //decode netstring received from arduino
  if(readings.length()==finalLength){
    string ir1=readings.substr(3,3);

    valIr1=atoi(ir1.c_str());
  	
    string ir2=readings.substr(6,3);

    valIr2=atoi(ir2.c_str());
    
    string ir3=readings.substr(9,3);

    valIr3=atoi(ir3.c_str());
    
    string us1=readings.substr(12,3);

    valUs1=atoi(us1.c_str());
    
    string us2=readings.substr(15,3);

    valUs2=atoi(us2.c_str());
    
    string us3=readings.substr(18,3);

    valUs3=atoi(us3.c_str());

    string wheelE=readings.substr(21,length-18);

    valWheelE=atoi(wheelE.c_str());
    
}

  cout<<"Wheel Encoder value " << valWheelE <<endl;

//Map decoded sensor values
  sensorBoardData.putTo_MapOfDistances(4,valUs2);
  sensorBoardData.putTo_MapOfDistances(3,valUs3);
  sensorBoardData.putTo_MapOfDistances(1,valIr3);
  sensorBoardData.putTo_MapOfDistances(2,valIr2);
  sensorBoardData.putTo_MapOfDistances(0,valIr1);
  sensorBoardData.putTo_MapOfDistances(5,valUs1);
  vd.setAbsTraveledPath(valWheelE);

  Container Pvd=Container(Container::VEHICLEDATA, vd);
  Container c = Container(Container::USER_DATA_0, sensorBoardData);
  
  distribute(c);
  distribute(Pvd);

  tcflush(fd, TCIFLUSH);
}

if(wd!=-1 && wd!=0)
  tcflush(wd, TCOFLUSH);
//usleep(2000000);

 }  
        if(wd!=-1 && wd!=0){
        msv::write("512060,");
        msv::close(wd);
        }
        if(fd!=-1 && fd!=0)
        msv::close(fd);
        
        cout << "Proxy: Captured " << captureCounter << " frames." << endl;

        return ModuleState::OKAY;
    }


void connect(string address,int z)
{
  
 
MODEMDEVICE=address.c_str ();
  if(MODEMDEVICE!=0)
  open1(MODEMDEVICE,z);
}


/*
Found at https://github.com/ranma1988/cArduino/blob/master/SOURCE/cArduino.cpp
*/
bool open1(const char *DeviceFileName,int x)
{
  struct termios newtio;

  MODEMDEVICE = DeviceFileName;
  /*
  Open modem device for reading and writing and not as controlling tty
  because we don't want to get killed if linenoise sends CTRL-C.
  */
  if(MODEMDEVICE==0)
  return false;
if(x==1){
  fd = ::open(MODEMDEVICE, O_RDWR | O_NOCTTY );
  if (fd <0) {
    perror(MODEMDEVICE);
    return false;
  }

  tcgetattr(fd,&oldR); /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
}
if(x==2){
  wd = ::open(MODEMDEVICE, O_RDWR | O_NOCTTY );
  if (wd <0) {
    perror(MODEMDEVICE);
    return false;
  }

  tcgetattr(wd,&oldW); /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
}
  /*
  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
  CRTSCTS : output hardware flow control (only used if the cable has
      all necessary lines. See sect. 7 of Serial-HOWTO)
  CS8     : 8n1 (8bit,no parity,1 stopbit)
  CLOCAL  : local connection, no modem contol
  CREAD   : enable receiving characters
  */
  newtio.c_cflag =  B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;

  /*
  IGNPAR  : ignore bytes with parity errors
  ICRNL   : map CR to NL (otherwise a CR input on the other computer
      will not terminate input)
  otherwise make device raw (no other input processing)
  */
  newtio.c_iflag = IGNPAR | ICRNL;

  /*
  Raw output.
  */
  newtio.c_oflag = 0;

  /*
  ICANON  : enable canonical input
  disable all echo functionality, and don't send signals to calling program
  */
  newtio.c_lflag = ICANON;

  /*
  initialize all control characters
  default values can be found in /usr/include/termios.h, and are given
  in the comments, but we don't need them here
  */
  newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
  newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  newtio.c_cc[VERASE]   = 0;     /* del */
  newtio.c_cc[VKILL]    = 0;     /* @ */
  newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
  newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
  newtio.c_cc[VSWTC]    = 0;     /* '\0' */
  newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
  newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  newtio.c_cc[VEOL]     = 0;     /* '\0' */
  newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  newtio.c_cc[VEOL2]    = 0;     /* '\0' */

  /*
  now clean the modem line and activate the settings for the port
  */
 if(x==1){
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
}
if(x==2){
  tcflush(wd, TCOFLUSH);
  tcsetattr(wd,TCSANOW,&newtio);
}
  /*
  terminal settings done, now handle input
  */
  return true;
}

string read()
{
  /* read blocks program execution until a line terminating character is
  input, even if more than 255 chars are input. If the number
  of characters read is smaller than the number of chars available,
  subsequent reads will return the remaining chars. res will be set
  to the actual number of characters actually read */
  char buf[255];

  int res = ::read(fd,buf,255);
  buf[res]=0;             /* set end of string, so we can printf */

  string ret(buf);
  return ret;
}
void write(string text)
{
  ::write(  wd,(char*)text.c_str(),(size_t)text.length() );
}
void close(int x){
  ::close(x);
}
} // msv


