#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h> //Linux os
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include <iomanip>
#include <chrono>
#include <numeric>
//Graphs
#include "pbPlots.hpp"
#include "supportLib.hpp"

typedef void* HANDLE;
typedef int BOOL;
ßßßßßßßßßßßßßßßßßßßßßßß
enum EAppMode
{
	AM_UNKNOWN,
	AM_DEMO,
	AM_INTERFACE_LIST,
	AM_PROTOCOL_LIST,
	AM_VERSION_INFO
};

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
EAppMode g_eAppMode = AM_DEMO;

const string g_programName = "HelloEposCmd";

//Profile Velocity Default Inputs
long targetvelocity_nd_1 = 0; //rpm
long targetvelocity_nd_2 = 0; //rpm
long double simtime = 0;
vector<double> p_CurrentIs_saved;
vector<double> p_Time_saved;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   RunProfileVelocityMode(unsigned int* p_pErrorCode);
int   PrepareProfileVelocityMode(unsigned int* p_pErrorCode);
int   PrintAvailableInterfaces();
int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
int	  PrintAvailableProtocols();
int   PrintDeviceVersion();
void  Draw_plot_current_time(vector<double> *plot_current, vector<double>  *plot_time);
void  Calculate_averaged_current(vector<double> plot_current, vector<double>  plot_time);

void  Calculate_averaged_current(vector<double> plot_current, vector<double>  plot_time)
{
	double averaged_current = (std::accumulate(plot_current.begin(), plot_current.end(),0.0)) / plot_current.size();

	int counter_positive_elements = 0, counter_negative_elements = 0, counter_for_matching_time_step = 0,positive_piq_time_step = 0, negative_piq_time_step = 0;
	double summed_positive_current = 0, summed_negative_current = 0; 
	double avareged_positive_current = 0, avareged_negative_current = 0;
	double positive_current_piq = 0, negative_current_piq = 0;
	//averaged positive current
	for(std::vector<double>::iterator it = plot_current.begin(); it != plot_current.end() ; it ++)
	{
		counter_for_matching_time_step  ++;

		if (*it >= 0)
		{
			summed_positive_current +=*it;
			counter_positive_elements ++;
			if ( positive_current_piq <= *it)
			{
				positive_current_piq = *it;
				positive_piq_time_step = counter_for_matching_time_step;
			}
		}
		else
		{
			summed_negative_current +=*it;
			counter_negative_elements ++;
			if ( negative_current_piq >= *it)
			{
				negative_current_piq = *it;
				negative_piq_time_step = counter_for_matching_time_step;
			}
		}

	}

	avareged_positive_current = summed_positive_current / plot_current.size();
	avareged_negative_current = summed_negative_current / plot_current.size();

	std::cout << " Averaged Current "<<averaged_current<<" mA"<<endl;
	std::cout << " Averaged Positive Current "<<avareged_positive_current<<" mA" << " on "<< counter_positive_elements<< " measurements" <<endl;
	std::cout << " Averaged Negative Current "<<avareged_negative_current<<" mA" << " on "<< counter_negative_elements<< " measurements" <<endl;

	std::cout << " Piq Positive Current "<<positive_current_piq<<" mA" << " at "<< plot_time[positive_piq_time_step]<< "ms" <<endl;
	std::cout << " Piq Negative Current "<<negative_current_piq<<" mA" << " at "<< plot_time[negative_piq_time_step]<< "ms" <<endl;
}


void Draw_plot_current_time(vector<double> *plot_current, vector<double>  *plot_time)
{
	
	bool success;
    StringReference *errorMessage = new StringReference();
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = plot_time;
	series->ys = plot_current;
	series->linearInterpolation = false;
    series->pointType = toVector(L"dots");
	series->lineType = toVector(L"dotted");
	series->lineThickness = 2;
	series->color = CreateRGBColor(1,0,0);

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 1080;
	settings->height = 720;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(L"Current - Time Graph ");
	settings->xLabel = toVector(L"Time (ms)");
	settings->yLabel = toVector(L"Current (mA)");
	settings->scatterPlotSeries->push_back(series);

	success = DrawScatterPlotFromSettings(imageReference, settings, errorMessage);

    if(success){
        vector<double> *pngdata = ConvertToPNG(imageReference->image);
        WriteToFile(pngdata, "currentxtime" + std::to_string(targetvelocity_nd_1)+ " rpm"+ ".png");
        DeleteImage(imageReference->image);
	}else{
        cerr << "Error: ";
        for(wchar_t c : *errorMessage->string){
            wcerr << c;
        }
        cerr << endl;
	}

	success ? 0 : 1;

}

void PrintUsage()
{
	cout << "Usage: HelloEposCmd" << endl;
	cout << "\t-h : this help" << endl;
	cout << "\t-n : node id (default 1)" << endl;
	cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)"  << endl;
	cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)"  << endl;
	cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)"  << endl;
	cout << "\t-p   : port name (COM1, USB0, CAN0,... default - USB0)" << endl;
	cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
	cout << "\t-l   : list available interfaces (valid device name and protocol stack required)" << endl;
	cout << "\t-r   : list supported protocols (valid device name required)" << endl;
	cout << "\t-v   : display device version" << endl;
	cout << "Profile velocity Mode Settings----------------------------------------------------------" << endl;
	cout << "-Maxon Motor node 1 Settings------------------------------------------------------------" << endl;
	cout << "\t-x : specify target velocity (rpm)" << endl;
	cout << "\t-y : input simulation time (sec)" << endl;
	cout << "-Maxon Motor node 2 Settings------------------------------------------------------------" << endl;
	cout << "\t-x : specify target velocity (rpm)" << endl;
	cout << "\t-y : input simulation time (sec)" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate<<endl;

	msg << "Profile velocity Mode Parameters:"<<endl;
	msg << "target velocity node 1    = " << targetvelocity_nd_1 << "(rpm)"<<endl;
	msg << "target velocity node 2    = " << targetvelocity_nd_2 << "(rpm)"<<endl;
	msg << "simulation time     = " << simtime << "(sec)"<<endl;



	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	/*
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_portName = "USB0"; 
	g_baudrate = 1000000;
	*/

	//CAN
	g_usNodeId = 1;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "CANopen"; 
	g_interfaceName = "CAN_mcp251x 0"; 
	g_portName = "CAN0"; 
	g_baudrate = 250000; 
	targetvelocity_nd_1 = 100; //rpm
	targetvelocity_nd_2 = 100; //rpm

	simtime = 2; //sec
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;
		
		//Get the actual protocol parameter ans save on pointer variables
		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			//Set the protocol parameter
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				//Check if it is same as required
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	opterr = 0;

	while((lOption = getopt(argc, argv, "hlrvd:s:i:p:b:n:x:q:y:")) != -1)
	{
		switch (lOption)
		{
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case 'x':
				targetvelocity_nd_1 = atoi(optarg);
				break;
			case 'q':
				targetvelocity_nd_2 = atoi(optarg);
				break;
			case 'y':
				simtime = atoi(optarg);
				break;
			case 'l':
				g_eAppMode = AM_INTERFACE_LIST;
				break;
			case 'r':
				g_eAppMode = AM_PROTOCOL_LIST;
				break;
			case 'v':
				g_eAppMode = AM_VERSION_INFO;
				break;
			
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				LogInfo(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}

bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	int p_CurrentIs;
	/*
	vector<double> p_CurrentIs_saved;
	vector<double> p_Time_saved;
	*/
	unsigned int pNbOfBytesWritten;

	msg << "set profile velocity mode, node = " << p_usNodeId;

	LogInfo(msg.str());

	//Changes the operational mode to "profile velocity mode" ->pg.25 Firmware
	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{		

		stringstream msg;
		msg << "move with target velocity node1= " << targetvelocity_nd_1 << " rpm, node = " << p_usNodeId;
		LogInfo(msg.str());
		msg << "move with target velocity node2 = " << targetvelocity_nd_2 << " rpm, node = " << p_usNodeId;
		LogInfo(msg.str());

		//Loop with timer
		int terminate_measuring = 1;


		auto start_measuring = std::chrono::high_resolution_clock::now();
		
		while(terminate_measuring)
		{
			auto end_measuring = std::chrono::high_resolution_clock::now();
			auto elapsed_time = (end_measuring - start_measuring) /std::chrono::milliseconds(1);

			if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity_nd_1, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
			}
			
			//Call by SDO (No PDO communication by raspberry pi (Linux))
			//Current actual value avaraged (Object = 0x30D1 / 01)
			//Current actual value (Object = 0x30D1 / 02)
			//NbOfBytesToRead = 4
			//sleep(0.5);
			if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x30D1,0x01, &p_CurrentIs, 4,&pNbOfBytesWritten, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
			
			}
			p_CurrentIs_saved.push_back(p_CurrentIs);
			p_Time_saved.push_back(elapsed_time); //ms
			//push ellapsed time too for the figure.
			//std::cout<<"current is (mA)"<<p_CurrentIs<<endl;
			//std::cout<<"Elapsed time "<<elapsed_time<<endl;

			if (elapsed_time >= simtime*1000)//ms// 
			{
				terminate_measuring = 0;
			} 
			else{
				usleep(5000);
			}
		}

		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt velocity movement");

			//stops the movement with profile decleration
			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
		}


		//EPOS Command Library which is only option on Linux, based on SDO data exchange
		//we have to poll the required information by our application code every time. 
		std::cout<<"Total exchanged SDO data number: "<<p_CurrentIs_saved.size()<<endl;
		std::cout<<"avarage elapsed time per meas.  "<<(simtime/p_CurrentIs_saved.size())*1000<<" ms"<<endl;

		//Draw_plot_current_time(&p_CurrentIs_saved,&p_Time_saved);

	}

	return lResult;
}
//If there is error, will inform
int PrepareProfileVelocityMode(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						unsigned int lDeviceErrorCode = 0;
						unsigned int p_rlErrorCode = 0;
						VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId, 1, &lDeviceErrorCode, &p_rlErrorCode);
						std::cout<< std::hex<<"Device Error: 0x"<<lDeviceErrorCode<<" Check Firmware Doc"<<endl;
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

int MaxFollowingErrorDemo(unsigned int& p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	const unsigned int EXPECTED_ERROR_CODE = 0x8611;
	unsigned int lDeviceErrorCode = 0;

	lResult = VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId, &p_rlErrorCode);

	if(lResult)
	{
		lResult = VCS_SetMaxFollowingError(g_pKeyHandle, g_usNodeId, 1, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, 1000, 1, 1, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId, 1, &lDeviceErrorCode, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = lDeviceErrorCode == EXPECTED_ERROR_CODE ? MMC_SUCCESS : MMC_FAILED;
	}

	return lResult;
}

int RunProfileVelocityMode(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	//
	lResult = ProfileVelocityMode(g_pKeyHandle, g_usNodeId, lErrorCode);

	if(lResult != MMC_SUCCESS)
	{
		LogError("ProfileVelocityMode", lResult, lErrorCode);
	}
	else
	{
		//Changes the device state to "disable"
		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
		{
			LogError("VCS_SetDisableState", lResult, lErrorCode);
			lResult = MMC_FAILED;
		}
	}

	return lResult;
}

void PrintHeader()
{
	SeparatorLine();

	LogInfo("Knorr - Bremse sfs AG. IP1747_EMB_2021 Test Bench");

	SeparatorLine();
}

int PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

int PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pInterfaceNameSel;

	return lResult;
}

int PrintDeviceVersion()
{
	int lResult = MMC_FAILED;
	unsigned short usHardwareVersion = 0;
	unsigned short usSoftwareVersion = 0;
	unsigned short usApplicationNumber = 0;
	unsigned short usApplicationVersion = 0;
	unsigned int ulErrorCode = 0;

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
	{
		printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
				g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int PrintAvailableProtocols()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pProtocolNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("protocol stack name = %s\n", pProtocolNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pProtocolNameSel;

	return lResult;
}

int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	PrintHeader();

	SetDefaultParameters();

	if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
	{
		return lResult;
	}

	PrintSettings();
	
	//Open Device and prepare for communication
    if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("OpenDevice", lResult, ulErrorCode);
        return lResult;
    }

    //Check the errors and inform
    if((lResult = PrepareProfileVelocityMode(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("PrepareProfileVelocityMode", lResult, ulErrorCode);
        return lResult;
    }

    if((lResult = RunProfileVelocityMode(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("RunProfileVelocityMode", lResult, ulErrorCode);
        return lResult;
    }

    if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("CloseDevice", lResult, ulErrorCode);
        return lResult;
    }
	
	Draw_plot_current_time(&p_CurrentIs_saved,&p_Time_saved);
	Calculate_averaged_current(p_CurrentIs_saved,p_Time_saved);
	return lResult;
}
