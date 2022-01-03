//2 Maxon Motor Bench Controll Programm
//Load Motor (Velocity Mode- Node1) and Test Motor (Cyclic Synchronus Torque Mode -Node2)
//Integration with CSV files

#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream> // std::stringstream
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
#include <fstream>
#include <utility> 
#include <stdexcept> // std::runtime_error


//Graphs
#include "pbPlots.hpp"
#include "supportLib.hpp"

typedef void* HANDLE;
typedef int BOOL;

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
unsigned short g_usNodeId_1 = 1;
unsigned short g_usNodeId_2 = 2;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
EAppMode g_eAppMode = AM_DEMO;

const string g_programName = "HelloEposCmd";

//Profile Velocity Default Inputs
long targetvelocity_1 = 0; //rpm
long targetvelocity_2 = 0; //rpm
int TargetTorqueNode2 = 0; //1000x * Motor Rated Torque
long double simtime = 0; //sec
/* vector<double> p_CurrentIs_saved;
vector<double> p_Time_saved; */

//TestProfile Variables
float IMaxDrive;
float ImaxBrake; 
float AmountOfCurrentSteps;
float WMaxDrive; 
float WmaxBrake;
float AmountOfVelocitySteps;

int NominalCurrent;
 

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif //

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void SetDefaultParameters(float Wstep, float Istep);
int   ParseArguments(int argc, char** argv);
int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   RunProfileVelocityMode(unsigned int* p_pErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved);
int   RunCyclicTorqueandProfileVelocityMode(unsigned int* p_pErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved, vector<double>p_Velocity_saved);
int   PrepareProfileVelocityMode(unsigned int* p_pErrorCode,unsigned short g_usNodeId_local);
int   PrintAvailableInterfaces();
int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
int	  PrintAvailableProtocols();
int   PrintDeviceVersion();
void  Draw_plot_current_time(vector<double> *plot_current, vector<double>  *plot_time);
void  Calculate_averaged_current(vector<double> plot_current, vector<double>  plot_time);
void  PDO_Mapping(unsigned int *p_pErrorCode,unsigned short g_usNodeId_local);
int   PrepareCyclicTorqueMode(unsigned int* p_pErrorCode,unsigned short g_usNodeId_local);
int  CyclicSynchronusTroqueModeSettings(HANDLE p_DeviceHandle, unsigned short p_usNodeId , unsigned int * p_rlErrorCode, int lResult);
int  ProfileVelocityModeSettings(HANDLE p_DeviceHandle, unsigned short p_usNodeId , unsigned int *p_rlErrorCode,int lResult);
std::vector<std::pair<std::string, std::vector<float>>> ReadCsv_string_int_pair(std::string filename);
std::vector<float> PreapereDataSet_forCurrentStep();
std::vector<float> PreapereDataSet_forVelocityStep();


void PrepareCSV(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> dataset);
void AddData2CSV(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> add_datas);

void AddData2CSV(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> add_datas)
{	
	/* std::ofstream TestProfileFile; */
	std::ofstream TestProfileFile (filename, std::ofstream::out);
	/* TestProfileFile.open(filename); */
	 // Make sure the file is open
	if(!TestProfileFile.is_open()) throw std::runtime_error("Could not open TestProfile.csv file");

	// Send data to the stream
    for(int i = 0; i < add_datas.at(0).second.size(); ++i)
    {
        for(int j = 0; j < add_datas.size(); ++j)
        {
            TestProfileFile << add_datas.at(j).second.at(i);
            if(j != add_datas.size() - 1) TestProfileFile << ";"; // No comma at end of line
        }
        TestProfileFile << "\n";
    }
    
    // Close the file
    TestProfileFile.close();

}

void PrepareCSV(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> dataset)
{
	// Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size
    
    // Create an output filestream object
    std::ofstream myFile(filename);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ";"; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ";"; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();

}

std::vector<float> PreapereDataSet_forVelocityStep()
{
	std::vector<float>  Vstep;
	float Wstep_incrementation = (WMaxDrive + WmaxBrake) / AmountOfVelocitySteps ;
	
	AmountOfVelocitySteps ++; //to take wbrake value
	bool terminate_assign_data;
	for(int i= 0; i < AmountOfVelocitySteps; i ++)
	{
		Vstep.push_back(WMaxDrive - Wstep_incrementation*i);
	}
	//Check there is a 0 amp in the vector?
	
	/* bool there_is_a_zero_amp;
	for(int i= 0; i < AmountOfCurrentSteps; i ++)
	{
		if(Istep.at(i) == 0)
		{
			there_is_a_zero_amp = true;
		}
	}

	bool find_the_right_position_for_zero_amp = 0;
	//If there is no add additional zero amp test.
	if (there_is_a_zero_amp == false)
	{
		for(int i= 0; i < AmountOfCurrentSteps; i ++)
		{
			if (Istep.at(i) < 0 && find_the_right_position_for_zero_amp == false)
			{	
				std::cout<<"Istep.at "<<i<<" value "<<Istep.at(i)<<endl;
				auto iter = Istep.begin();
				iter = Istep.insert(iter+i,0);
				find_the_right_position_for_zero_amp = true;
				AmountOfCurrentSteps ++;
			}
		}
	} */
	
	/* for(int i= 0; i < AmountOfVelocitySteps; i ++)
	{
		std::cout<<" Vstep  : "<<i<< " value  "<< Vstep.at(i)<<endl;
	} */
	return Vstep;
}

std::vector<float> PreapereDataSet_forCurrentStep()
{
	std::vector<float>  Istep;
	float Istep_incrementation = (IMaxDrive + ImaxBrake) / AmountOfCurrentSteps ;
	
	bool terminate_assign_data; //
	AmountOfCurrentSteps ++; //to take also min value
	for(int i= 0; i < AmountOfCurrentSteps; i ++)
	{
		Istep.push_back(IMaxDrive - Istep_incrementation*i);
	}
	//Check there is a 0 amp in the vector?
	
	bool there_is_a_zero_amp;
	for(int i= 0; i < AmountOfCurrentSteps; i ++)
	{
		if(Istep.at(i) == 0)
		{
			there_is_a_zero_amp = true;
		}
	}

	bool find_the_right_position_for_zero_amp = 0;
	//If there is no add additional zero amp test.
	if (there_is_a_zero_amp == false)
	{
		for(int i= 0; i < AmountOfCurrentSteps; i ++)
		{
			if (Istep.at(i) < 0 && find_the_right_position_for_zero_amp == false)
			{	
				std::cout<<"Istep.at "<<i<<" value "<<Istep.at(i)<<endl;
				auto iter = Istep.begin();
				iter = Istep.insert(iter+i,0);
				find_the_right_position_for_zero_amp = true;
				AmountOfCurrentSteps ++;
			}
		}
	}
	
	/* for(int i= 0; i < AmountOfCurrentSteps; i ++)
	{
		std::cout<<" Istep  : "<<i<< " value  "<< Istep.at(i)<<endl;
	} */
	return Istep;
}

std::vector<std::pair<std::string, std::vector<float>>> ReadCsv_string_int_pair(std::string filename)
{
	// Reads a CSV file into a vector of <string, vector<*int*>> pairs where //Fix -> use float type
    // each pair represents <column name, column values>
	// Create a vector of <string, float vector> pairs to store the result -> TestData
	std::vector<std::pair<std::string, std::vector<float>>> TestData;
	// Create an input filestream
	std::ifstream TestProfileFile(filename);
	 // Make sure the file is open
	if(!TestProfileFile.is_open()) throw std::runtime_error("Could not open TestProfile.csv file");
	
	//HelperVariables
	std::string line,colname;
	float val;
	
	//Read the colmn names
	if (TestProfileFile.good())
	{
		// Extract the first line in the file
        std::getline(TestProfileFile, line);

		 // Create a stringstream from line
        std::stringstream StreamLine(line);

		//Extract the each column name
		while(std::getline(StreamLine, colname, ';')) 
		{
        	// Initialize and add <colname, int vector> pairs to result
        	TestData.push_back({colname, std::vector<float> {} } ); //Adds a new element at the end of the vector
        }
	}
	// Read data, line by line
	while(std::getline(TestProfileFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream StreamLine(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        // Extract each integer
        while(StreamLine >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            TestData.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(StreamLine.peek() == ';') 
			{
				StreamLine.ignore();
			}
            // Increment the column index
            colIdx++;
        }
    }

	TestProfileFile.close();

	/* std::cout <<TesProfileDatas.size()<<endl;
	std::cout <<TesProfileDatas.at(3).first<<endl; //all string of 3. element of cover vector (pair)
	std::cout <<TesProfileDatas.at(3).first.at(3)<<endl; //all string of 3. element of pair
	std::cout <<TesProfileDatas.at(1).second.at(0)<<endl;  */

	IMaxDrive = TestData.at(0).second.at(0);
	ImaxBrake = TestData.at(1).second.at(0);
	AmountOfCurrentSteps= TestData.at(2).second.at(0);
	WMaxDrive = TestData.at(3).second.at(0);
	WmaxBrake = TestData.at(4).second.at(0);
	AmountOfVelocitySteps= TestData.at(5).second.at(0);
	simtime = TestData.at(6).second.at(0);
	
	std::cout<<"IMaxDrive "<< IMaxDrive<< endl;
	std::cout<<"ImaxBrake "<<ImaxBrake << endl;
	std::cout<<"AmountOfCurrentSteps "<<AmountOfCurrentSteps << endl;
	std::cout<<"WMaxDrive "<< WMaxDrive<< endl;
	std::cout<<"WmaxBrake "<< WmaxBrake<< endl;
	std::cout<<"AmountOfVelocitySteps"<<AmountOfVelocitySteps<< endl;

	return TestData;

}

int CyclicSynchronusTroqueModeSettings(HANDLE p_DeviceHandle, unsigned short p_usNodeId , unsigned int * p_rlErrorCode, int lResult)
{
	unsigned int pNbOfBytesWritten;
	//Check Default Object Values
	//Indicates the configured input value for the torque controller in Cyclic Synchronous Torque Mode. The value 
	//s given in per thousand of “Motor rated torque” on page 6-231).
	/* int TargetTorque;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6071,0x00, &TargetTorque, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	}
	else
	{
		std::cout<<" TargetTorque (CyclicSynchronusTroqueModeSettings) (% Nominal Current)  :"<<TargetTorque<<endl;
	} */
	//  (Nominal torque (max. continuous torque)	1010 mNm)

	// Target torque is 1000x Motor rated torque which is multiplication of Nominal Current and Motor Torque Constant
	int MotorRatedTorque;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6076,0x00, &MotorRatedTorque, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	}
	else
	{
		std::cout<<" MotorRatedTorque (CyclicSynchronusTroqueModeSettings)  :"<<MotorRatedTorque<<endl;
	}
	//918426 µNm  = 75903 µNm/A * 12,1 A =  918,426 mNm

	//Get Nominal Current
	
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x3001,0x01, &NominalCurrent, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	}
	else
	{
		std::cout<<" NominalCurrent (CyclicSynchronusTroqueModeSettings)  :"<<NominalCurrent<<endl;
	}
	//12,1 A

	//Get Torque Constant
	int TorqueConstant;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x3001,0x05, &TorqueConstant, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	}
	else
	{
		std::cout<<" TorqueConstant (CyclicSynchronusTroqueModeSettings)  :"<<TorqueConstant<<endl;
	}
	//75903 µNm/A

	//Get Operation Mode (A)
	int opMode;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6061,0x00, &opMode, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" OpMode (CyclicSynchronusTroqueModeSettings)  :"<<opMode<<endl;
	}

	int CurrentModeSettingValue;
	if(VCS_GetCurrentMustEx(p_DeviceHandle, p_usNodeId,&CurrentModeSettingValue, p_rlErrorCode) == 0)
	{
		LogError("VCS_SetCurrentMustEx", lResult, *p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		std::cout<<" VCS_GetCurrentMustEx (CyclicSynchronusTroqueModeSettings)  :"<<CurrentModeSettingValue<<endl;
	}


	return lResult;
}

int ProfileVelocityModeSettings(HANDLE p_DeviceHandle, unsigned short p_usNodeId , unsigned int *p_rlErrorCode, int lResult)
{	
	unsigned int pNbOfBytesWritten;
	//Check Default Object Values
	//Get Operation Mode (A) see->Application Note 7.5 Profile Velocity Mode
	int opMode;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6061,0x00, &opMode, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6061", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" OpMode (ProfileVelocityModeSettings)  :"<<opMode<<endl;
	}
	
	//Get Parameter (B)
	int MaxProfileVelocity;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x607F,0x00, &MaxProfileVelocity, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x607F", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" MaxProfileVelocity (ProfileVelocityModeSettings)  :"<<MaxProfileVelocity<<endl;
	}

	int ProfileAcceleration;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6083,0x00, &ProfileAcceleration, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6083", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" ProfileAcceleration (ProfileVelocityModeSettings)  :"<<ProfileAcceleration<<endl;
	}

	int ProfileDeceleration;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6084,0x00, &ProfileDeceleration, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6084", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" ProfileDeceleration (ProfileVelocityModeSettings)  :"<<ProfileDeceleration<<endl;
	}

	int QuickStopDeceleration;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6085,0x00, &QuickStopDeceleration, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6085", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" QuickStopDeceleration (ProfileVelocityModeSettings)  :"<<QuickStopDeceleration<<endl;
	}

	int MotionProfileType;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6086,0x00, &MotionProfileType, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6086", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" MotionProfileType (ProfileVelocityModeSettings)  :"<<MotionProfileType<<endl;
	}

	//Enable Device (C=)
	int Controlword;
	if(VCS_GetObject(p_DeviceHandle, p_usNodeId, 0x6040,0x00, &Controlword, 4,&pNbOfBytesWritten, p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetObject 0x6040", lResult, *p_rlErrorCode);
	
	}
	else
	{
		std::cout<<" Controlword (ProfileVelocityModeSettings)  :"<<Controlword<<endl;
	}
	
	return lResult;
}

//PDO mapping could used y two nodes. Thats why g_usNodeID_local inculdes
void PDO_Mapping(unsigned int *p_pErrorCode,unsigned short g_usNodeId_local)
{	
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;
	
	//TxPDO1
	//COB-ID
	unsigned int long g_COBID;
	unsigned int pNbOfBytesWritten_PDO;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1800,0x01, &g_COBID, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}

	std::cout<< std::hex<<"COBID_TxPDO1 0x"<<g_COBID<<endl;

	//Transmission Type
	unsigned int transs_type = 0;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1800,0x02, &transs_type, 1,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Transmission type TxPDO1  %u \n" , (unsigned)transs_type);

	//Inhibit Time
	unsigned int inhibit_time = 0;
	//x100 microsecond
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1800,0x03, &inhibit_time, 2,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Inhibit Time TxPDO1  %u \n" , (unsigned)inhibit_time/10);

	
	/*
	//Number of mapped objects
	unsigned int mapped_objects;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A00,0x00, &mapped_objects, 1,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Number of mapped objects TxPDO1  %u \n" , (unsigned)mapped_objects);
	*/

	
	//Result = 2
	//1st mapped object TxPDO1
	unsigned int long first_mapped;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A00,0x01, &first_mapped, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 1 TxPDO1 0x"<<first_mapped<<endl;
	
	//2st mapped object TxPDO1
	unsigned int long second_mapped;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A00,0x02, &second_mapped, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 2 TxPDO1 0x"<<second_mapped<<endl;

	//3rd mapped object TxPDO1
	unsigned int long thirth_mapped;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A00,0x03, &thirth_mapped, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 3 TxPDO1 0x"<<thirth_mapped<<endl;

	//-------------------------------------------------------------------------------------------------------------------------
	//TxPDO2
	//COB-ID
	unsigned int long g_COBID_2;
	unsigned int pNbOfBytesWritten_PDO_2;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1801,0x01, &g_COBID_2, 4,&pNbOfBytesWritten_PDO_2, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}

	std::cout<< std::hex<<"COBID_TxPDO2 0x"<<g_COBID_2<<endl;

	//Transmission Type
	unsigned int transs_type_2 = 0;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1801,0x02, &transs_type_2, 1,&pNbOfBytesWritten_PDO_2, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Transmission type TxPDO2  %u \n" , (unsigned)transs_type_2);

	//Inhibit Time
	unsigned int inhibit_time_2 = 0;
	//x100 microsecond
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1801,0x03, &inhibit_time_2, 2,&pNbOfBytesWritten_PDO_2, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Inhibit Time TxPDO2  %u \n" , (unsigned)inhibit_time_2/10);

	
	/* 
	//Number of mapped objects
	unsigned int mapped_objects;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A01,0x00, &mapped_objects, 1,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Number of mapped objects TxPDO2  %u \n" , (unsigned)mapped_objects);
	 */

	
	//Result = 2
	//1st mapped object TxPDO1
	unsigned int long first_mapped_2;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A01,0x01, &first_mapped_2, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 1 TxPDO2 0x"<<first_mapped_2<<endl;
	
	//2st mapped object TxPDO1
	unsigned int long second_mapped_2;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A01,0x02, &second_mapped_2, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 2 TxPDO2 0x"<<second_mapped_2<<endl;

	//3rd mapped object TxPDO1
	unsigned int long thirth_mapped_2;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A01,0x03, &thirth_mapped_2, 4,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 3 TxPDO2 0x"<<thirth_mapped_2<<endl;

	//-------------------------------------------------------------------------------------------------------------------------
	//TxPDO3
	//COB-ID
	unsigned int long g_COBID_3;
	unsigned int pNbOfBytesWritten_PDO_3;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1802,0x01, &g_COBID_3, 4,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}

	std::cout<< std::hex<<"COBID_TxPDO3 0x"<<g_COBID_3<<endl;

	//Transmission Type
	unsigned int transs_type_3 = 0;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1802,0x02, &transs_type_3, 1,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Transmission type TxPDO3  %u \n" , (unsigned)transs_type_3);

	//Inhibit Time
	unsigned int inhibit_time_3 = 0;
	//x100 microsecond
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1802,0x03, &inhibit_time_3, 2,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Inhibit Time TxPDO3  %u \n" , (unsigned)inhibit_time_3/10);

	
	/*
	//Number of mapped objects
	unsigned int mapped_objects;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A02,0x00, &mapped_objects, 1,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Number of mapped objects TxPDO2  %u \n" , (unsigned)mapped_objects);
	*/

	
	//Result = 2
	//1st mapped object TxPDO1
	unsigned int long first_mapped_3;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A02,0x01, &first_mapped_3, 4,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 1 TxPDO3 0x"<<first_mapped_3<<endl;
	
	//2st mapped object TxPDO1
	unsigned int long second_mapped_3;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A02,0x02, &second_mapped_3, 4,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 2 TxPDO3 0x"<<second_mapped_3<<endl;

	//3rd mapped object TxPDO1
	unsigned int long thirth_mapped_3;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A02,0x03, &thirth_mapped_3, 4,&pNbOfBytesWritten_PDO_3, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 3 TxPDO3 0x"<<thirth_mapped_3<<endl;
	
	//-------------------------------------------------------------------------------------------------------------------------
	//TxPDO4
	//COB-ID
	unsigned int long g_COBID_4;
	unsigned int pNbOfBytesWritten_PDO_4;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1803,0x01, &g_COBID_4, 4,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}

	std::cout<< std::hex<<"COBID_TxPDO4 0x"<<g_COBID_4<<endl;

	//Transmission Type
	unsigned int transs_type_4 = 0;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1803,0x02, &transs_type_4, 1,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Transmission type TxPDO4  %u \n" , (unsigned)transs_type_4);

	//Inhibit Time
	unsigned int inhibit_time_4 = 0;
	//x100 microsecond
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1803,0x03, &inhibit_time_4, 2,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Inhibit Time TxPDO4  %u \n" , (unsigned)inhibit_time_4/10);

	
	/*
	//Number of mapped objects
	unsigned int mapped_objects;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A03,0x00, &mapped_objects, 1,&pNbOfBytesWritten_PDO, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	printf("Number of mapped objects TxPDO2  %u \n" , (unsigned)mapped_objects);
	*/

	
	//Result = 2
	//1st mapped object TxPDO1
	unsigned int long first_mapped_4;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A03,0x01, &first_mapped_4, 4,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 1 TxPDO4 0x"<<first_mapped_4<<endl;
	
	//2st mapped object TxPDO1
	unsigned int long second_mapped_4;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A03,0x02, &second_mapped_4, 4,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 2 TxPDO4 0x"<<second_mapped_4<<endl;

	//3rd mapped object TxPDO1
	unsigned int long thirth_mapped_4;
	if(VCS_GetObject(g_pKeyHandle, g_usNodeId_local, 0x1A03,0x03, &thirth_mapped_4, 4,&pNbOfBytesWritten_PDO_4, p_pErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, *p_pErrorCode);
	
	}
	std::cout<< std::hex<<"Mapped Object 3 TxPDO4 0x"<<thirth_mapped_4<<endl;

}

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
        WriteToFile(pngdata, "currentxtime" + std::to_string(targetvelocity_1)+ " rpm"+ ".png");
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
	cout << "Profile velocity Mode Settings-------------------------------------------------------------------" << endl;
	cout << "\t-q : target torque % " << endl;
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
	msg << "node id.1(test motor)    = " << g_usNodeId_1 << endl;
	msg << "node id.2(load motor)    = " << g_usNodeId_2 << endl;
	msg << "device name              = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name      = '" << g_protocolStackName << "'" << endl;
	msg << "interface name           = '" << g_interfaceName << "'" << endl;
	msg << "port name                = '" << g_portName << "'"<< endl;
	msg << "baudrate                 = " << g_baudrate<<endl;

	msg << "Profile velocity Mode Parameters:"<<endl;
	msg << "target velocity_Node1     = " << targetvelocity_1 << "(rpm)"<<endl;
	msg << "target torque (% Nominal Current)  = " << TargetTorqueNode2 <<endl;
	msg << "simulation time     = " << simtime << "(sec)"<<endl;



	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters(float Wstep, float Istep)
{
	/*
	//USB
	g_usNodeId_1 = 1;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_portName = "USB0"; 
	g_baudrate = 1000000;
	*/

	//CAN
	g_usNodeId_1 = 1;
	g_usNodeId_2 = 2;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "CANopen"; 
	g_interfaceName = "CAN_mcp251x 0"; 
	g_portName = "CAN0"; 
	g_baudrate = 250000; 
	targetvelocity_1 = Wstep; //rpm
	TargetTorqueNode2 = ((Istep*1000)*100) / 12100; //The value is given in per thousand of “Motor rated torque” on page 6-231).
	std::cout<<Istep<<" Target TorqueNode2"<<TargetTorqueNode2<<"  "<<NominalCurrent<<endl;
	/* simtime = 5; */ //sec


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

	while((lOption = getopt(argc, argv, "hlrvd:s:i:p:b:n:m:x:q:y:")) != -1)
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
				g_usNodeId_1 = (unsigned short)atoi(optarg);
				break;
			case 'm':
				g_usNodeId_2 = (unsigned short)atoi(optarg);
				break;
			case 'x':
				targetvelocity_1 = atoi(optarg);
				break;
			case 'q':
				TargetTorqueNode2 = atoi(optarg);
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

bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId_1_local, unsigned short p_usNodeId_2_local, unsigned int & p_rlErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	int p_CurrentIs;
	/*
	vector<double> p_CurrentIs_saved;
	vector<double> p_Time_saved;
	*/
	unsigned int pNbOfBytesWritten;

	msg << "set profile velocity mode, node = " << p_usNodeId_1_local<<" and "<<p_usNodeId_2_local;

	LogInfo(msg.str());

	//Changes the operational mode to "profile velocity mode" ->pg.25 Firmware
	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId_1_local, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode_Node1", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	
	//VCS_GetOperationMode doesnt work!


	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId_2_local, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode_Node2", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}


	else
	{		

		stringstream msg;
		msg << "move with target velocity = " << targetvelocity_1 << " rpm, node = " << p_usNodeId_1_local<<endl;
		msg << "move with target velocity = " << targetvelocity_2 << " rpm, node = " << p_usNodeId_2_local<<endl;
		LogInfo(msg.str());


		//Loop with timer
		int terminate_measuring = 1;


		auto start_measuring = std::chrono::high_resolution_clock::now();
		if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId_2_local, -targetvelocity_2, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity_Node2", lResult, p_rlErrorCode);
			}
		
		if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId_1_local, targetvelocity_1, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity_Node1", lResult, p_rlErrorCode);
			}
		
		
		while(terminate_measuring)
		{
			auto end_measuring = std::chrono::high_resolution_clock::now();
			auto elapsed_time = (end_measuring - start_measuring) /std::chrono::milliseconds(1);

			/* if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity_1, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
			} */
			
			//Call by SDO (No PDO communication by raspberry pi (Linux))
			//Current actual value avaraged (Object = 0x30D1 / 01)
			//Current actual value (Object = 0x30D1 / 02)
			//NbOfBytesToRead = 4
			//sleep(0.5);
			if(VCS_GetObject(p_DeviceHandle, p_usNodeId_2_local, 0x30D1,0x01, &p_CurrentIs, 4,&pNbOfBytesWritten, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_GetObject ox30d1", lResult, p_rlErrorCode);
			
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
			LogInfo("halt velocity movement Node1 and Node2");

			//stops the movement with profile decleration
			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId_1_local, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement_Node1", lResult, p_rlErrorCode);
			}

			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId_2_local, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement_Node2", lResult, p_rlErrorCode);
			}
		}


		//EPOS Command Library which is only option on Linux, based on SDO data exchange
		//we have to poll the required information by our application code every time. 
		std::cout<<"Total exchanged SDO data number: "<<std::dec<<p_CurrentIs_saved.size()<<endl;
		std::cout<<"avarage elapsed time per meas.  "<<std::dec<<(simtime/p_CurrentIs_saved.size())*1000<<" ms"<<endl;

		//Draw_plot_current_time(&p_CurrentIs_saved,&p_Time_saved);

	}

	return lResult;
}

bool CyclicTorqueandProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId_1_local, unsigned short p_usNodeId_2_local, unsigned int & p_rlErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved,vector<double>p_Velocity_saved)
{	
	int lResult = MMC_SUCCESS;
	int TargetTorqueSampled = TargetTorqueNode2 * 10 ;
	stringstream msg;
	int p_CurrentIs;
	unsigned int pProfileAccelerationN2;
	unsigned int pProfileDecelerationN2;
	int  pCurrentMust= 1;
	/*
	vector<double> p_CurrentIs_saved;
	vector<double> p_Time_saved;
	*/
	unsigned int pNbOfBytesWritten;
	msg << "set profile velocity mode, node = " << p_usNodeId_1_local<<" and "<<p_usNodeId_2_local;
	LogInfo(msg.str());

	//VCS_ActivateCurrentMode changes the operational mode to “current mode”
	if(VCS_ActivateCurrentMode(p_DeviceHandle, p_usNodeId_2_local , &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode_Node1", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	
	//VCS_GetOperationMode doesnt work!
	//Changes the operational mode to "profile velocity mode" ->pg.25 Firmware
	//Load Motor in ProfileVelecotiyMode
	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId_1_local, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode_Node2", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		//VCS_SetCurrentMust writes current mode setting value
		if(VCS_SetCurrentMustEx(p_DeviceHandle, p_usNodeId_1_local,1, &p_rlErrorCode) == 0)
		{
			LogError("VCS_SetCurrentMustEx", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
		}
		else
		{	
			
			lResult = CyclicSynchronusTroqueModeSettings(p_DeviceHandle, p_usNodeId_2_local , &p_rlErrorCode, lResult);
			lResult = ProfileVelocityModeSettings(p_DeviceHandle, p_usNodeId_1_local , &p_rlErrorCode, lResult);

			stringstream msg;
			msg << "move with target velocity = " << targetvelocity_1 << " rpm, node = " << p_usNodeId_1_local<<endl;
			msg << "set the current  = " << TargetTorqueNode2 << " (% Nominal Current)  , node = " << p_usNodeId_2_local<<endl;
			LogInfo(msg.str());
			//Loop with timer
			int terminate_measuring = 1;
			auto start_measuring = std::chrono::high_resolution_clock::now();
			if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId_1_local, targetvelocity_1, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_MoveWithVelocity_Node2", lResult, p_rlErrorCode);
			}

			//Set target torque to node 2 //Multiplied to equal EPOS STUDIO APP
			if(VCS_SetObject(p_DeviceHandle, p_usNodeId_2_local, 0x6071,0x00, &TargetTorqueSampled, 4,&pNbOfBytesWritten, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_GetObject 0x6061", lResult, p_rlErrorCode);
			}
			else
			{
				std::cout<<" Torque Setted on Node 2 (% Nominal Current)  :"<<endl;
			}

			int Velocity_Actual_Avaraged;
			while(terminate_measuring)
			{
				auto end_measuring = std::chrono::high_resolution_clock::now();
				auto elapsed_time = (end_measuring - start_measuring) /std::chrono::milliseconds(1);
				/* if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity_1, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
				} */
				
				//Call by SDO (No PDO communication by raspberry pi (Linux))
				//Current actual value avaraged (Object = 0x30D1 / 01)
				//Current actual value (Object = 0x30D1 / 02)
				//NbOfBytesToRead = 4
				//sleep(0.5);
				if(VCS_GetObject(p_DeviceHandle, p_usNodeId_2_local, 0x30D1,0x01, &p_CurrentIs, 4,&pNbOfBytesWritten, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					LogError("VCS_GetObject ox30d1", lResult, p_rlErrorCode);
				
				}
				//---------Experiment with Olivia
				if(VCS_GetObject(p_DeviceHandle, p_usNodeId_2_local, 0x30D3,0x01, &Velocity_Actual_Avaraged, 4,&pNbOfBytesWritten, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					LogError("VCS_GetObject 0x6061", lResult, p_rlErrorCode);
				}
				else
				{
					/* std::cout<<" Velocity Actual Avaraged (in the loop)  :"<<Velocity_Actual_Avaraged<<endl; */
				}



				p_CurrentIs_saved.push_back(p_CurrentIs);
				p_Time_saved.push_back(elapsed_time);
				p_Velocity_saved.push_back(Velocity_Actual_Avaraged); //ms
				//push ellapsed time too for the figure.
				//std::cout<<"current is (mA)"<<p_CurrentIs<<endl;
				//std::cout<<"Elapsed time "<<elapsed_time<<endl;
				if (elapsed_time >= simtime*1000)//ms// 
				{
					terminate_measuring = 0;
				} 
				else
				{
					usleep(5000);
				}
			}
		}	
		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt velocity movement Node1 and Node2");
			//stops the movement with profile decleration
			/* if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId_1_local, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement_Node1", lResult, p_rlErrorCode);
			} */
			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId_2_local, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement_Node2", lResult, p_rlErrorCode);
			}
		}
		//EPOS Command Library which is only option on Linux, based on SDO data exchange
		//we have to poll the required information by our application code every time. 
		std::cout<<"Total exchanged SDO data number: "<<std::dec<<p_CurrentIs_saved.size()<<endl;
		std::cout<<"avarage elapsed time per meas.  "<<std::dec<<(simtime/p_CurrentIs_saved.size())*1000<<" ms"<<endl;
		//Draw_plot_current_time(&p_CurrentIs_saved,&p_Time_saved);
	}
	return lResult;
}
//If there is error, will inform
int  PrepareProfileVelocityMode(unsigned int* p_pErrorCode,unsigned short g_usNodeId_local)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId_local, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	//Should be in preoperational state---PDO mapping
	//PDO_Mapping(p_pErrorCode, g_usNodeId_local);

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId_local << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId_local, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;
			//VCS_GetEnableState checks if the device is enabled. 
			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId_local, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId_local, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						unsigned int lDeviceErrorCode = 0;
						unsigned int p_rlErrorCode = 0;
						VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId_local, 1, &lDeviceErrorCode, &p_rlErrorCode);
						std::cout<< std::hex<<"Device Error: 0x"<<lDeviceErrorCode<<" Check Firmware Doc"<<endl;
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

int  PrepareCyclicTorqueMode(unsigned int* p_pErrorCode,unsigned short g_usNodeId_local)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId_local, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	//Should be in preoperational state---PDO mapping
	//PDO_Mapping(p_pErrorCode, g_usNodeId_local);

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId_local << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId_local, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}
		
		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId_local, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			char OpMode;
			

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId_local, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						unsigned int lDeviceErrorCode = 0;
						unsigned int p_rlErrorCode = 0;
						VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId_local, 1, &lDeviceErrorCode, &p_rlErrorCode);
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

	lResult = VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId_1, &p_rlErrorCode);

	if(lResult)
	{
		lResult = VCS_SetMaxFollowingError(g_pKeyHandle, g_usNodeId_1, 1, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = VCS_MoveToPosition(g_pKeyHandle, g_usNodeId_1, 1000, 1, 1, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId_1, 1, &lDeviceErrorCode, &p_rlErrorCode);
	}

	if(lResult)
	{
		lResult = lDeviceErrorCode == EXPECTED_ERROR_CODE ? MMC_SUCCESS : MMC_FAILED;
	}

	return lResult;
}

int RunProfileVelocityMode(unsigned int* p_pErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved)

{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	//
	lResult = ProfileVelocityMode(g_pKeyHandle, g_usNodeId_1,g_usNodeId_2, lErrorCode, p_CurrentIs_saved, p_Time_saved);
	//std::cout<<"ProfileVelocityMode Node Check Node1 "<<g_usNodeId_1<<"Node2 "<<g_usNodeId_2<< "tv1 "<<targetvelocity_1<<"tv2 "<<targetvelocity_2<<endl;


	if(lResult != MMC_SUCCESS)
	{
		LogError("ProfileVelocityMode_Node1&Node2", lResult, lErrorCode);
	}
	else
	{
		//Changes the device state to "disable"
		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId_2, &lErrorCode) == 0)
		{
			LogError("VCS_SetDisableState_Node1", lResult, lErrorCode);
			lResult = MMC_FAILED;
		}


		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId_1, &lErrorCode) == 0)
		{
			LogError("VCS_SetDisableState_Node2", lResult, lErrorCode);
			lResult = MMC_FAILED;
		}
	}

	return lResult;
}

int RunCyclicTorqueandProfileVelocityMode(unsigned int* p_pErrorCode, vector<double> p_CurrentIs_saved, vector<double> p_Time_saved, vector<double>p_Velocity_saved)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	//
	lResult = CyclicTorqueandProfileVelocityMode(g_pKeyHandle, g_usNodeId_1,g_usNodeId_2, lErrorCode, p_CurrentIs_saved,  p_Time_saved, p_Velocity_saved);
	//std::cout<<"ProfileVelocityMode Node Check Node1 "<<g_usNodeId_1<<"Node2 "<<g_usNodeId_2<< "tv1 "<<targetvelocity_1<<"tv2 "<<targetvelocity_2<<endl;


	if(lResult != MMC_SUCCESS)
	{
		LogError("ProfileVelocityMode_Node1&Node2", lResult, lErrorCode);
	}
	else
	{
		//Changes the device state to "disable"
		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId_1, &lErrorCode) == 0)
		{
			LogError("VCS_SetDisableState_Node1", lResult, lErrorCode);
			lResult = MMC_FAILED;
		}


		if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId_2, &lErrorCode) == 0)
		{
			LogError("VCS_SetDisableState_Node2", lResult, lErrorCode);
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

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId_1, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
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

	//Take Datas from csv file "TestProfile.csv"
	std::vector<std::pair<std::string, std::vector<float>>> TesProfileDatas;
	TesProfileDatas = ReadCsv_string_int_pair("TestProfile.csv");
	
	std::vector<float> Istep = PreapereDataSet_forCurrentStep();
	std::vector<float> Wstep = PreapereDataSet_forVelocityStep();
	
	//Automated Test Sets Starts Here
	for (int i = 0 ; i < Wstep.size(); i ++)
	{
		std::string filename_saved = "test_data_" + std::to_string(Wstep.at(i))+ " rpm"+ ".csv";
		for(int j = 0 ; j < Istep.size(); j ++)
		{	
			vector<double> p_CurrentIs_saved;
			vector<double> p_Time_saved;
			vector<double> p_Velocity_saved;

			SetDefaultParameters(Wstep.at(i), Istep.at(j));
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
			if((lResult = PrepareCyclicTorqueMode(&ulErrorCode,g_usNodeId_2))!=MMC_SUCCESS)
			{
				LogError("PrepareProfileVelocityMode_Node1", lResult, ulErrorCode);
				return lResult;
			}

			if((lResult = PrepareProfileVelocityMode(&ulErrorCode,g_usNodeId_1))!=MMC_SUCCESS)
			{
				LogError("PrepareProfileVelocityMode_Node1", lResult, ulErrorCode);
				return lResult;
			}
			
			/* if((lResult = PrepareProfileVelocityMode(&ulErrorCode,g_usNodeId_1))!=MMC_SUCCESS)
			{
				LogError("PrepareProfileVelocityMode_Node2", lResult, ulErrorCode);
				return lResult;
			} */

			if((lResult = RunCyclicTorqueandProfileVelocityMode(&ulErrorCode, p_CurrentIs_saved, p_Time_saved, p_Velocity_saved))!=MMC_SUCCESS)
			{
				LogError("RunProfileVelocityMode", lResult, ulErrorCode);
				return lResult;
			}

			/* if((lResult = RunProfileVelocityMode(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("RunProfileVelocityMode", lResult, ulErrorCode);
				return lResult;
			} */

			if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("CloseDevice", lResult, ulErrorCode);
				return lResult;
			}
			
			/* Draw_plot_current_time(&p_CurrentIs_saved,&p_Time_saved);
			Calculate_averaged_current(p_CurrentIs_saved,p_Time_saved); */
			
			//Prepare Test data
			//Wrap data to vector pairs
		
			std::vector<double> Index(p_Time_saved.size()) ; 
			std::iota (std::begin(Index), std::end(Index), 1); // Fill with 0, 1, ...

			std::vector<double> Ides(p_Time_saved.size(), Istep.at(j));
			

			if (j == 0)
			{
				std::vector<std::pair<std::string, std::vector<double>>> wrapped_datas = {{"Index", Index}, {"Iact", p_CurrentIs_saved}, {"Ides", Ides}, {"Wact", p_Velocity_saved}, {"Tact", p_Time_saved}, {"Time", p_Time_saved}};
				PrepareCSV(filename_saved, wrapped_datas); 

			}
			else
			{
				std::vector<std::pair<std::string, std::vector<double>>> add_datas = {{"Index", Index}, {"Iact", p_CurrentIs_saved}, {"Ides", Ides}, {"Wact", p_Velocity_saved}, {"Tact", p_Time_saved}, {"Time", p_Time_saved}};
				AddData2CSV(filename_saved, add_datas);
			}
			
		}
		
	}

	return lResult;
}
