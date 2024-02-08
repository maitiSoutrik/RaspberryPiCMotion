/// PMDApp.c is the application level file for the RaspberryPi / Linux serial example.
/// However there is nothing Linux specific in this file.  That is in PMDRaspianSer.c
/// This was tested on a Raspberry Pi 3 B+.
/// Hint: Raspberry Pi user's must disable serial debug messages first (raspi-config)
/// TLK 8/24/2021

#define PMD_W32SERIAL_INTERFACE


#include <stdio.h>
#include "C-Motion.h"
#include "PMDutil.h"
#include "PMDdiag.h"
#include "Examples.h"



PMDresult SetupAxis1(PMDAxisHandle* phAxis)
{
PMDresult result = PMD_NOERROR;

  // PMD_RESULT(PMDSetSampleTime (phAxis, 51))
  // PMD_RESULT(PMDSetSPIMode (phAxis, 0))
  // PMD_RESULT(PMDSetSerialPortMode (phAxis, 4, 0, 0, 0, 0))
  // PMD_RESULT(PMDSetCANMode (phAxis, 0, 6))
  PMD_RESULT(PMDSetOperatingMode (phAxis, 1))
  PMD_RESULT(PMDResetEventStatus (phAxis, 0x0000))
  PMD_RESULT(PMDSetMotorType (phAxis, 0))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMDeadTime, 550))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMSignalSense, 33023))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMRefreshPeriod, 8))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMRefreshTime, 2000))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMCurrentSenseTime, 2000))
  PMD_RESULT(PMDSetOutputMode (phAxis, 7))
  PMD_RESULT(PMDSetProfileMode (phAxis, 0))
  PMD_RESULT(PMDSetPositionErrorLimit (phAxis, 65535))
  PMD_RESULT(PMDSetSettleTime (phAxis, 0))
  PMD_RESULT(PMDSetSettleWindow (phAxis, 0))
  PMD_RESULT(PMDSetTrackingWindow (phAxis, 0))
  PMD_RESULT(PMDSetEncoderSource (phAxis, 0))
  PMD_RESULT(PMDSetGearMaster (phAxis, 0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKp, 100))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKi, 20))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopIlimit, 10000000))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKd, 2000))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopDerivativeTime, 1))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKout, 3000))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKvff, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKaff, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1Enable, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1A1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1A2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1K, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2Enable, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2A1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2A2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2K, 0))
  PMD_RESULT(PMDSetMotorBias (phAxis, 0))
  PMD_RESULT(PMDSetMotorLimit (phAxis, 12278))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 3277))
  PMD_RESULT(PMDSetMotionCompleteMode (phAxis, 0))
  PMD_RESULT(PMDSetSignalSense (phAxis, 0x0000))
  PMD_RESULT(PMDSetCaptureSource (phAxis, 0))
  PMD_RESULT(PMDSetPhaseCounts (phAxis, 1000))
  PMD_RESULT(PMDSetCommutationMode (phAxis, 0))
  PMD_RESULT(PMDSetPhaseCorrectionMode (phAxis, 1))
  PMD_RESULT(PMDSetPhaseInitializeMode (phAxis, 0))
  PMD_RESULT(PMDSetPhaseInitializeTime (phAxis, 14706))
  PMD_RESULT(PMDSetPhasePrescale (phAxis, 0))
  PMD_RESULT(PMDSetBreakpointValue (phAxis, 0x0000, 0x00000000))
  PMD_RESULT(PMDSetBreakpointValue (phAxis, 0x0001, 0x00000000))
  PMD_RESULT(PMDSetBreakpoint (phAxis, 0x0000, 0, 0, 0))
  PMD_RESULT(PMDSetBreakpoint (phAxis, 0x0001, 0, 0, 0))
  PMD_RESULT(PMDSetAuxiliaryEncoderSource (phAxis, 0, 0))
  PMD_RESULT(PMDSetPWMFrequency (phAxis, 5000))
  PMD_RESULT(PMDSetCurrentControlMode (phAxis, 1))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 0, 224))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 1, 29))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 2, 16383))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 0, 0))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 1, 0))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 2, 16384))
  PMD_RESULT(PMDSetAxisOutMask (phAxis, 0, 0, 0x0001, 0x0000))
  PMD_RESULT(PMDSetFaultOutMask (phAxis, 0x0600))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterOvervoltageLimit, 56943))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterUndervoltageLimit, 10353))
  PMD_RESULT(PMDSetCurrentFoldback (phAxis, PMDCurrentFoldbackContinuousCurrentLimit, 6820))
  PMD_RESULT(PMDSetCurrentFoldback (phAxis, PMDCurrentFoldbackI2tEnergyLimit, 2116))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventPositiveLimit, 0x0000))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventNegativeLimit, 0x0000))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventMotionError, 0x0005))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventCurrentFoldback, 0x0007))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMLimit, 16384))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterTemperatureLimit, 37778))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterTemperatureHysteresis, 1280))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterShuntVoltage, 56943))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterShuntPWMDutyCycle, 0))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterBusCurrentSupplyLimit, 39604))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterBusCurrentReturnLimit, 39604))
  PMD_RESULT(PMDUpdate (phAxis))
  PMD_RESULT(PMDClearDriveFaultStatus (phAxis))
  PMD_RESULT(PMDResetEventStatus (phAxis, 0x0000))
  PMD_RESULT(PMDSetOperatingMode (phAxis, 0x0003))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 3277))
  PMD_RESULT(InitializePhase (phAxis))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 0))
  PMD_RESULT(PMDCalibrateAnalog (phAxis, 0))
  sleep(1);
  PMD_RESULT(PMDSetOperatingMode (phAxis, 0x0037))
  PMD_RESULT(PMDSetGearRatio (phAxis, 0x00000000))

  return result;
}


void main()
{
	PMDuint16 generation, motorType, numberAxes, chip_count, custom, major, minor;
	PMDAxisHandle hAxis1; 
	PMDresult result;

	// User defined variables
	// PMDuint16 ControlMode = FOC;
	
	char localport[20]="/dev/ttyUSB1";

	result=PMDSetupAxisInterface_Serial( &hAxis1, PMDAxis1, localport );
	if(result)
	{
		printf("Error:Could not open serial port code=%x\n",result);
		return;
	}	
	
	PMDSerial_Sync(&hAxis1);
   
   	result=PMDGetVersion( &hAxis1, &generation, &motorType, &numberAxes, &chip_count, &custom, &major, &minor );
    if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{	
	printf("PMD Motion Processor MC%d%d%d%d0 v%d.%d\n\n", generation, motorType, numberAxes, chip_count, major, minor);

	}
	
	// To Do:
	// Add any initialization commands like SetupAxis1() generated by Pro-Motion.
	// Then start a move by call ProfileMove() 

	result = SetupAxis1(&hAxis1);

	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Axis 1 Setup Complete\n");
	}

	// result = PMDGetMotorType(&hAxis1, &motorType);
	// if(result)
	// {
	// 	printf("result=%x\n",result);
	// 	return;
	// }
	// else
	// {
	// 	printf("Motor Type is BLDC 3 Phase\n");
	// }

	// Start User Defined Code Here


	//Set the Profile Mode to Trapezoidal

	// for(;;)
	
	result = PMDSetProfileMode( &hAxis1, PMDProfileModeTrapezoidal);

	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Profile Mode Set to Trapezoidal\n");
	}
	// Load a destination Position for the axis
	result = PMDSetPosition(&hAxis1, 12345);
	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Position Set to 12345\n");
	}
	// Load Velocity for axis 
	result = PMDSetVelocity(&hAxis1, 20000);
	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Velocity Set to 20000\n");
	}
	// Load Acceleration for axis
	result = PMDSetAcceleration(&hAxis1, 1000);
	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Acceleration Set to 1000\n");
	}
	// specify that an update of profile params only is to occur
	result = PMDSetUpdateMask(&hAxis1, PMDUpdateMaskTrajectory);
	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Update Mask Set to Trajectory\n");
	}

	// Start the move
	result = PMDUpdate(&hAxis1);
	if(result)
	{
		printf("result=%x\n",result);
		return;
	}
	else
	{
		printf("Move Started\n");
	}

	PMDuint16 status = 0;
	while(!status)
	{
		result = PMDGetEventStatus(&hAxis1, &status);
		if(status == 0x00010)
		{
			printf("Motion Error\n");
			return;
		}
		status &=1;
		if(result)
		{
			printf("result=%x\n",result);
			return;
		}
		else
		{
			printf("Status=%x\n",status);
		}
	}
	
	

}

