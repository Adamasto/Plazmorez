#include "packet_builder.h"
#include "stdint.h"
#include "flash_control.h"
#include "status_control.h"
#include "comm_pars.h"
#include "posicioner_config.h"
#include <stdio.h>
extern AxisParam Axx[AXIS_NUM];
extern struct switcherBitMask switcherState;


udpPacketH udpPacket;

udpPacketH udpPacketBuilder()
{
//	udpPacket->identificator = 0xACAB;
//	udpPacket->version = 1;
//	udpPacket->network_address = flash_getNetworkAddress();
//	udpPacket->network_port = flash_getPort();
//	udpPacket->xME = status_GetLastStopCause(0);
//	udpPacket->yME = status_GetLastStopCause(1);
//	udpPacket->xPositionMm = status_GetCurrentPosition(0);
//	udpPacket->yPositionMm = status_GetCurrentPosition(1);
//	udpPacket->statusBitMask.xCalibrationFlag = Axx[0].status.waitBitMask.calibrationFlag;
//	udpPacket->statusBitMask.yCalibrationFlag = Axx[1].status.waitBitMask.calibrationFlag;
//	udpPacket->statusBitMask.xEngineWorkingCapacity = 1;
//	udpPacket->statusBitMask.ylEngineWorkingCapacity = 1;
//	udpPacket->statusBitMask.yrEngineWorkingCapacity = 1;
//	udpPacket->statusBitMask.xEncoderWorkingCapacity = 1;
//	udpPacket->statusBitMask.ylEncoderWorkingCapacity = 1;
//	udpPacket->statusBitMask.yrEncoderWorkingCapacity = 1;
//	udpPacket->xPE = status_GetCurrentErrorPosition(0);
//	udpPacket->yPE = status_GetCurrentErrorPosition(1);
//	udpPacket->statusMS.xEngineMS = status_GetCurrentMotionStateMask(0);
//	udpPacket->statusMS.yEngineMS = status_GetCurrentMotionStateMask(1);
//	udpPacket->xSpeed = status_GetCurrentSpeed(0);
//	udpPacket->ySpeed = status_GetCurrentSpeed(1);
	udpPacket.identificator = 0xA0140A33;
	udpPacket.version = 1;
	udpPacket.network_address = flash_getNetworkAddress();
	udpPacket.network_port = flash_getPort();
	udpPacket.xME = status_GetLastStopCause(0);
	udpPacket.yME = status_GetLastStopCause(1);
	udpPacket.xPositionMm = status_GetCurrentPosition(0);
	udpPacket.yPositionMm = status_GetCurrentPosition(1);
	udpPacket.statusBitMask.xCalibrationFlag = Axx[0].status.waitBitMask.calibrationFlag;
	udpPacket.statusBitMask.yCalibrationFlag = Axx[1].status.waitBitMask.calibrationFlag;
	udpPacket.statusBitMask.xEngineWorkingCapacity = 1;
	udpPacket.statusBitMask.ylEngineWorkingCapacity = 1;
	udpPacket.statusBitMask.yrEngineWorkingCapacity = 1;
	udpPacket.statusBitMask.xEncoderWorkingCapacity = 1;
	udpPacket.statusBitMask.ylEncoderWorkingCapacity = 1;
	udpPacket.statusBitMask.yrEncoderWorkingCapacity = 1;
	udpPacket.xPE = status_GetCurrentErrorPosition(0);
	udpPacket.yPE = status_GetCurrentErrorPosition(1);
	udpPacket.statusMS.xEngineMS = status_GetCurrentMotionStateMask(0);
	udpPacket.statusMS.yEngineMS = status_GetCurrentMotionStateMask(1);
	udpPacket.xSpeed = status_GetCurrentSpeed(0);
	udpPacket.ySpeed = status_GetCurrentSpeed(1);
	udpPacket.switcherBitMask = getSwitcherState();
	
	return udpPacket;
}
