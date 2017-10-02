#ifndef PACKET_BUILDER_H
#define PACKET_BUILDER_H

#include "stdint.h"
#include "lwip/opt.h"
#include "lwip/netbuf.h"
#include "lwip/memp.h"

#include <string.h>
typedef struct
{
	uint8_t switcherBitMask;
	unsigned xCalibrationFlag:1;
	unsigned yCalibrationFlag:1;
	unsigned xEngineWorkingCapacity:1;
	unsigned ylEngineWorkingCapacity:1;
	unsigned yrEngineWorkingCapacity:1;
	unsigned xEncoderWorkingCapacity:1;
	unsigned ylEncoderWorkingCapacity:1;
	unsigned yrEncoderWorkingCapacity:1;
	uint16_t reserved;
}statusBitMaskH;

typedef struct
{
	uint8_t xEngineMS;
	uint8_t yEngineMS;
	uint16_t resrved;
}statusMSH;

typedef struct
{
	uint32_t identificator;
	uint32_t version;
	uint32_t network_address;
	uint32_t network_port;
	uint32_t xME;
	uint32_t yME;
	uint32_t xPositionMm;
	uint32_t yPositionMm;
	statusBitMaskH statusBitMask;
	uint32_t xPE;
	uint32_t yPE;
	statusMSH statusMS;
	uint32_t xSpeed;
	uint32_t ySpeed;
	uint32_t switcherBitMask;
}udpPacketH;

udpPacketH udpPacketBuilder();
err_t netbuf_refUdp(struct netbuf *buf, udpPacketH *dataptr, u16_t size);









#endif /* PACKET_BUILDER_H */
