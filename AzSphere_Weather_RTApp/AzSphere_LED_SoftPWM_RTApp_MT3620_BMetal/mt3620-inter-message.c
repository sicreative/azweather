#include <stdbool.h>
#include <ctype.h>

#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-inter-message.h"


void init_inter_message();
int8_t dequeue(uint8_t* type, uint8_t* data, size_t* data_size);
void enqueue(uint32_t type, uint8_t* send, size_t send_size);
void interLog(const char *msg);
void sendADC(uint8_t* send);


static BufferHeader* outbound, * inbound;
static uint32_t sharedBufSize = INTER_BUFF_SIZE;
static const size_t payloadStart = 20;
static uint8_t buf[INTER_BUFF_SIZE+32];







void init_inter_message() {
	
	if (GetIntercoreBuffers(&outbound, &inbound, &sharedBufSize) == -1) {
		for (;;) {
			// empty
		}
	}
}

int8_t dequeue(uint8_t* type, uint8_t* data, size_t* data_size) {

	
	
	//uint32_t dataSize = sizeof(buf);


	// On success, dataSize is set to the actual number of bytes which were read.
	int r = DequeueData(outbound, inbound, sharedBufSize, buf, data_size);

	if (r == -1 || *data_size < payloadStart)
		return -1;

	*data_size -= (payloadStart + 1);
	*type = buf[payloadStart];

	for (int i = 0;i < *data_size;i++) {
		data[i] = buf[payloadStart + 1 + i];
	}
	
	



	return 0;

}



void enqueue(uint32_t type ,uint8_t *send,size_t send_size) {

	union encap32
	{
		uint32_t u32;
		uint8_t u8[4];
	} encap32;

	size_t size = send_size + 4;
	if (size + payloadStart > 256)
		return;

	encap32.u32 = type;

	for (size_t i = 0; i < 4;i++) {
		buf[payloadStart + i ] = encap32.u8[i];
	}
	
	for (size_t i = 4; i < size;i++) {
		buf[payloadStart + i ] = send[i-4];
	}

	EnqueueData(inbound, outbound, sharedBufSize, buf, size+payloadStart);
}


void interLog(const char *msg) {

	

	char prefix[] = "RTCore: ";
	char end[] = "\n";
	char total[strlen(msg) + strlen(prefix) + strlen(end)+1];
	strcpy(total, prefix);
	strcat(total, msg);
	strcat(total, end);
	//int length = strlen(msg)+1;

	enqueue(LOG, total, strlen(total)+1);

}

