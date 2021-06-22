#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "SBGC_Linux.h"

/* Defines serial port routines required for SBGC_parser, here */
class LinuxComObj : public SBGC_ComObj {
	int fd;
	public:
	inline void init(int fd) {
		this->fd = fd;

        printf("LinuxComObj's fd is set as FD(%d)\n", this->fd);
	}

	virtual uint16_t getBytesAvailable() {
        printf("getBytesAvailable(...)\n");
        abort();
		return true;
	}
	
	virtual uint8_t readByte() {
        printf("readByte(...)\n");
        abort();
		return 0;
	}
	
	virtual void writeByte(uint8_t b) {
        fprintf(stderr, "writeByte(%d) on FD(%d)\n", b, fd);
		if (write( fd, &b, 1) != 1) {
            fprintf(stderr, "Error %i from write: %s\n", errno, strerror(errno));
        }
	}
	
	// Arduino com port is not buffered, so empty space is unknown.
	virtual uint16_t getOutEmptySpace() {
		return 0xFFFF;
	}

};


/* Global variables */
SBGC_Parser sbgc_parser;  // SBGC command parser. Define one for each port.
LinuxComObj com_obj; // COM-port wrapper required for parser



// Prepare hardware, used in examples
void SBGC_Demo_setup(int fd) {
  com_obj.init(fd);
  sbgc_parser.init(&com_obj);
  
  // Use LED to show steps of program run
  //pinMode(13, OUTPUT);
  LED_ON();
}



void blink_led(uint8_t cnt) {
	for(uint8_t i=0; i<cnt; i++) {
		LED_OFF();
		usleep(150);
		LED_ON();
		usleep(200);
	}
}
