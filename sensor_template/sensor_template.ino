#include <SPI.h>
#include "nRF24L01.h"
#include <stdio.h>
#include "RF24.h"
#include "println.h"
#include "../constants.h"

#define MY_ADDR 1
RF24 rf24(9, 10);

/* For serial debugging. */
int serial_console_putc(char c, FILE *) {
	Serial.write(c);
	return 0;
}

uint8_t state; //state that the sensor is in. 0 = connected, 1 = connected, 2 = sleep
long my_pipe;
char *name = "template";
uint64_t master_general_address = (MY_ADDR % 4) + 2; //master will read on this address
uint64_t master_connection_address = 0x0000000001; //this is the address we write to connect to the master
char read_buffer[RF24_TRANSFER_SIZE];
char write_buffer[RF24_TRANSFER_SIZE];
char *write_data;
/* Run setup code. */
void setup() {
	Serial.begin(9600);
	state = DISCONNECTED;
	write_buffer[0] = MY_ADDR;
	write_data = write_buffer + 1;
	my_pipe = (MY_ADDR % 4) + 2;
	rf24.begin();
	rf24.setDataRate(RF24_1MBPS);
	rf24.setCRCLength(RF24_CRC_8);
	rf24.setPayloadSize(RF24_TRANSFER_SIZE);
	rf24.setChannel(101);
	rf24.setAutoAck(true);

	/* For debugging, comment out when not needed. */
	fdevopen(&serial_console_putc, NULL);
	rf24.printDetails();

	/*if (!connect_master()) {
		shutdown();
		}*/
	rf24.openWritingPipe(my_pipe);
}


bool connect_master() {
	Serial.print("connecting master...");
	Serial.flush();
	char *start_msg = (char *) malloc(RF24_TRANSFER_SIZE);
	sprintf(start_msg, "%c%s", MY_ADDR, "connect");
	Serial.print("saved start message");
	Serial.flush();
	//rf24.openWritingPipe(master_connection_address);
	rf24.openWritingPipe(0x00F0F0F0F0);
	rf24.openReadingPipe(1, 0xF0F0F0F0D2);
	bool connected = false;
	int timeout = 100; //timeout in ms;
	while (!connected) {
		rf24.stopListening();
		bool connected = rf24.write(start_msg, sizeof(char) * RF24_TRANSFER_SIZE);
		if (!connected) {
			if (timeout > 1000) {
				free(start_msg);
				Serial.println("failed.");
				return false;
			}
			delay(timeout);

			timeout += 100;
			Serial.print("failed...");
		}
		rf24.startListening();

	}
	free(start_msg);
	Serial.println("done.");
	return true;
}

/* Writes data to master
 * Use this for sensor-type slaves that gather data locally and send to the master
 * Comment out the write_data line in loop function if not needed.
 * Write data to write_data, which can store a max of (RF24_TRANSFER_SIZE - 1) bytes/chars.
 */
void write_data() {
	/* Write code here. */
	rf24.stopListening();
	bool received = false;
	while (!received) {
		received = rf24.write(write_buffer, RF24_TRANSFER_SIZE);
		rf24.startListening();
	}
}
/* Shut down this sensor. */
void shutdown() {
	//power down antenna, set all unused pins low, put microcontroller to sleep for 1/2(?) second then wake up
}

/* Handles an incoming packet meant for this slave.
 * @data The data packet meant for this slave.
 */
void read_handler(char *data) {
	if (strstr(data, "shutdown") == data) {
		shutdown();
	} else {
		/* Write code here. */
	}
}

/* Reads data from master
 * All slaves should use this, to receive command messages such as shutdown.
 */
void read_data() {
	if (rf24.available()) {
		rf24.read(read_buffer, RF24_TRANSFER_SIZE);
		if (read_buffer[0] == MY_ADDR) {
			read_handler(read_buffer + 2);
		}
	}
}
void loop() {
	// put your main code here, to run repeatedly
	Serial.println("looping...");
	if (state == DISCONNECTED) {
		if (!connect_master()) {
			state = SLEEP;
			Serial.println("failed to connect");
		}
	} else if (state == CONNECTED) {
		read_data();
		write_data();
		delay(100);
	} else if (state == SLEEP) {
		delay(1000);
		if (connect_master()) {
			state = CONNECTED;
		} else {
			Serial.println("failed to connect");
			//should it wake up?
		} else if (state == DEEP_SLEEP) {
			delay(10000);
			if (connect_master()) {
				state = CONNECTED;
			} else {
				Serial.println("failed to connect");
			}
		}
	}
}
