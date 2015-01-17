#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define RF24_TRANSFER_SIZE 32
#define MY_ADDR 1
RF24 rf24(9, 10);

/* Run setup code. */
void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  rf24.begin();
  connect_master();
	
}

char *name = "speed";
uint64_t master_general_address; //master will read on this address
uint64_t master_connection_address = 0x0000000001; //this is the address we write to connect to the master
char read_buffer[RF24_TRANSFER_SIZE];
char write_buffer[RF24_TRANSFER_SIZE];

void connect_master() {
  //send, listen, send, listen.... until we get a response from master
  //master uses pipe1 to establish connections, use pipe2 to flood out to all sensors, pipe3 to connect to privileged sensors, pipe4 to receive in
  Serial.println("connect master");
	char *start_msg = (char *) malloc(32);
  Serial.println("malloc");
	sprintf(start_msg, "%c%c%s", 0, MY_ADDR, "connect");
  Serial.println("sprintf");
  rf24.openWritingPipe(master_connection_address);
  Serial.println("openwritingpipe");
  rf24.write(start_msg, strlen(start_msg));
  rf24.openReadingPipe(1, 0x0000000101);
  rf24.startListening();
  Serial.println("StartListening");
  bool not_connected = true;
  while (not_connected) {
		Serial.println("connecting with master");
    if (rf24.available()) {
			Serial.print("waiting for master...");
      rf24.read(read_buffer, RF24_TRANSFER_SIZE);
      not_connected = false;
    } else {
      delay(500);
			Serial.print("waiting for master again...");
      if (rf24.available()) {
        rf24.read(read_buffer, RF24_TRANSFER_SIZE);
        not_connected = false;
      } else {
				Serial.println("problems encountered, retrying.");
				rf24.stopListening();
				rf24.write(start_msg, strlen(start_msg));
				rf24.startListening();
			}
    }
  }
}

/* Writes data to master
 * Use this for sensor-type slaves that gather data locally and send to the master
 * Comment out the write_data line in loop function if not needed.
 */
void write_data() {
	/* Write code here. */
	rf24.stopListening();
	rf24.write(write_buffer, RF24_TRANSFER_SIZE);
	rf24.startListening();
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
  // put your main code here, to run repeatedly:
	//send, sleep, send, sleep...
	Serial.println("looping");
	read_data();
	write_data();
}
