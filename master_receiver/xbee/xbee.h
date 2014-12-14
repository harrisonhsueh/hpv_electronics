/* Copyright (c) 2012 Tristan Hughes, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "mbed.h"

/** Xbee interface class for configuring, sending and recieving data using an Xbee */
class xbee
{
private:
    PinName _tx;
    PinName _rx;
    PinName _reset;
public:
    /** Configure serial data pin.
      * @param tx The serial tx pin the xbee is conected to.
      * @param rx The serial rx pin the xbee is conected to.
      * @param reset The pin connected to the Xbee reset pin.
      */
    xbee(PinName tx, PinName rx, PinName reset);
    ~xbee();
    /** Puts the Xbee into config mode.
      * @return Returns 1 on success.
      */
    int ConfigMode();
    /** Gets the serial number/mac address of the Xbee and places it into serial_no.
      * @param serial_no array to store the serial of Xbee (must be 8 long).
      * @return Returns 1 on success.
      */
    int GetSerial(int*);
    /** Sets the encryption key. This should be a 128-bit key.
      * @param key Pointer to the network key to set.
      * @return Returns 1 on success.
      */
    int SetKey(char*);
    /** Sets the id of the PAN network for the Xbee to use
      * @param pan_id The id of the PAN for the Xbee to use.
      * @return Returns 1 on success.
      */
    int SetPanId(int);
    /** Writes the settings to the Non volatile memory on the Xbee
      * @param key Pointer to the network key to set.
      * @return Returns 1 on success.
      */
    int WriteSettings();
    /** Exits config mode
      * @return Returns 1 on success.
      */
    int ExitConfigMode();
    /** Sends data in the send_Data buffer.
      * @param data_buf Pointer to the buffer of data to send.
      * @returns 1 on success.
      */
    int SendData(char*);
    /** Recieves data sent to the xbee.
      * @param data_buf Pointer to the buffer to put recieved data into.
      * @param numchar Number of characters to read. If 0, will use the size of data_buf.
      */
    void RecieveData(char*, int);
        /** Resets the Xbee.
      */
    void Reset();

};