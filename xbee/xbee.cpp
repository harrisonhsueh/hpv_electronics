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
 
 #include "xbee.h"


xbee::xbee(PinName tx, PinName rx, PinName reset)
{
    _tx = tx;
    _rx = rx;
    _reset = reset;
}

xbee::~xbee()
{
}

int xbee::ConfigMode()
{
    int a;
    Serial DATA(_tx,_rx);
    wait(2);
    DATA.printf("+++");
    while (a != 75) {
        if (DATA.readable()) {
            a = DATA.getc();
        }
    }
    wait(1);
    printf("config mode\n");
    return 1;
}

int xbee::GetSerial(int *serial_no)
{
    int sh1,sh2,sh3,sl1,sl2,sl3,sl4;
    Serial DATA(_tx,_rx);
    wait_ms(50);
    DATA.printf("ATSL \r");
    DATA.scanf ("%2x%2x%2x%2x",&sl1,&sl2,&sl3,&sl4);
    wait_ms(500);
    DATA.printf("ATSH \r");
    DATA.scanf ("%2x%2x%2x",&sh1,&sh2,&sh3);

    serial_no[0] = sh1;
    serial_no[1] = sh2;
    serial_no[2] = sh3;
    serial_no[3] = sl1;
    serial_no[4] = sl2;
    serial_no[5] = sl3;
    serial_no[6] = sl4;

    return 1;
}

int xbee::SetKey(char* key)
{
    Serial DATA(_tx,_rx);
    DATA.printf("ATEE 1 \r");

    DATA.scanf ("%*s");
    wait_ms(1);
    DATA.printf("ATKY %s \r",key);
    DATA.scanf ("%*s");
    return 1;
}

int xbee::WriteSettings()
{
    Serial DATA(_tx,_rx);
    wait_ms(5);
    DATA.printf("ATWR \r");
    DATA.scanf ("%*s");
    return 1;
}

int xbee::ExitConfigMode()
{
    Serial DATA(_tx,_rx);
    wait_ms(5);
    DATA.printf("ATCN \r");
    DATA.scanf ("%*s");
    return 1;
}

int xbee::SendData(char *data_buf)
{
    Serial DATA(_tx,_rx);
    DATA.printf("%s",data_buf);
    return 1;
}

void xbee::RecieveData(char *data_buf, int numchar)
{
    int count=0;
    if(numchar == 0) {
        numchar = sizeof(data_buf);
    }
    Serial DATA(_tx,_rx);
    while(numchar!=count) {
        if(DATA.readable()) {
            *data_buf = DATA.getc();
            data_buf+=1;
            count++;
        }

    }
}

int xbee::SetPanId(int pan_id)
{
    Serial DATA(_tx,_rx);
    wait_ms(5);
    DATA.printf("ATID %i\r",pan_id);
    DATA.scanf ("%*s");
    return 1;
}

void xbee::Reset()
{
    DigitalOut rst(_reset);
    rst = 0;
    wait_ms(10);
    rst = 1;
    wait_ms(1);
}


