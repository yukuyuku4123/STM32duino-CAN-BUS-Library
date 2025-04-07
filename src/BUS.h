/*
The MIT License

Copyright (c) 2025 zukki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BUS_h
#define BUS_h

#include "Arduino.h"

#define _1M 1000000
#define _500K 500000
#define _250K 250000

class BUS {
public:
    struct CAN_Msg {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
    };
    static CAN_Msg CAN_Rx_Msg;
    static CAN_Msg CAN_Tx_Msg;

    typedef void (*CallbackFunc)(void);

    BUS();
    int begin(int BitRate);
    int setID(uint32_t id, uint32_t id2);
    int setMask(uint32_t id, uint32_t mask);
    void write(uint32_t id, uint8_t* data, uint8_t len);
    int available(void);
    void read();

    void onReceive(CallbackFunc func) { onReceiveCallback = func; }
    void onTransmit(CallbackFunc func) { onTransmitCallback = func; }

    void handleReceive() { if(onReceiveCallback) onReceiveCallback(); }
    void handleTransmit() { if(onTransmitCallback) onTransmitCallback(); }

private:
    CallbackFunc onReceiveCallback = nullptr;
    CallbackFunc onTransmitCallback = nullptr;
};

extern "C" {
    void CAN_RX0_IRQHandler(void);
    void CAN_TX_IRQHandler(void);
}

#endif