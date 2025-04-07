/*
The MIT License

Copyright (c) 2025 zukki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "BUS.h"
#include "Arduino.h"

BUS::CAN_Msg BUS::CAN_Rx_Msg;
BUS::CAN_Msg BUS::CAN_Tx_Msg;

static BUS* activeBusInstance = nullptr;

BUS::BUS() {
    activeBusInstance = this;
}

int BUS::begin(int BitRate) {
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA11 (CAN_RX) and PA12 (CAN_TX)
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODER11_Pos) | (0x02 << GPIO_MODER_MODER12_Pos);

    // Set alternate function 9 for PA11 (CAN_RX) and PA12 (CAN_TX)
    GPIOA->AFR[1] |= (0x09 << (4 * (11 - 8))) | (0x09 << (4 * (12 - 8)));

    // Configure CAN
    CAN->MCR |= CAN_MCR_INRQ;
    while (!(CAN->MSR & CAN_MSR_INAK));

    CAN->MCR &= ~CAN_MCR_SLEEP;
    CAN->MCR |= CAN_MCR_ABOM;

    // Set bit rate
    switch (BitRate) {
        case 1000000:
            CAN->BTR = (0x00 << CAN_BTR_SJW_Pos) |  // SJW = 1 TQ
                       (0x0C << CAN_BTR_TS1_Pos) |  // TS1 = 13 TQ
                       (0x01 << CAN_BTR_TS2_Pos) |  // TS2 = 2 TQ
                       (0x03 << CAN_BTR_BRP_Pos);   // BRP = 3
            break;
        case 500000:
            CAN->BTR = (0x00 << CAN_BTR_SJW_Pos) |  // SJW = 1 TQ
                       (0x0C << CAN_BTR_TS1_Pos) |  // TS1 = 13 TQ
                       (0x01 << CAN_BTR_TS2_Pos) |  // TS2 = 2 TQ
                       (0x06 << CAN_BTR_BRP_Pos);   // BRP = 6
            break;
        case 250000:
            CAN->BTR = (0x00 << CAN_BTR_SJW_Pos) |  // SJW = 1 TQ
                       (0x0C << CAN_BTR_TS1_Pos) |  // TS1 = 13 TQ
                       (0x01 << CAN_BTR_TS2_Pos) |  // TS2 = 2 TQ
                       (0x0C << CAN_BTR_BRP_Pos);   // BRP = 12
            break;
        default:
            // Default to 125 kbps if BitRate is not recognized
            CAN->BTR = (0x00 << CAN_BTR_SJW_Pos) |  // SJW = 1 TQ
                       (0x0C << CAN_BTR_TS1_Pos) |  // TS1 = 13 TQ
                       (0x01 << CAN_BTR_TS2_Pos) |  // TS2 = 2 TQ
                       (0x0F << CAN_BTR_BRP_Pos);   // BRP = 16
            break;
    }

    // Configure CAN filter
    CAN->FMR |= CAN_FMR_FINIT;

    CAN->FA1R &= ~CAN_FA1R_FACT0;
    CAN->FS1R |= CAN_FS1R_FSC0;
    CAN->FM1R &= ~CAN_FM1R_FBM0;
    //CAN->FM1R |= CAN_FM1R_FBM0;  // Set filter mode to list mode

    // Set filter ID (standard 11-bit ID shifted to fit into the filter)
    CAN->sFilterRegister[0].FR1 = 0x0000;
    CAN->sFilterRegister[0].FR2 = 0x0000;

    // Set filter mask (match the full ID or specific bits you want to ignore)
    //CAN->sFilterRegister[0].FR1 |= 0x0000;  // Set filter mask high (11-bit mask)
    //CAN->sFilterRegister[0].FR2 = 0x0000;  // Set filter mask low (if using 32-bit)

    CAN->FFA1R &= ~CAN_FFA1R_FFA0;
    CAN->FA1R |= CAN_FA1R_FACT0;

    CAN->FMR &= ~CAN_FMR_FINIT;

    CAN->MCR &= ~CAN_MCR_INRQ;
    while (CAN->MSR & CAN_MSR_INAK);

    // Enable CAN interrupts (optional)
    CAN->IER |= CAN_IER_TMEIE;
    NVIC_EnableIRQ(CAN_TX_IRQn);

    CAN->IER |= CAN_IER_FMPIE0;
    CAN->IER |= CAN_IER_FFIE0;
    CAN->IER |= CAN_IER_FOVIE0;

    NVIC_SetPriority(CAN_RX0_IRQn, 6);
    NVIC_EnableIRQ(CAN_RX0_IRQn);

    return 0;
}

int BUS::setID(uint32_t id1, uint32_t id2) {
    // Configure CAN filter
    CAN->FMR |= CAN_FMR_FINIT;

    CAN->FA1R &= ~CAN_FA1R_FACT0;
    CAN->FS1R |= CAN_FS1R_FSC0;
    CAN->FM1R |= CAN_FM1R_FBM0;

    // Set filter ID (standard 11-bit ID shifted to fit into the filter)
    CAN->sFilterRegister[0].FR1 = (id1 << 21);
    CAN->sFilterRegister[0].FR2 = (id2 << 21);

    CAN->FFA1R &= ~CAN_FFA1R_FFA0;
    CAN->FA1R |= CAN_FA1R_FACT0;

    CAN->FMR &= ~CAN_FMR_FINIT;

    CAN->MCR &= ~CAN_MCR_INRQ;
    while (CAN->MSR & CAN_MSR_INAK);

    return 0;
}

int BUS::setMask(uint32_t id, uint32_t mask) {
    // Configure CAN filter
    CAN->FMR |= CAN_FMR_FINIT;

    CAN->FA1R &= ~CAN_FA1R_FACT0;
    CAN->FS1R |= CAN_FS1R_FSC0;
    CAN->FM1R &= ~CAN_FM1R_FBM0;

    // Set filter ID (standard 11-bit ID shifted to fit into the filter)
    CAN->sFilterRegister[0].FR1 = (id << 21);
    CAN->sFilterRegister[0].FR2 = (mask << 21);

    CAN->FFA1R &= ~CAN_FFA1R_FFA0;
    CAN->FA1R |= CAN_FA1R_FACT0;

    CAN->FMR &= ~CAN_FMR_FINIT;

    CAN->MCR &= ~CAN_MCR_INRQ;
    while (CAN->MSR & CAN_MSR_INAK);

    return 0;
}

void BUS::write(uint32_t id, uint8_t* data, uint8_t len) {
    // Wait for an empty transmit mailbox
    while (!(CAN->TSR & CAN_TSR_TME0));

    CAN_Tx_Msg.id = id;
    for (int i = 0; i < len; i++) {
        CAN_Tx_Msg.data[i] = data[i];
    }
    CAN_Tx_Msg.len = len;
    // Set up the transmit header
    CAN->sTxMailBox[0].TIR = CAN_Tx_Msg.id << CAN_TI0R_STID_Pos;
    CAN->sTxMailBox[0].TDTR = CAN_Tx_Msg.len;

    // Copy data to transmit mailbox
    CAN->sTxMailBox[0].TDLR = (CAN_Tx_Msg.data[0] << 0) | (CAN_Tx_Msg.data[1] << 8) | (CAN_Tx_Msg.data[2] << 16) | (CAN_Tx_Msg.data[3] << 24);
    CAN->sTxMailBox[0].TDHR = (CAN_Tx_Msg.data[4] << 0) | (CAN_Tx_Msg.data[5] << 8) | (CAN_Tx_Msg.data[6] << 16) | (CAN_Tx_Msg.data[7] << 24);

    // Request transmission
    CAN->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

int BUS::available(void) {
    return (CAN->RF0R & CAN_RF0R_FMP0);
}

void BUS::read() {
    if (CAN->RF0R & CAN_RF0R_FMP0) {
        CAN_Rx_Msg.id = (CAN->sFIFOMailBox[0].RIR >> 21) & 0x7FF; // Extract standard ID
        CAN_Rx_Msg.len = (CAN->sFIFOMailBox[0].RDTR) & 0xF; // Extract data length

        CAN_Rx_Msg.data[0] = (CAN->sFIFOMailBox[0].RDLR) & 0xFF; // Byte 0 (MSB of RDLR)
        CAN_Rx_Msg.data[1] = (CAN->sFIFOMailBox[0].RDLR >> 8) & 0xFF; // Byte 1
        CAN_Rx_Msg.data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;  // Byte 2
        CAN_Rx_Msg.data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;  // Byte 3 (LSB of RDLR)
        CAN_Rx_Msg.data[4] = (CAN->sFIFOMailBox[0].RDHR) & 0xFF; // Byte 4 (MSB of RDHR)
        CAN_Rx_Msg.data[5] = (CAN->sFIFOMailBox[0].RDHR >> 8) & 0xFF; // Byte 5
        CAN_Rx_Msg.data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;  // Byte 6
        CAN_Rx_Msg.data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;  // Byte 7 (LSB of RDHR)
        // Release FIFO 0
        CAN->RF0R |= 0x20;
    }
}

extern "C" void CAN_RX0_IRQHandler() {
    if (CAN->RF0R & CAN_RF0R_FMP0) {
        if (activeBusInstance) {
            activeBusInstance->CAN_Rx_Msg.id = (CAN->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos) & 0x7FF;
            activeBusInstance->CAN_Rx_Msg.len = (CAN->sFIFOMailBox[0].RDTR) & 0xF;

            uint32_t RDLR = CAN->sFIFOMailBox[0].RDLR;
            uint32_t RDHR = CAN->sFIFOMailBox[0].RDHR;

            activeBusInstance->CAN_Rx_Msg.data[0] = (RDLR >> 0) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[1] = (RDLR >> 8) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[2] = (RDLR >> 16) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[3] = (RDLR >> 24) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[4] = (RDHR >> 0) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[5] = (RDHR >> 8) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[6] = (RDHR >> 16) & 0xFF;
            activeBusInstance->CAN_Rx_Msg.data[7] = (RDHR >> 24) & 0xFF;

            CAN->RF0R |= CAN_RF0R_RFOM0;
            activeBusInstance->handleReceive();
        }
    }
}

extern "C" void CAN_TX_IRQHandler() {
    if (activeBusInstance) {
        if (CAN->TSR & CAN_TSR_RQCP0) {
            CAN->TSR |= CAN_TSR_RQCP0;
            activeBusInstance->handleTransmit();
        }
        if (CAN->TSR & CAN_TSR_RQCP1) {
            CAN->TSR |= CAN_TSR_RQCP1;
        }

        if (CAN->TSR & CAN_TSR_RQCP2) {
            CAN->TSR |= CAN_TSR_RQCP2;
        }
    }
}