///*
//The contents of this code and instructions are the intellectual property of Carbon Aeronautics.
//The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence.
//This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics
//(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
//This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity
//with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by
//the software and hardware described in it. As Carbon Aeronautics has no control over the use, init, assembly, modification or misuse of the hardware,
//software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury.
//By the act of copying, use, init or assembly, the user accepts all resulting liability.
//
//1.0  5 October 2022 -  initial release
//*/
//
//#include <PulsePosition.h>
//PulsePositionInput ReceiverInput(RISING);
//float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
//int ChannelNumber=0;
//void read_receiver(void){
//  ChannelNumber = ReceiverInput.available();
//  if (ChannelNumber > 0) {
//      for (int i=1; i<=ChannelNumber;i++){
//    ReceiverValue[i-1]=ReceiverInput.read(i);
//    }
//  }
//}
//void init() {
//  Serial.begin(57600);
//  pinMode(13, OUTPUT);
//  digitalWrite(13, HIGH);
//  ReceiverInput.begin(14);
//}
//void loop() {
//  read_receiver();
//  Serial.print("Number of channels: ");
//  Serial.print(ChannelNumber);
//  Serial.print(" Roll [µs]: ");
//  Serial.print(ReceiverValue[0]);
//  Serial.print(" Pitch [µs]: ");
//  Serial.print(ReceiverValue[1]);
//  Serial.print(" Throttle [µs]: ");
//  Serial.print(ReceiverValue[2]);
//  Serial.print(" Yaw [µs]: ");
//  Serial.println(ReceiverValue[3]);
//  delay(50);
//}

#include "Receiver.h"
#include <cassert>

void Receiver::init(int pin1, int pin2, int pin3, int pin4) {
    if (!initialized) {
        pinMode(pin1, INPUT);
        this->pin1 = pin1;
        pinMode(pin2, INPUT);
        this->pin2 = pin2;
        pinMode(pin3, INPUT);
        this->pin3 = pin3;
        pinMode(pin4, INPUT);
        this->pin4 = pin4;
        initialized = true;
    }
}

float Receiver::fetch(int channel) {
    assert(initialized);
    switch (channel) {
        case 1:
            return pulseIn(pin1, HIGH, 25000);
        case 2:
            return pulseIn(pin2, HIGH, 25000);
        case 3:
            return pulseIn(pin3, HIGH, 25000);
        case 4:
            return pulseIn(pin4, HIGH, 25000);
        default:
            assert(false && "Invalid channel number");
    }
}