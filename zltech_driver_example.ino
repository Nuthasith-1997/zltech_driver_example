#include <ModbusMaster.h>
#include <SoftwareSerial.h>
// https://github.com/4-20ma/ModbusMaster
// https://github.com/PaulStoffregen/SoftwareSerial

#include "zltech_item.h"

int32_t position[2] = {0, 0};
int16_t velocity[2] = {0, 0};
uint16_t acc_time = 50;
uint16_t dec_time = 50;

ModbusMaster node;
SoftwareSerial mySerial(6,7);       // pin 6 = RX, pin 7 = TX

float Data_Modbus[5];

void setup(){
    Serial.begin(57600);
    mySerial.begin(9600);
    node.begin(1, mySerial);

    Serial.println("Initializing..");

    if (!setControlMode(3)){
        Serial.println("Cannot set Velocity Mode.");
    }
    if (!setControlWord(8)){
        Serial.println("Cannot enable motors.");
    }
    if (!setAccTime(acc_time)){
        Serial.println("Cannot set AccTime.");
    }
    if (!setDecTime(dec_time)){
        Serial.println("Cannot set DecTime.");
    }
}

void loop(){
    if (!getVelocity(velocity)){
        Serial.println("Cannot get velocity.");
    }
    if (!getPosition(position)){
        Serial.println("Cannot get position.");
    }
    Serial.print("Left = ");
    Serial.print(velocity[0]);
    Serial.print(" ");
    Serial.print("Right = ");
    Serial.print(velocity[1]);
    Serial.print(" ");
    Serial.print("PulseL = ");
    Serial.print(position[0]);
    Serial.print(" ");
    Serial.print("PulseR = ");
    Serial.println(position[1]);

    setVelocity(0,0);
}

bool setControlMode(uint16_t value){
    uint8_t result = node.writeSingleRegister(CONTROL_MODE, value);
    return result == node.ku8MBSuccess;
}

bool setControlWord(uint16_t value){
    uint8_t result = node.writeSingleRegister(CONTROL_WORD, value);
    return result == node.ku8MBSuccess;
}

bool setAccTime(uint16_t value){
    uint8_t result1 = node.writeSingleRegister(ACC_TIME_LEFT, value);
    bool ret1 = (result1 == node.ku8MBSuccess);
    uint8_t result2 = node.writeSingleRegister(ACC_TIME_RIGHT, value);
    bool ret2 = (result2 == node.ku8MBSuccess);
    return ret1 && ret2;
}

bool setDecTime(uint16_t value){
    uint8_t result1 = node.writeSingleRegister(DECEL_TIME_LEFT, value);
    bool ret1 = (result1 == node.ku8MBSuccess);
    uint8_t result2 = node.writeSingleRegister(DECEL_TIME_RIGHT, value);
    bool ret2 = (result2 == node.ku8MBSuccess);
    return ret1 && ret2;
}

bool setVelocity(int16_t value_left, int16_t value_right){
    uint8_t result1 = node.writeSingleRegister(TARGET_VEL_LEFT, value_left);
    bool ret1 = (result1 == node.ku8MBSuccess);
    uint8_t result2 = node.writeSingleRegister(TARGET_VEL_RIGHT, value_right);
    bool ret2 = (result2 == node.ku8MBSuccess);
    return ret1 && ret2;
}

bool setTorque(int16_t value_left, int16_t value_right){
    uint8_t result1 = node.writeSingleRegister(TARGET_TORQUE_LEFT, value_left);
    bool ret1 = (result1 == node.ku8MBSuccess);
    uint8_t result2 = node.writeSingleRegister(TARGET_TORQUE_RIGHT, value_right);
    bool ret2 = (result2 == node.ku8MBSuccess);
    return ret1 && ret2;
}

bool getPosition(int32_t* dest){
    uint16_t data[4];
    uint8_t result = node.readHoldingRegisters(ACTUAL_POS_HIGH_LEFT, 4);
    if (result == node.ku8MBSuccess){
        for (uint8_t i = 0; i < 4; i++){
            data[i] = node.getResponseBuffer(i);
        }
        *dest = data[0] << 16;
        *dest = *dest + data[1];
        *(dest+1) = data[2] << 16;
        *(dest+1) = *(dest+1) + data[3];
        return true;
    }
    else{
        return false;
    }
}

bool getVelocity(int16_t* dest){
    uint16_t data[2];    
    uint8_t result = node.readHoldingRegisters(ACTUAL_VEL_LEFT, 2);
    if (result == node.ku8MBSuccess){
        for (uint8_t i = 0; i < 2; i++){
            data[i] = node.getResponseBuffer(i);
        }
        *dest = data[0];
        *(dest+1) = data[1];
        return true;
    }
    else{
        return false;
    }
}
