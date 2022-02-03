#include <ModbusMaster.h>
// https://github.com/4-20ma/ModbusMaster

#include "zltech_item.h"

int32_t position[2] = {0, 0};
int16_t velocity[2] = {0, 0};
uint16_t acc_time = 50;
uint16_t dec_time = 50;

volatile int16_t velocity_target_left, velocity_target_right;

ModbusMaster node;

void setup(){
    Serial.begin(57600);
    Serial1.begin(9600);
    node.begin(1, Serial1);

    Serial.println("Initializing..");

    // Set control mode (3 is Velocity mode).
    // You can find it in the given manual.
    if (!setControlMode(3)){
        Serial.println("Cannot set Velocity Mode.");
    }
    // Set control word (8 is enable tne driver).
    if (!setControlWord(8)){
        Serial.println("Cannot enable motors.");
    }
    
    // Less value mean faster response.
    // You should check the unit of "acc_time" and "dec_time" in the manual.
    if (!setAccTime(acc_time)){
        Serial.println("Cannot set AccTime.");
    }
    if (!setDecTime(dec_time)){
        Serial.println("Cannot set DecTime.");
    }
}

void loop(){
    // We read the volocities and positions then store in 1x2 arrays "velocity" and "position".
    if (!getVelocity(velocity)){
        Serial.println("Cannot get velocity.");
    }
    if (!getPosition(position)){
        Serial.println("Cannot get position.");
    }
    
    // Just print the actual velocities.
    // Note that reading velocities is 10 times of the actual velocities.
    Serial.print("Left = ");
    Serial.print(velocity[0]*0.1);
    Serial.print(" ");
    Serial.print("Right = ");
    Serial.print(velocity[1]*0.1);
    Serial.print(" ");
    
    // The pulses reflect the position of the motors.
    // 1 round or 2*pi radian is equal to 4096 pulse (for the given motors).
    Serial.print("PulseL = ");
    Serial.print(position[0]);
    Serial.print(" ");
    Serial.print("PulseR = ");
    Serial.println(position[1]);

    // Input the velocities for each motors here!
    velocity_target_left  = 0;
    velocity_target_right = 0;
    
    if (!setVelocity(velocity_target_left,velocity_target_right)){
        Serial.print("Cannot set velocities!");
    }
}


// Below this line is the implementation of the MODBUS communication.
// You do not need to modify the lines of code below, unless you want to practice!
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
