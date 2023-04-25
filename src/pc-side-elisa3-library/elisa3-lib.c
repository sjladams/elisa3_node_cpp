#include <stdio.h>
#include "elisa3-lib.h"
#include <tgmath.h>
#include <stdbool.h>
#ifdef _WIN32
#include "windows.h"
#endif

#if defined(__linux__) || defined(__APPLE__)
#include "pthread.h"
	#include <time.h>
	#include <sys/time.h>
#endif

// macro for handling flags byte
#define FRONT_IR_ON(x) ((x) |= (1 << 1))
#define BACK_IR_ON(x) ((x) |= (1 << 0))
#define ALL_IR_ON(x) ((x) |= (1<<0) | (1 << 1))
#define TV_REMOTE_ON(x) ((x) |= (1<<2))
#define SLEEP_ON(x) ((x) = 0x08)
#define CALIBRATION_ON(x) ((x) |= (1<<4))
#define OBSTACLE_AVOID_ON(x) ((x) |= (1<<6))
#define CLIFF_AVOID_ON(x) ((x) |= (1<<7))
#define FRONT_IR_OFF(x) ((x) &= ~(1 << 1))
#define BACK_IR_OFF(x) ((x) &= ~(1 << 0))
#define ALL_IR_OFF(x) ((x) &= ~(1 << 0) & ~(1 << 1))
#define TV_REMOTE_OFF(x) ((x) &= ~(1 << 2))
#define SLEEP_OFF(x) ((x) &= ~(1 << 3))
#define CALIBRATION_OFF(x) ((x) &= ~(1 << 4))
#define OBSTACLE_AVOID_OFF(x) ((x) &= ~(1 << 6))
#define CLIFF_AVOID_OFF(x) ((x) &= ~(1 << 7))

#define RAD_2_DEG 57.2957796

//#define ROBOTS 3
#define NUM_ROBOTS 4
#define PAYLOAD_SIZE 13
#define ADDR_SIZE 2
#define ROBOT_PACKET_SIZE (PAYLOAD_SIZE+ADDR_SIZE)
#define PACKETS_SIZE 64
#define OVERHEAD_SIZE (2*NUM_ROBOTS+1)
#define UNUSED_BYTES 3
#define BULK_NB 4
#define ROBOT_MSG 15 //15, 21, 32

// The usb buffer between the pc and the base-station is 64 bytes.
// Each packet exchanged with the bast-station must contain as the
// first byte the "command id" that at the moment can be either
// "change robot state" (0x27) or "goto base-station bootloader" (0x28).
// In order to optimize the throughput the packet exchanged with the radio
// base-station contains informations to send to four different robots
// simultaneously.
// Each robot must be identified by a 2 byte address, thus we have:
// 64 - 1 - 2*4 = 55 / 4 = 13 bytes usable for the payload of each robot.
//
// Payload content for each robot:
// --------------------------------------------------------------------------
// R | B | G | IR/flags | Right | Left | Leds | ...remaining 6 bytes not used
// --------------------------------------------------------------------------
//
// * R, B, G: values from 0 (OFF) to 100 (ON max power)
// * IR/flags:
//   - first two bits are dedicated to the IRs:
//     0x00 => all IRs off
//     0x01 => back IR on
//     0x02 => front IRs on
//     0x03 => all IRs on
//   - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
//   - fourth bit is used for sleep (1 => go to sleep for 1 minute)
//   - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
//   - sixth bits is reserved (used by radio station)
//   - seventh bit is used for enabling/disabling onboard obstacle avoidance
//   - eight bit is used for enabling/disabling onboard cliff avoidance
// * Right, Left: speed (in percentage); MSBit indicate direction: 1=forward, 0=backward; values from 0 to 100
// * Leds: each bit define whether the corresponding led is turned on (1) or off(0); e.g. if bit0=1 then led0=on
// * remaining bytes free to be used
//
// Overhead content :
// - command: 1 byte, indicates which command the packet refer to
// - address: 2 bytes per robot

//formation + laplacian
int L[18][18] = {{2, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-1, 2, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-1, 0, 2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, -1, -1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int Bx[18][18] = {{0, 2, 234, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-2, 0, 0, -292, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-234, 0, 0, -524, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 292, 524, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int By[18][18] = {{0, -558, -224, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {558, 0, 0, 141, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {224, 0, 0, -193, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, -141, 193, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
//int By[18][18] = {{0, -250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};


int error = 20; //in mm
int u_max = 30;
float speed_max = 0;
float speed_temp[100] = {0};
bool reset_flag[100] = {false};
bool reset_theta[100] = {false};
const int ROBOTS = 4;
unsigned int currNumRobots = 4;

//float testing = 0.0;
// robots
int type = 0;
int robotAddress[100];
char leftSpeed[100];
char rightSpeed[100];
signed int xPos[100];
signed int yPos[100];
signed int xPos_fixed[100];
signed int yPos_fixed[100];
char redLed[100], greenLed[100], blueLed[100];
unsigned int proxValue[100][8];
unsigned int proxAmbientValue[100][8];
unsigned int groundValue[100][4];
unsigned int groundAmbientValue[100][4];
unsigned int batteryAdc[100];
unsigned int batteryPercent[100];
signed int accX[100], accY[100], accZ[100];
unsigned char selector[100];
unsigned char tvRemote[100];
unsigned char flagsRX[100];
unsigned char flagsTX[100][2];
unsigned char smallLeds[100];
signed long int leftMotSteps[100], rightMotSteps[100];
signed int robTheta[100], robXPos[100], robYPos[100];
signed int robTheta_filtered[100] = {0};
signed int testing[100];
signed int robXPos_fixed[100], robYPos_fixed[100], robXPos_temp1[100];
signed int robYPos_temp1[100]={0};
unsigned int trigger_count[100];
unsigned int reset_count[100] = {0};
unsigned int trigger_count_previous[100];
unsigned char sleepEnabledFlag[100];
bool init_flag = false;
unsigned int turn[100];
signed int rSpeed[100];

//profiling
unsigned long int control_time[100];
unsigned long int loop_time[100];
unsigned long int init_time[100];
unsigned long int comm_time[100];

// Communication
char RX_buffer[64]={0};         // Last packet received from base station
char TX_buffer[64]={0};         // Next packet to send to base station
#ifdef _WIN32
DWORD commThreadId;
HANDLE commThread;
HANDLE mutexTx;
HANDLE mutexRx;
HANDLE mutexThread;
#endif
#if defined(__linux__) || defined(__APPLE__)
pthread_t commThread;
pthread_mutex_t mutexTx;
pthread_mutex_t mutexRx;
pthread_mutex_t mutexThread;
#endif
double numOfErrors[100], numOfPackets=0, errorPercentage[100];
unsigned char lastMessageSentFlag[100];
unsigned char calibrationSent[100];
unsigned char calibrateOdomSent[100];
unsigned char stopTransmissionFlag = 0;
unsigned int currPacketId = 0;
signed int currRobotId = -1;
signed int currRobotId_L = 0;
unsigned int nb_init_rounds = 0;
unsigned char usbCommOpenedFlag = 0;
unsigned int neigh_id = ROBOTS+1;

// functions declaration
#ifdef _WIN32
DWORD WINAPI CommThread( LPVOID lpParameter);
#endif
#if defined(__linux__) || defined(__APPLE__)
void *CommThread(void *arg);
#endif

char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
        return ((-value)&0x7F);
    }
}

int computeVerticalAngle(signed int x, signed int y) {

    int currentAngle = 0;

    currentAngle = (signed int)(atan2f((float)x, (float)y)*RAD_2_DEG);

    if(currentAngle<0) {
        currentAngle = 360+currentAngle;	// angles from 0 to 360
    }

    return currentAngle;

}

int getIdFromAddress(int address) {
    int i=0;
    for(i=0; i<currNumRobots; i++) {
        if(address == robotAddress[i]) {
            return i;
        }
    }
    return -1;
}

void enableReset(int address){
    int i=0;
    for(i=0; i<currNumRobots; i++) {
        if(address == robotAddress[i]) {
            break;
        }
    }

    reset_flag[i] = true;
}

void resetTheta(int address){
    int i=0;
    for(i=0; i<currNumRobots; i++) {
        if(address == robotAddress[i]) {
            break;
        }
    }

    reset_theta[i] = true;
}

void disableReset(int address){
    int i=0;
    for(i=0; i<currNumRobots; i++) {
        if(address == robotAddress[i]) {
            break;
        }
    }

    reset_flag[i] = false;
}

void startCommunication(int *robotAddr, int numRobots) {
    if(usbCommOpenedFlag==1) {
        return;
    }
    openCommunication();
    TX_buffer[0]=0x27;

#ifdef _WIN32
    commThread = CreateThread(NULL, 0, CommThread, NULL, 0, &commThreadId);
    mutexTx = CreateMutex(NULL, FALSE, NULL);
    mutexRx = CreateMutex(NULL, FALSE, NULL);
    mutexThread = CreateMutex(NULL, FALSE, NULL);
#endif

#if defined(__linux__) || defined(__APPLE__)
    if(pthread_create(&commThread, NULL, CommThread, NULL)) {
        fprintf(stderr, "Error creating thread\n");
    }
    if (pthread_mutex_init(&mutexTx, NULL) != 0) {
        printf("\n mutex init failed\n");
    }
    if (pthread_mutex_init(&mutexRx, NULL) != 0) {
        printf("\n mutex init failed\n");
    }
    if (pthread_mutex_init(&mutexThread, NULL) != 0) {
        printf("\n mutex init failed\n");
    }
#endif

    setRobotAddresses(robotAddr, numRobots);

    usbCommOpenedFlag = 1;

}

void stopCommunication() {
    closeCommunication();

#ifdef _WIN32
    TerminateThread(commThread, 0);
    CloseHandle(commThread);
    CloseHandle(mutexTx);
    CloseHandle(mutexRx);
    CloseHandle(mutexThread);
#endif

#if defined(__linux__) || defined(__APPLE__)
    pthread_cancel(commThread);
	pthread_mutex_destroy(&mutexTx);
	pthread_mutex_destroy(&mutexRx);
	pthread_mutex_destroy(&mutexThread);
#endif

    usbCommOpenedFlag = 0;

}

void setMutexTx() {
#ifdef _WIN32
    WaitForSingleObject(mutexTx, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexTx);
#endif
}

void freeMutexTx() {
#ifdef _WIN32
    ReleaseMutex(mutexTx);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexTx);
#endif
}

void setMutexRx() {
#ifdef _WIN32
    WaitForSingleObject(mutexRx, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexRx);
#endif
}

void freeMutexRx() {
#ifdef _WIN32
    ReleaseMutex(mutexRx);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexRx);
#endif
}

void setMutexThread() {
#ifdef _WIN32
    WaitForSingleObject(mutexThread, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexThread);
#endif
}

void freeMutexThread() {
#ifdef _WIN32
    ReleaseMutex(mutexThread);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexThread);
#endif
}

unsigned char checkConcurrency(int id) {
    int packetId = currPacketId;
    if(id>=(packetId*4+0) && id<=(packetId*4+3)) {    // the current robot data could be accessed concurrently so beware!
        return 1;
    } else {
        return 0;
    }
}

void setLeftSpeed(int robotAddr, char value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        leftSpeed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setRightSpeed(int robotAddr, char value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        rightSpeed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

// Newly added functions to save xPos and yPos of robots in an array to be transmitted
void setXpos(int robotAddr, int value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        xPos[id] = value;

        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setXpos_fixed(int robotAddr, int value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        xPos_fixed[id] = value;

        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setYpos(int robotAddr, int value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        yPos[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setYpos_fixed(int robotAddr, int value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        yPos_fixed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setLeftSpeedForAll(char *value) {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        leftSpeed[i] = value[i];
    }
    freeMutexTx();
}

void setRightSpeedForAll(char *value) {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        rightSpeed[i] = value[i];
    }
    freeMutexTx();
}


// SELF-MADE
void setAllColors(int robotAddr, char red, char green, char blue) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        redLed[id] = red;
        blueLed[id] = blue;
        greenLed[id] = green;
        if(enableMut) {
            freeMutexTx();
        }
    }
}


void setRed(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    //unsigned char enableMut = 1;
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
//        if(value > 100) {
//            value = 100;
//        }
        if(enableMut) {
            setMutexTx();
        }
        //printf("[%d] - Red - id: %d - value: %d - enabled: %d, \r\n", robotAddr, id, value, enableMut);
        redLed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setGreen(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    //unsigned char enableMut = 1;
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
//        if(value > 100) {
//            value = 0;
//        }
        if(enableMut) {
            setMutexTx();
        }
        //printf("[%d] - Green - id: %d - value: %d - enabled: %d, \r\n", robotAddr, id, value, enableMut);
        greenLed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setBlue(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    //unsigned char enableMut = 1;
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
//        if(value > 100) {
//            value = 100;
//        }
        if(enableMut) {
            setMutexTx();
        }
        //printf("[%d] - Blue - id: %d - value: %d - enabled: %d, \r\n", robotAddr, id, value, enableMut);
        blueLed[id] = value;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setRedForAll(unsigned char *value) {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        redLed[i] = value[i];
    }
    freeMutexTx();
}

void setGreenForAll(unsigned char *value) {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        greenLed[i] = value[i];
    }
    freeMutexTx();
}

void setBlueForAll(unsigned char *value) {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        blueLed[i] = value[i];
    }
    freeMutexTx();
}

void turnOnFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        FRONT_IR_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOffFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        FRONT_IR_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOnBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        BACK_IR_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOffBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        BACK_IR_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOnAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        ALL_IR_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOffAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        ALL_IR_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void enableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        TV_REMOTE_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void disableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        TV_REMOTE_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void enableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        SLEEP_ON(flagsTX[id][0]);
        sleepEnabledFlag[id] = 1;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void disableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        SLEEP_OFF(flagsTX[id][0]);
        sleepEnabledFlag[id] = 0;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void enableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        OBSTACLE_AVOID_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void disableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        OBSTACLE_AVOID_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void enableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        CLIFF_AVOID_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void disableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        CLIFF_AVOID_OFF(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void resetRobotData(int robotIndex) {
    redLed[robotIndex] = 0;
    blueLed[robotIndex] = 0;
    greenLed[robotIndex] = 0;
    flagsTX[robotIndex][0] = 0;
    rightSpeed[robotIndex] = 0;
    leftSpeed[robotIndex] = 0;
    smallLeds[robotIndex] = 0;
    flagsTX[robotIndex][1] = 0;
}

void setRobotAddress(int robotIndex, int robotAddr) {
    unsigned char enableMut=0;
    if(robotIndex>=0 && robotIndex<=(currNumRobots-1)) {    // the index must be within the robots list size
        enableMut = checkConcurrency(robotIndex);
        if(enableMut) {
            setMutexTx();
        }
        robotAddress[robotIndex] = robotAddr;
        resetRobotData(robotIndex);
        if(enableMut) {
            freeMutexTx();
        }
        waitForUpdate(robotAddr, 100000);   // wait for the data of the current robot are received (otherwise old data of the previous robot would be sent to the user)
    }
}

void setRobotAddresses(int *robotAddr, int numRobots) {
    int i = 0;
    setMutexTx();
    for(i=0; i<numRobots; i++) {
        robotAddress[i] = robotAddr[i];
        resetRobotData(i);
    }
    freeMutexTx();
    for(i=0; i<numRobots; i+=4) {   // wait for correct data (data from current robots and not previous ones) received from all the robots
        waitForUpdate(i, 100000);
    }
    waitForUpdate(numRobots-1, 100000); // if numRobots is a multiple of 8 then this call is useless...don't care
    currNumRobots = numRobots;
}

unsigned int getProximity(int robotAddr, int proxId) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = proxValue[id][proxId];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getProximityAmbient(int robotAddr, int proxId) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = proxAmbientValue[id][proxId];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getGround(int robotAddr, int groundId) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = groundValue[id][groundId];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getGroundAmbient(int robotAddr, int groundId) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = groundAmbientValue[id][groundId];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

void getAllProximity(int robotAddr, unsigned int* proxArr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    int i = 0;
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        for(i=0; i<8; i++) {
            proxArr[i] = proxValue[id][i];
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
}

void getAllProximityAmbient(int robotAddr, unsigned int* proxArr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    int i = 0;
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        for(i=0; i<8; i++) {
            proxArr[i] = proxAmbientValue[id][i];
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
}

void getAllGround(int robotAddr, unsigned int* groundArr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    int i = 0;
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        for(i=0; i<4; i++) {
            groundArr[i] = groundValue[id][i];
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
}

void getAllGroundAmbient(int robotAddr, unsigned int* groundArr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    int i = 0;
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        for(i=0; i<4; i++) {
            groundArr[i] = groundAmbientValue[id][i];
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
}

void getAllProximityFromAll(unsigned int proxArr[][8]) {
    int i = 0, j = 0;
    setMutexRx();
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<8; j++) {
            proxArr[i][j] = proxValue[i][j];
        }
    }
    freeMutexRx();
}

void getAllProximityAmbientFromAll(unsigned int proxArr[][8]) {
    int i = 0, j = 0;
    setMutexRx();
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<8; j++) {
            proxArr[i][j] = proxAmbientValue[i][j];
        }
    }
    freeMutexRx();
}

void getAllGroundFromAll(unsigned int groundArr[][4]) {
    int i = 0, j = 0;
    setMutexRx();
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<4; j++) {
            groundArr[i][j] = groundValue[i][j];
        }
    }
    freeMutexRx();
}

void getAllGroundAmbientFromAll(unsigned int groundArr[][4]) {
    int i = 0, j = 0;
    setMutexRx();
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<4; j++) {
            groundArr[i][j] = groundAmbientValue[i][j];
        }
    }
    freeMutexRx();
}

unsigned int getBatteryAdc(int robotAddr) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = batteryAdc[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned long int getProfilingTime(int robotAddr, int type) {
    unsigned long int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        switch (type) {
            case 0:
                tempVal = control_time[id];
                break;
            case 1:
                tempVal = loop_time[id];
                break;
            case 2:
                tempVal = init_time[id];
                break;
            case 3:
                tempVal = comm_time[id];
                break;

        }
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getBatteryPercent(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned int tempVal=0;
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        if(batteryAdc[id] >= 934) {           // 934 is the measured adc value when the battery is charged
            batteryPercent[id] = 100;
        } else if(batteryAdc[id] <= 780) {    // 780 is the measrued adc value when the battery is discharged
            batteryPercent[id] = 0;
        } else {
            batteryPercent[id] = (unsigned int)((float)(((float)batteryAdc[id]-780.0)/(934.0-780.0))*100.0);
        }
        tempVal = batteryPercent[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getAccX(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = accX[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getAccY(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = accY[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getAccZ(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = accZ[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned char getSelector(int robotAddr) {
    unsigned char tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = selector[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned char getTVRemoteCommand(int robotAddr) {
    unsigned char tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = tvRemote[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomTheta(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robTheta[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

float getSpeed(int robotAddr) {
    float tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = speed_temp[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomTheta_filtered(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = testing[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int setTheta(int robotAddr, signed int value) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        robTheta_filtered[id] = value;
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomXpos(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robXPos[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getXpos_fixed(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robXPos_fixed[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomYpos(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robYPos[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getYpos_fixed(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robYPos_fixed[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomXpos_temp1(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robXPos_temp1[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed int getOdomYpos_temp1(int robotAddr) {
    signed int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = robYPos_temp1[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}


unsigned int getNbEvents(int robotAddr) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = trigger_count[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getNbResets(int robotAddr) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = reset_count[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

unsigned int getTurn(int robotAddr) {
    unsigned int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = turn[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

void setSmallLed(int robotAddr, int ledId, int state) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        if(state==0) {
            smallLeds[id] &= ~(1<<ledId);
        } else {
            smallLeds[id] |= (1<<ledId);
        }
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOffSmallLeds(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        smallLeds[id] = 0;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void turnOnSmallLeds(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        smallLeds[id] = 0xFF;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

int getVerticalAngle(int robotAddr) {
    int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = computeVerticalAngle(accX[id], accY[id]);
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

void calibrateSensors(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        calibrationSent[id] = 0;
        CALIBRATION_ON(flagsTX[id][0]);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void calibrateSensorsForAll() {
    int i = 0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        calibrationSent[i] = 0;
        CALIBRATION_ON(flagsTX[i][0]);
    }
    freeMutexTx();
}

void startOdometryCalibration(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        calibrateOdomSent[id] = 0;
        flagsTX[id][1] |= (1<<0);
        if(enableMut) {
            freeMutexTx();
        }
    }
}

unsigned char robotIsCharging(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        if((flagsRX[id]&0x01) == 0x01) {
            return 1;
        } else {
            return 0;
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
    return 0;
}

unsigned char robotIsCharged(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        if((flagsRX[id]&0x04) == 0x04) {
            return 1;
        } else {
            return 0;
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
    return 0;
}

unsigned char buttonIsPressed(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        if((flagsRX[id]&0x02) == 0x02) {
            return 1;
        } else {
            return 0;
        }
        if(enableMut) {
            freeMutexRx();
        }
    }
    return 0;
}

void resetFlagTX(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        flagsTX[id][0] = 0;
        flagsTX[id][1] = 0;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

unsigned char getFlagTX(int robotAddr, int flagInd) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return flagsTX[id][flagInd];
    }
    return -1;
}

unsigned char getFlagRX(int robotAddr) {
    unsigned char tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = flagsRX[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed long int getLeftMotSteps(int robotAddr) {
    signed long int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = leftMotSteps[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

signed long int getRightMotSteps(int robotAddr) {
    signed long int tempVal=0;
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        tempVal = rightMotSteps[id];
        if(enableMut) {
            freeMutexRx();
        }
        return tempVal;
    }
    return -1;
}

bool getInitFlag(){
    return init_flag;
}

float getUmax(){
    return speed_max;
}

void resetMessageIsSentFlag(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        lastMessageSentFlag[id] = 0;
        if(enableMut) {
            freeMutexRx();
        }
    }
}

unsigned char messageIsSent(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexRx();
        }
        if(lastMessageSentFlag[id]==3) {
            if(enableMut) {
                freeMutexRx();
            }
            return 1;
        } else {
            if(enableMut) {
                freeMutexRx();
            }
            return 0;
        }
    }
    return -1;
}

double getRFQuality(int robotAddr) {
    double tempVal=0;
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        setMutexThread();
        tempVal = 100.0-errorPercentage[id];
        freeMutexThread();
        return tempVal;
    }
    return -1;
}

void stopTransferData() {
    stopTransmissionFlag = 1;
}

void resumeTransferData() {
    stopTransmissionFlag = 0;
}


void setCompletePacket(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds) {
    int id = getIdFromAddress(robotAddr);
    unsigned char enableMut = checkConcurrency(id);
    if(id>=0) {
        if(enableMut) {
            setMutexTx();
        }
        redLed[id] = red;
        blueLed[id] = blue;
        greenLed[id] = green;
        flagsTX[id][0] = flags[0];
        flagsTX[id][1] = flags[1];
        leftSpeed[id] = left;
        rightSpeed[id] = right;
        smallLeds[id] = leds;
        if(enableMut) {
            freeMutexTx();
        }
    }
}

void setCompletePacketForAll(int *robotAddr, char *red, char *green, char *blue, char flags[][2], char *left, char *right, char *leds) {
    int i=0;
    setMutexTx();
    for(i=0; i<currNumRobots; i++) {
        redLed[i] = red[i];
        blueLed[i] = blue[i];
        greenLed[i] = green[i];
        flagsTX[i][0] = flags[i][0];
        flagsTX[i][1] = flags[i][1];
        leftSpeed[i] = left[i];
        rightSpeed[i] = right[i];
        smallLeds[i] = leds[i];
        robotAddress[i] = robotAddr[i];
    }
    freeMutexTx();
}

unsigned char sendMessageToRobot(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds, unsigned long us) {
    int robotAddrArr[1] = {robotAddr};
    setRobotAddresses(robotAddrArr, 1);
    setCompletePacket(robotAddr, red, green, blue, flags, left, right, leds);
    return waitForUpdate(robotAddr, us);
}

unsigned char waitForUpdate(int robotAddr, unsigned long us) {
#ifdef _WIN32
    SYSTEMTIME startTime;
    FILETIME startTimeF;
    ULONGLONG startTime64;
    SYSTEMTIME exitTime;
    FILETIME exitTimeF;
    ULONGLONG exitTime64;
    GetSystemTime(&startTime);
    SystemTimeToFileTime(&startTime, &startTimeF);
    startTime64 = (((ULONGLONG) startTimeF.dwHighDateTime) << 32) + startTimeF.dwLowDateTime;
#endif

#if defined(__linux__) || defined(__APPLE__)
    struct timeval startTime, exitTime;
    gettimeofday(&startTime, NULL);
	gettimeofday(&exitTime, NULL);
#endif

    resetMessageIsSentFlag(robotAddr);

    while(messageIsSent(robotAddr)==0) {
#ifdef _WIN32
        GetSystemTime(&exitTime);
        SystemTimeToFileTime(&exitTime, &exitTimeF);
        exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;
        if(((exitTime64-startTime64)/10 > us)) {
            return 1;
        }
#endif

#if defined(__linux__) || defined(__APPLE__)
        gettimeofday(&exitTime, NULL);
        if((((exitTime.tv_sec * 1000000 + exitTime.tv_usec)-(startTime.tv_sec * 1000000 + startTime.tv_usec)) > us)) {
            return 1;
        }
#endif
    }

    return 0;
}

void transferData() {

    int err = 0;

#ifdef _WIN32
    WaitForSingleObject(mutexTx, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexTx);
#endif

    int i;


    if (init_flag) {
        // find id of robot that needs updating
        if (currPacketId == 0) {
            neigh_id = ROBOTS + 1;
            for (i = 0; i < ROBOTS; i++) {
                //printf("Robot %d, trigger old: %d, trigger: %d \n", i, trigger_count_previous[i], trigger_count[i]);
                if (trigger_count_previous[i] < trigger_count[i]) {
                    if ( (trigger_count[i]-trigger_count_previous[i])>(trigger_count[neigh_id]-trigger_count_previous[neigh_id])){
                        neigh_id = i;

                    }
                    //printf("ROBOT %d needs to trigger\n", i);

                    //break;
                }
            }
        }

		//printf("sending operational msgs\n");
        //printf("TRIGGER %d \n", neigh_id);

        int i;
        int j;
        for (i = 0; i < BULK_NB; i++) {
            if (sleepEnabledFlag[i] == 1) {
                for (j = 1; j < ROBOT_MSG - 2; j++) {
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + j] = 0x00;
                }
            } else {
                // operational packet

                //printf("Send data of robot %d \n", neigh_id);
                trigger_count_previous[neigh_id] = trigger_count[neigh_id];

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = neigh_id;

                // must be changed to the angle of the robot -- live update
                //TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = speed(rightSpeed[currPacketId * BULK_NB + i]);// speed right
                //TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = speed(leftSpeed[currPacketId * BULK_NB + i]); // speed left
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = robTheta_filtered[currPacketId * BULK_NB + i]& 0xFF;// speed right
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = robTheta_filtered[currPacketId * BULK_NB + i] >> 8 & 0xFF; // speed left


                if (neigh_id < ROBOTS + 1) {
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = CALIBRATION_ON(flagsTX[currPacketId * BULK_NB + i][0]);
                } else {
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = CALIBRATION_OFF(flagsTX[currPacketId * BULK_NB + i][0]);
                }

                // Reset robot odometry
                if (reset_flag[(currPacketId * BULK_NB + i)]){
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = 0xE0;
                    disableReset((currPacketId * BULK_NB + i));
                    reset_flag[(currPacketId * BULK_NB + i)] = false;
                }else{
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = 0x00;
                }

                // Reset robot orientation
                if (reset_theta[(currPacketId * BULK_NB + i)]){
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] |= 0x15;
                    reset_theta[(currPacketId * BULK_NB + i)] = false;
                }else{
                    TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] |= 0x00;
                }

                // Own position -- live update
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (xPos[currPacketId * BULK_NB + i]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (xPos[currPacketId * BULK_NB + i]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (yPos[currPacketId * BULK_NB + i]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (yPos[currPacketId * BULK_NB + i]) >> 8 & 0xFF;

                //Neighbours position that needs updating -- broadcasted update
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (robXPos_fixed[neigh_id]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (robXPos_fixed[neigh_id]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (robYPos_fixed[neigh_id]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (robYPos_fixed[neigh_id]) >> 8 & 0xFF;

            }
            TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                    (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
            TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
        }

//        if (reset_flag){
//            if ((currPacketId+1)>=(ROBOTS/BULK_NB)) {
//                disableReset();
//                reset_flag = false;
//                reset_theta = false;
//            }
//        }
    }else{
        // Init packets

        // update robots with all the positions
        switch (type){

        case 0 :
			printf("sending type 0, %d, %d\n", currPacketId, (int) ceil(currNumRobots/4.0));
            // update parameters
            for (i = 0; i < BULK_NB; i++) {
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = (currPacketId * BULK_NB + i) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = (robTheta_filtered[currPacketId * BULK_NB + i]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = (robTheta_filtered[currPacketId * BULK_NB + i])>> 8 & 0xFF;

                // Own position -- live update
				TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = (ROBOTS) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (xPos[currPacketId * BULK_NB + i]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (xPos[currPacketId * BULK_NB + i]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (yPos[currPacketId * BULK_NB + i]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (yPos[currPacketId * BULK_NB + i]) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (error) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (error) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (u_max) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (u_max) >> 8 & 0xFF;

                // send init phase + currently send robot positions
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = 0x80;

                // Reset orientation at init
                //TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] |= 0x15;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                        (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
            }

            if ((currPacketId+1)>=((int) ceil(currNumRobots/4.0))) {
                type = 1;
            }


            break;
        case 1 :
            // update positions
			printf("sending type 1, %d, %d\n", currPacketId, currRobotId);
            for (i = 0; i < BULK_NB; i++) {
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = (xPos[currRobotId * 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = (xPos[currRobotId * 3]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = (yPos[currRobotId * 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = (yPos[currRobotId * 3]) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (xPos[currRobotId * 3 + 1]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (xPos[currRobotId * 3 + 1]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (yPos[currRobotId * 3 + 1]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (yPos[currRobotId * 3 + 1]) >> 8 & 0xFF;

                //printf("RobYtemp %d\n", yPos[currRobotId*3+1]);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (xPos[currRobotId * 3 + 2]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (xPos[currRobotId * 3 + 2]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (yPos[currRobotId * 3 + 2]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (yPos[currRobotId * 3 + 2]) >> 8 & 0xFF;

                // send init phase + currently send robot positions
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = 0x80 | (currRobotId+1);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                        (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
            }
            //currRobotId += 3;

            if ((robYPos_temp1[ROBOTS-1] > 0) && ((currPacketId+1)>=((int) ceil(currNumRobots/4.0)))){
                type = 2;
                currRobotId_L = -1;
            }

            break;

        case 2 :
            // update L
//			printf("sending type 2, %d, %d\n", currPacketId, currRobotId_L);
            for (i = 0; i < BULK_NB; i++) {
                printf("sending type 2, %d, %d\n", currPacketId, currRobotId_L);
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) >> 8 & 0xFF;

                //printf("RobYtemp %d\n", yPos[currRobotId*3+1]);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (L[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) >> 8 & 0xFF;

                // send init phase + currently send robot positions
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = 0x90 | (currRobotId_L);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                        (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
            }

            if ((currPacketId+1)>=((int) ceil(currNumRobots/4.0)) && (currRobotId_L+1)>=(ROBOTS/6)) {
                type = 3;
                currRobotId_L = -1;
            }

            break;

        case 3 :
            // update Bx // B[0]
		
            for (i = 0; i < BULK_NB; i++) {
				printf("sending type 3, Bx[%d][%d] %d\n", currPacketId * BULK_NB + i, currRobotId_L * 3 + 2 ,Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]);
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) >> 8 & 0xFF;

                //printf("RobYtemp %d\n", yPos[currRobotId*3+1]);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (Bx[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) >> 8 & 0xFF;

                // send init phase + currently send robot positions
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = 0xC0 | (currRobotId_L);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                        (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
            }

            if ((currPacketId+1)>=((int) ceil(currNumRobots/4.0)) && (currRobotId_L+1)>=(ROBOTS/6)) {
                type = 4;
                currRobotId_L = -1;
            }

            break;
        case 4 :
			printf("sending type 4, %d, %d\n", currPacketId, currRobotId_L);
            // update By // B[1]
            for (i = 0; i < BULK_NB; i++) {
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 1] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 2] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 3] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 5] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 1]) >> 8 & 0xFF;

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 6] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 7] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 2]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 8] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 9] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 3]) >> 8 & 0xFF;

                //printf("RobYtemp %d\n", yPos[currRobotId*3+1]);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + 10] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 11] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 4]) >> 8 & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 12] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) & 0xFF;
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 13] = (By[currPacketId * BULK_NB + i][currRobotId_L * 3 + 5]) >> 8 & 0xFF;

                // send init phase + currently send robot positions
                TX_buffer[(i * ROBOT_PACKET_SIZE) + 4] = 0xD0 | (currRobotId_L);

                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG - 1] =
                        (robotAddress[currPacketId * BULK_NB + i] >> 8) & 0xFF;     // address of the robot
                TX_buffer[(i * ROBOT_PACKET_SIZE) + ROBOT_MSG] = robotAddress[currPacketId * BULK_NB + i] & 0xFF;
            }

            if ((currPacketId+1)>=((int) ceil(currNumRobots/4.0)) && (currRobotId_L+1)>=(ROBOTS/6)) {
                init_flag = true;
                //currRobotId_L = 0;
            }
            break;
		}
        
        //printf("send init data, %d\n", currRobotId);
    }

#ifdef _WIN32
    ReleaseMutex(mutexTx);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexTx);
#endif

    // transfer the data to the base-station
    err = usb_send(TX_buffer, PACKETS_SIZE);
    if (err < 0) {
        printf("send error!\n");
    }

#ifdef _WIN32
    WaitForSingleObject(mutexTx, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexTx);
#endif

    for (i = 0; i < BULK_NB; i++) {
        calibrationSent[currPacketId * BULK_NB + i]++;

        calibrateOdomSent[currPacketId * BULK_NB + i]++;

        if (calibrationSent[currPacketId * BULK_NB + i] > 2) {
            CALIBRATION_OFF(flagsTX[currPacketId * BULK_NB + i][0]);
        }
        if (calibrateOdomSent[currPacketId * BULK_NB + i] > 2) {
            flagsTX[currPacketId * BULK_NB + i][1] &= ~(1 << 0);
        }
    }

#ifdef _WIN32
    ReleaseMutex(mutexTx);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexTx);
#endif


    RX_buffer[0] = 0;
    RX_buffer[16] = 0;
    RX_buffer[32] = 0;
    RX_buffer[48] = 0;
    err = usb_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
    if (err < 0) {
        printf("receive error!\n");
    }

#ifdef _WIN32
    WaitForSingleObject(mutexRx, INFINITE);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_lock(&mutexRx);
#endif

    // when the flag "lastMessageSentFlag" is reset we aren't sure the current message is really sent to the radio module
    // for next transmission to the robots so we wait the message is sent twice

    for (i = 0; i < BULK_NB; i++) {
        if (lastMessageSentFlag[currPacketId * BULK_NB + i] == 0) {
            lastMessageSentFlag[currPacketId * BULK_NB + i] = 1;
        } else if (lastMessageSentFlag[currPacketId * BULK_NB + i] == 1) {
            lastMessageSentFlag[currPacketId * BULK_NB + i] = 2;
        }
    }

    // the base-station returns this "error" codes:
    // - 0 => transmission succeed (no ack received though)
    // - 1 => ack received (should not be returned because if the ack is received, then the payload is read)
    // - 2 => transfer failed
    for (i = 0; i<BULK_NB; i++) {

        if ((int) ((unsigned char) RX_buffer[ROBOT_MSG*i + i]) <= 2) { // if something goes wrong skip the data
            //printf("transfer failed to robot %d, %d (addr=%d)\n", i, currPacketId ,RX_buffer[ROBOT_MSG*i + i]);
            numOfErrors[currPacketId * BULK_NB + i]++;
        } else {
            if (lastMessageSentFlag[currPacketId * BULK_NB + i] == 2) {
                lastMessageSentFlag[currPacketId * BULK_NB + i] = 3;
            }
            //printf("received robot %d, %d, type = %d \n", currPacketId, i, (int) ((unsigned char) RX_buffer[ROBOT_MSG*i + i]));
            // extract the sensors data for the first robot based on the packet id (first byte):
            // id=3 | prox0         | prox1         | prox2         | prox3         | prox5         | prox6         | prox7         | flags
            // id=4 | prox4         | gound0        | ground1       | ground2       | ground3       | accX          | accY          | tv remote
            // id=5 | proxAmbient0  | proxAmbient1  | proxAmbient2  | proxAmbient3  | proxAmbient5  | proxAmbient6  | proxAmbient7  | selector
            // id=6 | proxAmbient4  | goundAmbient0 | goundAmbient1 | goundAmbient2 | goundAmbient3 | accZ          | battery       | free byte
            switch ((int) ((unsigned char) RX_buffer[ROBOT_MSG*i + i])) {
                case 3:
                    proxValue[currPacketId * BULK_NB + i][0] = (((signed int) RX_buffer[ROBOT_MSG*i + i + 2] << 8) |
                                                                (unsigned char) RX_buffer[ROBOT_MSG*i + i +1]);
                    proxValue[currPacketId * BULK_NB + i][1] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +4] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +3];
                    proxValue[currPacketId * BULK_NB + i][2] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +6] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +5];
                    proxValue[currPacketId * BULK_NB + i][3] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    proxValue[currPacketId * BULK_NB + i][5] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +10] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +9];
                    proxValue[currPacketId * BULK_NB + i][6] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +12] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +11];
                    proxValue[currPacketId * BULK_NB + i][7] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +14] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +13];
                    flagsRX[currPacketId * BULK_NB + i] = (unsigned char) RX_buffer[ROBOT_MSG*i + i +15];
                    break;

                case 4:
                    proxValue[currPacketId * BULK_NB + i][4] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +2] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +1];
                    groundValue[currPacketId * BULK_NB + i][0] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +4] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +3];
                    groundValue[currPacketId * BULK_NB + i][1] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +6] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +5];
                    groundValue[currPacketId * BULK_NB + i][2] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    groundValue[currPacketId * BULK_NB + i][3] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +10] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +9];
                    accX[currPacketId * BULK_NB + i] = (int) ((RX_buffer[ROBOT_MSG*i + i +12] << 8) | (RX_buffer[ROBOT_MSG*i + i +11]));
                    accY[currPacketId * BULK_NB + i] = (int) ((RX_buffer[ROBOT_MSG*i + i +14] << 8) | (RX_buffer[ROBOT_MSG*i + i +13]));
                    tvRemote[currPacketId * BULK_NB + i] = (unsigned char) RX_buffer[ROBOT_MSG*i + i +15];
                    break;

                case 5:
                    //proxAmbientValue[currPacketId * BULK_NB + i][0] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +2] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +1];
                    //proxAmbientValue[currPacketId * BULK_NB + i][1] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +4] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +3];
                    //proxAmbientValue[currPacketId * BULK_NB + i][2] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +6] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +5];
                    //proxAmbientValue[currPacketId * BULK_NB + i][3] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    init_time[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +4] << 24) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +3] << 16) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +2] << 8) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +1]));
                    comm_time[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +8] << 24) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +7] << 16) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +6] << 8) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +5]));
                    if ((((unsigned int) RX_buffer[ROBOT_MSG*i + i + 9])&0x80) == 0x80) {
                        rSpeed[currPacketId * BULK_NB + i] =   ((unsigned int) RX_buffer[ROBOT_MSG*i + i + 9])&0x7F;
                    }else{
                        rSpeed[currPacketId * BULK_NB + i] =   -(((unsigned int) RX_buffer[ROBOT_MSG*i + i + 9])&0x7F);
                    }
                    turn[currPacketId * BULK_NB + i] =   ((unsigned int) RX_buffer[ROBOT_MSG*i + i + 10]);
                    testing[currPacketId * BULK_NB + i] =
                            ((((signed int) RX_buffer[ROBOT_MSG*i + i +12] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +11])/10);
                    reset_count[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +14] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +13];
                    selector[currPacketId * BULK_NB + i] = (unsigned char) RX_buffer[ROBOT_MSG*i + i +15];
                    break;

                case 6:
                    //proxAmbientValue[currPacketId * BULK_NB + i][4] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +2] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +1];
                    //groundAmbientValue[currPacketId * BULK_NB + i][0] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +4] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +3];
                    //groundAmbientValue[currPacketId * BULK_NB + i][1] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +6] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +5];
                    //groundAmbientValue[currPacketId * BULK_NB + i][2] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    loop_time[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +4] << 24) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +3] << 16) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +2] << 8) |
                                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +1]));
//                    control_time[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +8] << 24) |
//                                                                ((unsigned char) RX_buffer[ROBOT_MSG*i + i +7] << 16) |
//                                                                ((unsigned char) RX_buffer[ROBOT_MSG*i + i +6] << 8) |
//                                                                ((unsigned char) RX_buffer[ROBOT_MSG*i + i +5]));
                    reset_count[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    speed_temp[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +10] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +9];
                    accZ[currPacketId * BULK_NB + i] = (int) ((RX_buffer[ROBOT_MSG*i + i +12] << 8) | (RX_buffer[ROBOT_MSG*i + i +11]));
                    robYPos_temp1[currPacketId * BULK_NB + i] = ((signed int) RX_buffer[ROBOT_MSG*i + i +12] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +11];
                    robXPos_temp1[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +14] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +13];
                    //trigger_count[currPacketId * BULK_NB + i] = ((unsigned int) RX_buffer[ROBOT_MSG*i + i +15] << 8);

                    if (speed_temp[currPacketId * BULK_NB + i] > speed_max){
                        speed_max = speed_temp[currPacketId * BULK_NB + i];
                    }else if (speed_temp[currPacketId * BULK_NB + i] < -speed_max){
                        speed_max = -speed_temp[currPacketId * BULK_NB + i];
                    }

                    break;

                case 7:
                    //leftMotSteps[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +4] << 24) |
                    //                                            ((unsigned char) RX_buffer[ROBOT_MSG*i + i +3] << 16) |
                    //                                            ((unsigned char) RX_buffer[ROBOT_MSG*i + i +2] << 8) |
                    //                                            ((unsigned char) RX_buffer[ROBOT_MSG*i + i +1]));
                    //rightMotSteps[currPacketId * BULK_NB + i] = ((signed long) ((unsigned char) RX_buffer[ROBOT_MSG*i + i +8] << 24) |
                    //                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +7] << 16) |
                    //                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +6] << 8) |
                    //                                             ((unsigned char) RX_buffer[ROBOT_MSG*i + i +5]));
                    robXPos_fixed[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +2] << 8) |  (unsigned char) RX_buffer[ROBOT_MSG*i + i +1];
                    robYPos_fixed[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +4] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +3];
                    trigger_count[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +6] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +5];
                    //robYPos_temp1[currPacketId * BULK_NB + i] =
                    //        ((signed int) RX_buffer[ROBOT_MSG*i + i +8] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    turn[currPacketId * BULK_NB + i] =   ((unsigned int) RX_buffer[ROBOT_MSG*i + i + 7]);
                    //robYPos_temp1[currPacketId * BULK_NB + i] =| (unsigned char) RX_buffer[ROBOT_MSG*i + i +7];
                    robTheta[currPacketId * BULK_NB + i] = (
                            (((signed int) RX_buffer[ROBOT_MSG*i + i +10] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +9]) / 10);//%360;
                    robXPos[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +12] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +11];
                    robYPos[currPacketId * BULK_NB + i] =
                            ((signed int) RX_buffer[ROBOT_MSG*i + i +14] << 8) | (unsigned char) RX_buffer[ROBOT_MSG*i + i +13];
                    printf("Received pos:: %d, %d with value: %d, %d \n", i, currPacketId * BULK_NB + i, robXPos[currPacketId * BULK_NB + i],  robYPos[currPacketId * BULK_NB + i]);
                    //printf("pos:: %d, %d, %d, with value: %d, %d \n", i, robXPos_temp1[currPacketId * BULK_NB + i],  robYPos_temp1[currPacketId * BULK_NB + i]);
                    break;
            }
        }
    }


#ifdef _WIN32
    ReleaseMutex(mutexRx);
#endif
#if defined(__linux__) || defined(__APPLE__)
    pthread_mutex_unlock(&mutexRx);
#endif

}

#ifdef _WIN32
DWORD WINAPI CommThread( LPVOID lpParameter) {
    int i = 0;

    SYSTEMTIME currTimeRF;
    FILETIME currTimeRFF;
    ULONGLONG currTimeRF64;
    SYSTEMTIME txTimeRF;
    FILETIME txTimeRFF;
    ULONGLONG txTimeRF64;
    SYSTEMTIME exitTime;
    FILETIME exitTimeF;
    ULONGLONG exitTime64;

    GetSystemTime(&currTimeRF);
    SystemTimeToFileTime(&currTimeRF, &currTimeRFF);
    currTimeRF64 = (((ULONGLONG) currTimeRFF.dwHighDateTime) << 32) + currTimeRFF.dwLowDateTime;
    GetSystemTime(&txTimeRF);
    SystemTimeToFileTime(&txTimeRF, &txTimeRFF);
    txTimeRF64 = (((ULONGLONG) txTimeRFF.dwHighDateTime) << 32) + txTimeRFF.dwLowDateTime;
    GetSystemTime(&exitTime);
    SystemTimeToFileTime(&exitTime, &exitTimeF);
    exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;

    while(1) {

        if(stopTransmissionFlag==1) {
            continue;
        }

        transferData();

        currPacketId++;
        if(currPacketId>=(currNumRobots/BULK_NB)) {
            currPacketId = 0;
        }

        while(1) {
            GetSystemTime(&currTimeRF);
            SystemTimeToFileTime(&currTimeRF, &currTimeRFF);
            currTimeRF64 = (((ULONGLONG) currTimeRFF.dwHighDateTime) << 32) + currTimeRFF.dwLowDateTime;    // 100 nsec resolution

            if(((currTimeRF64-txTimeRF64)/10000 > 4)) {   // 4 ms => transfer @ 250 Hz
                GetSystemTime(&txTimeRF);
                SystemTimeToFileTime(&txTimeRF, &txTimeRFF);
                txTimeRF64 = (((ULONGLONG) txTimeRFF.dwHighDateTime) << 32) + txTimeRFF.dwLowDateTime;
                break;
            }
        }

        numOfPackets++;

        if(((currTimeRF64-exitTime64)/10000 > 5000)) { // 5 seconsd
            GetSystemTime(&exitTime);
            SystemTimeToFileTime(&exitTime, &exitTimeF);
            exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;
            setMutexThread();
            for(i=0; i<currNumRobots; i++) {
                errorPercentage[i] = numOfErrors[i]/numOfPackets*100.0;
                //printf("errorPercentage[%d] = %f\r\n", i, errorPercentage[i]);
                numOfErrors[i] = 0;
            }
            freeMutexThread();
            numOfPackets = 0;
        }
    }

    return 0;
}
#endif

#if defined(__linux__) || defined(__APPLE__)
void *CommThread(void *arg) {
	int i = 0;
	struct timeval currTimeRF, txTimeRF, exitTime;

    gettimeofday(&currTimeRF, NULL);
	gettimeofday(&txTimeRF, NULL);
	gettimeofday(&exitTime, NULL);


    while(1) {

        if(stopTransmissionFlag==1) {
            continue;
        }

        transferData();

        currPacketId++;
        if(currPacketId>=((int) ceil(currNumRobots/4.0))) {
            currPacketId = 0;

            if (type == 1){
			    currRobotId++;
			}
			if (type > 1){
                currRobotId_L++;
            }
        }

        //printf("currRobotId: %d, %d \n", currRobotId, (ROBOTS/3));
        if(currRobotId>(ROBOTS/3)) {
            currRobotId = 0;
        }

        if(currRobotId_L>(ROBOTS/6)) {
            currRobotId_L = 0;
            //nb_init_rounds++;
        }

        while(1) {
            gettimeofday(&currTimeRF, NULL);

            if((((currTimeRF.tv_sec * 1000000 + currTimeRF.tv_usec)-(txTimeRF.tv_sec * 1000000 + txTimeRF.tv_usec)) > 4000)) {   // 4 ms => transfer @ 250 Hz
                gettimeofday(&txTimeRF, NULL);
                break;
            }
        }

        numOfPackets++;

        if(((currTimeRF.tv_sec * 1000000 + currTimeRF.tv_usec)-(exitTime.tv_sec * 1000000 + exitTime.tv_usec) > 5000000)) { // 5 seconds
            gettimeofday(&exitTime, NULL);
            setMutexThread();
            for(i=0; i<currNumRobots; i++) {
                errorPercentage[i] = numOfErrors[i]/numOfPackets*100.0;
                //printf("errorPercentage[%d] = %f\r\n", i, errorPercentage[i]);
                numOfErrors[i] = 0;
            }
            freeMutexThread();
            numOfPackets = 0;
        }
    }

}
#endif


