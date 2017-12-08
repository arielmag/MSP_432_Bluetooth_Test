/*
 * This program is used to test bluetooth communication with the MSP 432. A
 * terminal emulator is used on a phone paired with a bluetooth module to send
 * commands over bluetooth. The MSP 432 then receives the commands and performs
 * an action if the command is recognized.
 *
 * Recognized commands:
 * STA : Transmit status of system with date and time, arm/disarmed status, and lock/unlocked status
 * LCK : Lock/unlock the door and transmit the outcome
 * ARM : Arm/disarm the alarm and transmit the outcome
 *
 * Connections:
 * P3.2/UCA2RX -> Bluetooth Module TX
 * P3.3/UCA2TX -> Bluetooth Module RX
 * 5V -> Bluetooth Module 5V Pin
 * GND ->  Bluetooth Module GND Pin
 *
 * Based off of the MSP432 DriverLib program: uart_pc_echo_12mhz_brclk
 *
 * Ariel Magyar
 */


/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 100

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 *
 * Configured for a 9600 baud rate (specified by bluetooth module) and 3MHz
 * SMCLK speed (default speed)
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        19,                                       // BRDIV
        8,                                        // UCxBRF
        0,                                        // UCxBRS
        EUSCI_A_UART_NO_PARITY,                   // No Parity
        EUSCI_A_UART_LSB_FIRST,                   // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                // One stop bit
        EUSCI_A_UART_MODE,                        // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

void Init48MHz();

volatile uint8_t receivedData = 0;

/*
 * Flag to indicate if new unprocessed data has been sent by phone to
 * prevent responding to same command twice.
 */
volatile uint8_t received_data = 0;

/*
 * Buffer used to organize received data
 */
volatile uint8_t buffer[BUFFER_SIZE];

/*
 * Current place in buffer
 */
volatile uint16_t buffer_index;

/*
 * Message to send out
 */
char print_all[70];

/*
 * Used to store last 3 bytes of received data
 */
uint8_t read_data[3];

/*
 * Flag to indicate if new unprocessed data has been sent by phone to
 * prevent responding to same command twice.
 */
int data_used = 0;

static volatile uint32_t aclk, mclk, smclk, hsmclk, bclk;

int main(void)
{
    // Initialize MCLK to 48MHz
    Init48MHz();

    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Use LED on P1.0 to indicate when data is received. Switch off at start.
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    //MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();   

    // Check clock rates - SMCLK should be 3MHz
    aclk = CS_getACLK();
    mclk = CS_getMCLK();
    smclk = CS_getSMCLK();
    hsmclk = CS_getHSMCLK();
    bclk = CS_getBCLK();

    buffer_index = 0;

    data_used = 0;

    // Set initial status for armed and locked
    int isArmed = 0;
    int isLocked = 1;

    while(1)
    {

        // Set date to Sunday, November 11, 2011 at 11:11:11
        unsigned char rtc_registers[15]={0x11, 0x11, 0x11, 0x01, 0x11, 0x11, 0x11, 0};


        // Check if data was sent
        if(buffer_index > 2 && data_used == 0){

            int i, j=2;

            // Clear print all
            for(i=0;i<58;i++){
               print_all[i] = 0;
            }

            // Check last 3 characters transmitted
            for(i=0; i<3; i++){
                read_data[j] = buffer[buffer_index-1-i];
                j--;
            }


            // Send status if received STA
            if(read_data[0] == 'S' && read_data[1] == 'T' && read_data[2] == 'A'){

                char print_date[27];
                sprintf(print_date,"\nDATE: %02x/%02x/%02x %02x:%02x:%02x\n", rtc_registers[5], rtc_registers[4], rtc_registers[6], rtc_registers[2], rtc_registers[1], rtc_registers[0]);

                strcat(print_all, print_date);

                if(isArmed){
                    char print_arm_disarm[] = "ALARM: ARMED\n";
                    strcat(print_all, print_arm_disarm);
                }else{
                    char print_arm_disarm[] = "ALARM: DISARMED\n";
                    strcat(print_all, print_arm_disarm);
                }

                if(isLocked){
                    char print_lock_unlock[] = "LOCK: LOCKED\n\n";
                    strcat(print_all, print_lock_unlock);
                }else{
                    char print_lock_unlock[] = "LOCK: UNLOCKED\n\n";
                    strcat(print_all, print_lock_unlock);
                }

                // Transmit data stored in print_all array
                for(i=0;i<70;i++){
                    MAP_UART_transmitData(EUSCI_A2_BASE, print_all[i]);
                }

                data_used = 1;
            }

            // Lock/unlock if received LCK
            if(read_data[0] == 'L' && read_data[1] == 'C' && read_data[2] == 'K'){
                if(isLocked){
                    char print[] = "\nDOOR UNLOCKED\n\n";
                    strcat(print_all, print);
                }else{
                    char print[] = "\nDOOR LOCKED\n\n";
                    strcat(print_all, print);
                }

                // Transmit data stored in print_all array
                for(i=0;i<70;i++){
                    MAP_UART_transmitData(EUSCI_A2_BASE, print_all[i]);
                }

                isLocked = !isLocked;

                data_used = 1;
            }

            // Arm/disarm if received ARM
            if(read_data[0] == 'A' && read_data[1] == 'R' && read_data[2] == 'M'){
                if(isArmed){
                    char print[] = "\nALARM DISARMED\n\n";
                    strcat(print_all, print);
                }else{
                    char print[] = "\nALARM ARMED\n\n";
                    strcat(print_all, print);
                }

                // Transmit data stored in print_all array
                for(i=0;i<70;i++){
                    MAP_UART_transmitData(EUSCI_A2_BASE, print_all[i]);
                }

                isArmed = !isArmed;

                data_used = 1;
            }


        }
    }
}

/* EUSCI A2 UART ISR - Echoes data back to PC host */
// Use UART2 because using P3.2 and P3.3 for RX and TX
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        // Toggle LED to show data was received
        P1OUT ^= BIT0;

        received_data = MAP_UART_receiveData(EUSCI_A2_BASE);

        // Restart buffer if reached maximum
        if(buffer_index>BUFFER_SIZE-1) {
            buffer_index = 0;
        }

        // Store data in buffer.
        // Not using a buffer causes problems with data being received too quickly.
        buffer[buffer_index++]= received_data;

        MAP_UART_transmitData(EUSCI_A2_BASE, received_data);  // send byte out UART2 port

        // Indicate new data received as being unused
        data_used = 0;
    }
}

/*
 * Set the clock module to use the external 48 MHz crystal.
 * Credit: EGR 326 Lecture Slide
 */
void Init48MHz(){
    /* Configuring pins for peripheral/crystal usage */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(32000,48000000); // enables getMCLK, getSMCLK to know externally set frequencies
    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     *
    * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false); // false means that there are no timeouts set,will return when stable

    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}
