#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#define GREEN1 GPIO_PIN_1     //PD_1
#define RED1 GPIO_PIN_2      //PD_2
#define LDR1 GPIO_PIN_7      //PD_0 A7

#define GREEN2 GPIO_PIN_1     //PE_1
#define RED2 GPIO_PIN_2      //PE_2
#define LDR2 GPIO_PIN_1      //PE_3 F1

#define GREEN3 GPIO_PIN_5    //PA_5
#define RED3 GPIO_PIN_6      //PA_6
#define LDR3 GPIO_PIN_4      //PB_4

#define GREEN4 GPIO_PIN_4    //PE_4
#define RED4 GPIO_PIN_3      //PE_5 A3
#define LDR4 GPIO_PIN_2                        //PB_5 A2

//PROTOTIPOS------------------------------
void UART1_Init(void);
void UART1_WriteChar(char data);
//----------------------------------------
//Variables globales----------------------
int p1, p2, p3, p4;
unsigned char PSTATUS;
//----------------------------------------

int main(void){
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                            //TURN PORTF ON
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Inicializar UART1
    UART1_Init();

    //LED PINS CONFIG
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, RED1|GREEN1);        //LEDS S1
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, RED2|GREEN2);        //LEDS S2
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, RED3|GREEN3);        //LEDS S3
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GREEN4);        //LEDS S4
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, RED4);

    //LDR CONFIG
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, LDR1);                //LDR S1
    //GPIOPadConfigSet(GPIO_PORTD_BASE, LDR1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, LDR2);                //LDR S2
    //GPIOPadConfigSet(GPIO_PORTE_BASE, LDR2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, LDR3);                //LDR S3
    //GPIOPadConfigSet(GPIO_PORTB_BASE, LDR3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, LDR4);                //LDR S4
    //GPIOPadConfigSet(GPIO_PORTB_BASE, LDR4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);


while (1) {

    if(GPIOPinRead(GPIO_PORTA_BASE, LDR1)==0){          //LECTURA S1
        GPIOPinWrite(GPIO_PORTD_BASE, GREEN1, GREEN1);
        GPIOPinWrite(GPIO_PORTD_BASE, RED1, 0);
        p1=1;
    }else{
        GPIOPinWrite(GPIO_PORTD_BASE, RED1, RED1);
        GPIOPinWrite(GPIO_PORTD_BASE, GREEN1, 0);
        p1=0;
    }

    if(GPIOPinRead(GPIO_PORTF_BASE, LDR2)==0){          //LECTURA S2
        GPIOPinWrite(GPIO_PORTE_BASE, GREEN2, GREEN2);
        GPIOPinWrite(GPIO_PORTE_BASE, RED2, 0);
        p2=1;
    }else{
        GPIOPinWrite(GPIO_PORTE_BASE, RED2, RED2);
        GPIOPinWrite(GPIO_PORTE_BASE, GREEN2, 0);
        p2=0;
    }

    if(GPIOPinRead(GPIO_PORTB_BASE, LDR3)==0){          //LECTURA S3
        GPIOPinWrite(GPIO_PORTA_BASE, GREEN3, 0xFF);
        GPIOPinWrite(GPIO_PORTA_BASE, RED3, 0);
        p3=1;
    }else{
        GPIOPinWrite(GPIO_PORTA_BASE, RED3, RED3);
        GPIOPinWrite(GPIO_PORTA_BASE, GREEN3, 0);
        p3=0;
    }

    if(GPIOPinRead(GPIO_PORTA_BASE, LDR4)==0){          //LECTURA S4
        GPIOPinWrite(GPIO_PORTE_BASE, GREEN4, GREEN4);
        GPIOPinWrite(GPIO_PORTA_BASE, RED4, 0);
        p4=1;
    }else{
        GPIOPinWrite(GPIO_PORTA_BASE, RED4, RED4);
        GPIOPinWrite(GPIO_PORTE_BASE, GREEN4, 0);
        p4=0;
    }


    PSTATUS = (p1) | (p2 << 1) | (p3 << 2) | (p4 << 3);
    UART1_WriteChar(PSTATUS);
    } //FIN WHILE
}

void UART1_Init(void) {
    // Habilitar el módulo UART1 y los pines asociados
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configurar los pines PB0 y PB1 para la función UART1
    GPIOPinConfigure(GPIO_PB0_U1RX); //Definir pinB0 como RX
    GPIOPinConfigure(GPIO_PB1_U1TX); //Definir pinB1 como TX
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configurar la velocidad de transmisión (en este caso, 115200 bps)
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Habilitar el módulo UART1
    UARTEnable(UART1_BASE);
}

void UART1_WriteChar(char data) {
    // Esperar hasta que el buffer de transmisión esté vacío
    while(UARTBusy(UART1_BASE)){}

    // Enviar el carácter
    UARTCharPut(UART1_BASE, data);
}
