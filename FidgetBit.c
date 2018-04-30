#include <msp430.h>
#include <stdio.h>
#include <stdbool.h>




/**
 * main.c
 *
 * Includes implementation with HeartRate calulation,
 * Pedometer step counting, and Persistence of Vision
 * Display.
 *
 */
enum Mode{heartbeat, steps};
enum Mode mode = heartbeat;

int numbers [10][5] = {{0, 31, 17, 31, 0}, {0, 9, 31, 1, 0}, {0, 23, 21, 29, 0}, {0, 21, 21, 31, 0}, {0, 28, 4, 31, 0}, {0, 19, 21, 23, 0},
                       {0, 31, 21, 23, 0}, {0, 24, 16, 31, 0}, {0, 31, 21, 31, 0}, {0, 28, 20, 31, 0}};

//int bpm[3][5] = {{31, 21, 21, 10, 0}, {31, 20, 20, 28, 0}, {31, 8, 4, 8, 31}};
//int steps[5][5] = {{8, 21, 21, 2, 0}, {16, 16, 31, 16, 16}, {31, 21, 21, 17, 0}, {31, 20, 20, 8, 0}, {8, 21, 21, 2, 0}};
int heart[7] = {8, 28, 30, 15, 30, 28, 8};
int shoe[9] = {3, 7, 7, 31, 31, 0, 21, 20, 16};
int arrow[9] = {17, 10, 4, 17, 10, 4, 17, 10, 4};

static const int LED1 = 0x10;
static const int LED2 = 0x20;
static const int LED3 = 0x01;
static const int LED4 = 0x02;
static const int LED5 = 0x04;

//  port 1 - 4(0x10)LED1 5(0x20)LED2
//  port 2 - 0(0x01)LED3 1(0x02)LED4 2(0x04)LED5


//Function Prototypes
void setup();//94
void adc_Setup();//121
void adc_Sample();//254
void displayLine(int line);//348
int getNumArrayIndex(char c);//378
void displayNum(char c);//403
void displayChar(char* c);//414
void displayString(char* s);// 424
void displayVal(int value);//433
void getBPM();//178
void msDelay (unsigned int msTime);//170
void calibrate();//132
void getSteps();//213
int squareRoot(int n);//382
void BPM(void);
void printCharacters();
void display(int letter);
//TIMER0 INTERRUPT 268
//BUTTON INTERRUPT 308


// Variables for heartbeat
int adc[8] = {0};           //Sets up an array of 1 integer for adc readings
int timercount = 0;         //Counter for number of timed interrupts
int heartbeatcount = 0;     //Counter for number of heartbeats
int upperThreshold = 400;   //Upper threshold for heartbeat reading
int lowerThreshold = 400;   //Lower threshold for heartbeat reading
int peak = 530;
bool readBeat = false;      //flag that determines if we want to read heartbeat or not


//variables for step counter
int xval[15] = {0};
int yval[15] = {0};
int zval[15] = {0};
int xavg = 443;
int yavg = 493;
int zavg = 833;
int stepThreshold = 75;
int lowerStepThreshold = 65;
int stepFlag = 0;
int stepTotal = 0;
int getStepsInitFlag = 1;
bool getStepsFlag = false;

//Other important variables
int displayValue = 0;       //Value to be displayed on leds
int timerMode = 0;
int prevValue = 0;
int testFlag = 1;
int testFlag2 = 1;

int CCR0Refill = 500;

// Global variable for timer to print char
char charToPrint;

char printChar;
int lineIndex = 0;
int charIndex = 0;
int waitFlag = 0;
int heartPrintFlag = 0;
int stepPrintFlag = 0;
int charLen = 0;
int iconCompleteFlag = 0;
int iconFlag = 0;


int main(void) {
    WDTCTL = WDTPW + WDTHOLD;
      if(CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF) {
          while(1);
      }
      setup();
      while(1){
          if (getStepsFlag){
              getSteps();
          }
      }
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
__bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

void setup() {
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
    P1DIR &= 0x00;          //Reset dir and output
    P1OUT &= 0x00;
    P2DIR &= 0x00;
    P2OUT &= 0x00;
    adc_Setup();            //set up adc
    P1SEL = 0;
    P1DIR |= BIT4 + BIT5;               //set leds as outputs and everything else as input
    P2SEL = 0;
    P2DIR |= BIT0 + BIT1 + BIT2;
    P1REN |= BIT3;                   // Enable internal pull-up/down resistors
    P1OUT |= BIT3;                   //Select pull-up mode for P1.3
    P1IE |= BIT3;                       // P1.3 interrupt enabled
    P1IES |= BIT3;                     // P1.3 Hi/lo edge
    P1IFG &= ~BIT3;                  // P1.3 IFG cleared

    P1REN |= BIT2;                   // Enable internal pull-up/down resistors
    P1OUT |= BIT2;                   //Select pull-up mode for P1.3
    P1IE |= BIT2;                       // P1.3 interrupt enabled
    P1IES |= BIT2;                     // P1.3 Hi/lo edge
    P1IFG &= ~BIT2;                  // P1.3 IFG cleared

    // testing ISR time
    P2DIR |= BIT5;
    P2OUT &= ~BIT5;

    _BIS_SR(GIE);          // Enter LPM0 w/ interrupt
}

// ADC set-up function
void adc_Setup()
{
    ADC10CTL1 = CONSEQ_1 + INCH_7;                      // Go through multiple channels, A7 to A0
    ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;   // Sample & Hold Time + ADC10 ON + Interrupt Enable
    ADC10DTC1 = 0x08;                                  // 1 conversion
    ADC10AE0 |= 0xC3;                                   // option select for heartbeat, x,y,z
    CCTL0 = CCIE;                             // CCR0 interrupt disabled
    TACTL = TASSEL_2 + MC_1 + ID_3;           // SMCLK/8, upmode
//    CCR0 = 32000;                           // 0.5 Hz lower = fastur
    CCR0 = CCR0Refill;                           // ?? Hz
}

void calibrate()
{

    int xsum = 0;
    int ysum = 0;
    int zsum = 0;
    int i = 0;
    for (i = 0; i < 15; i++)
    {
        if(i == 2) {
            P2OUT &= ~LED5;
        }
        if(i == 5) {
            P2OUT &= ~LED4;
        }
        if(i == 8) {
            P2OUT &= ~LED3;
        }
        if(i == 11) {
            P1OUT &= ~LED2;
        }
        if(i == 14) {
            P1OUT &= ~LED1;
        }
        adc_Sample();
        xval[i] = adc[1];
        xsum = xval[i] + xsum;
        yval[i] = adc[0];
        ysum = yval[i] + ysum;
        zval[i] = adc[6];
        zsum = zval[i] + zsum;
    }

    xavg = xsum/15.0;
    yavg = ysum/15.0;
    zavg = zsum/15.0;
}

//fixed timer delay function
void msDelay (unsigned int msTime) {
    long counter;
    for (counter = 0; counter <= msTime; counter++) {
        _delay_cycles(16000000/16000);
    }
}

//Getting heartbeat
void getBPM()
{
    bool heartbeatdetected = false;
    bool heartbeatFlag = false;                //Boolean to see if a heartbeat value has surpassed the upper threshold or gone under the lower threshold
    readBeat = true;
    CCTL0 = CCIE;                             // CCR0 interrupt enabled

    while(readBeat)
    {
        adc_Sample();      // Function call for adc_samp
        if(adc[0] > peak) {         //check if peak value has been hit yet, if not set peak to current highest value
            peak = adc[0];
        }
        if((int)(peak*0.9) >= adc[0] && heartbeatFlag) {         //if adc value is less than peak, peak has been hit
            if(peak > (int)(upperThreshold * 1.1) || peak < (int)(upperThreshold * 0.9)) {
                upperThreshold = (int)(peak * 0.90);                 //set upper threshold to 92% of peak
                lowerThreshold = (int)(upperThreshold * 0.80);       //set lower threshold to 88% of upper threshold
            }
        }

        if(adc[0] > upperThreshold && !heartbeatFlag) {     //Check if heartbeat adc value is higher than upper threshold
            heartbeatcount++;
            heartbeatFlag = true;       //Prevent other heartbeats to be recorded until adc goes under lower threshold
        }
        if(adc[0] <= lowerThreshold && heartbeatFlag) {     //Check if heartbeat adc value is lower than lower threshold
            heartbeatFlag = false;      //Allow heartbeats to be recorded again
            peak = 0;
        }
    }
    heartbeatcount = 0;
    timercount = 0;
//    CCTL0 = ~CCIE;                             // CCR0 interrupt disabled
}

//Getting steps
void getSteps() {
    int acc=0;
    int totvect = 0;
    int totave = 0;
    int xaccl = 0;
    int yaccl = 0;
    int zaccl = 0;
    int i = 0;
    int xsquare, ysquare, zsquare, squareTot = 0;


//    while(1) {
        adc_Sample();
        xaccl = adc[1];
        yaccl = adc[0];
        zaccl = adc[6];

        xsquare = (xaccl - xavg)* (xaccl - xavg);
        ysquare = (yaccl - yavg)*(yaccl - yavg);
        zsquare = (zaccl - zavg)*(zaccl - zavg);
        squareTot =  xsquare + ysquare + zsquare;


        totave = squareRoot(squareTot);
        //cal steps

        if (totave > stepThreshold && stepFlag == 0) {
           stepTotal = stepTotal + 1;
           stepFlag = 1;
        }
//        else if (totave > stepThreshold && stepFlag == 1) {
//
//        }
        if (totave < lowerStepThreshold  && stepFlag == 1) {
          stepFlag = 0;
        }
        if(mode == steps) {
            displayValue = stepTotal;
          //  char str[5];
          //  sprintf(str, "%d", displayValue);
          //  chartoprint = displayValue[0];
        }
//    }
}


// ADC sample conversion function
void adc_Sample()
{
    __bic_SR_register(GIE);// Low Power Mode 0, ADC10_ISR
    ADC10CTL0 &= ~ENC;              // Disable Conversion
    while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
    ADC10SA = (int)adc;             // Transfers data to next array (DTC auto increments address)
    ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
//    __bis_SR_register(CPUOFF + GIE);// Low Power Mode 0, ADC10_ISR
    __bis_SR_register(GIE);// Low Power Mode 0, ADC10_ISR

}

void printCharacters(){
    // need to set this to be done only after 3000
 int charLen;
    if(waitFlag > 0){
        waitFlag--;
        P1OUT &= ~(LED1 + LED2);
        P2OUT &= ~(LED3 + LED4 + LED5);
        return;
    }
    char str[5];
    if(mode == steps && iconFlag == 1){
     charLen = 9;
     display(arrow[lineIndex]);
     if(lineIndex == charLen - 1){
         iconCompleteFlag = 1;
     }
    }
    else if(mode == heartbeat && iconFlag == 1){
     charLen = 7;
     display(heart[lineIndex]);
     if(lineIndex == charLen - 1) {
         iconCompleteFlag = 1;
     }
    }
    else{
     charLen = 5;
    }
    sprintf(str, "%d", displayValue);
    if(charIndex < strlen(str)){
     charToPrint = str[charIndex];
    }

    if(!iconFlag) {
     int num = getNumArrayIndex(charToPrint);
     display(numbers[num][lineIndex]);
    }

    lineIndex += 1;

    lineIndex = lineIndex % charLen;
     if(charIndex >= strlen(str) && iconCompleteFlag == 1){             // do we need the charIndex >= strlen(str) ??
         charIndex = 0;
         lineIndex = 0;
         iconFlag = 0;
         iconCompleteFlag = 0;
         if (mode == heartbeat){
             waitFlag = 3;
         }
         else{
             waitFlag = 5;
         }
     }
     else if(lineIndex == 0){
        charIndex++;
        charToPrint = str[charIndex];
        waitFlag = 1;

    }
    if(charIndex >= strlen(str)) {
         iconFlag = 1;
    }


}
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void){
//    int oldbuttonstate = 0;
//
//     int buttonstate;
//
//
//     buttonstate = !(P1IN & BIT3);
//
//     if( (buttonstate != oldbuttonstate)) {
//         CCTL0 = ~CCIE;
//     }
//    if(P1IN&BIT3 != 0) {
//        CCTL0 = ~CCIE;
//    }

    //testing ISR time begin
    P2OUT ^= BIT5;
    P2OUT ^= BIT5;
    P2OUT |= BIT5;

    testFlag = P1IN&BIT3;
    testFlag2 = P1IN&BIT2;
        if(testFlag != 0) {
//            msDelay(1);
            testFlag = P1IN&BIT3;
            if(testFlag != 0){
                CCTL0 = ~CCIE;
                testFlag = 0;

                if(mode == heartbeat) {
                    P1OUT &= 0x00;
                    P2OUT &= ~(BIT0 + BIT1 + BIT2);
                    msDelay(1000);
                    P1OUT |= LED1;
                    msDelay(1000);
                    P1OUT |= LED2;
                    msDelay(1000);
                    P2OUT |= LED3;
                    msDelay(1000);
                    P2OUT |= LED4;
                    msDelay(1000);
                    P2OUT |= LED5;
                    P1IFG &= ~BIT3;                     // P1.3 IFG cleared
//                    CCR0 = 32000;
                    CCR0 = CCR0Refill;
                    timerMode = 1;
                    getBPM();
                    testFlag = 0;
                    testFlag2 = 0;
                }
                if(mode == steps) {
                    P1OUT &= ~(LED1 + LED2);
                    P2OUT &= ~(LED3 + LED4 + LED5);
                    msDelay(400);
                    P1OUT |= LED1;
                    msDelay(400);
                    P1OUT |= LED2;
                    msDelay(400);
                    P2OUT |= LED3;
                    msDelay(400);
                    P2OUT |= LED4;
                    msDelay(400);
                    P2OUT |= LED5;
                    P1IFG &= ~BIT3;
//                    calibrate();
                    getStepsFlag = !getStepsFlag;
                    testFlag = 0;
                    testFlag2 = 0;
                    CCTL0 = CCIE;
                }
            }
        }

        else if(testFlag2 != 0) {
//            msDelay(1);
            testFlag2 = P1IN&BIT2;
            if(testFlag2 != 0){

                if(mode == heartbeat) {
                    mode = steps;
                }
                else {
                    mode = heartbeat;
                }
                P1OUT &= 0x00;
                P2OUT &= ~(BIT0 + BIT1 + BIT2);
                msDelay(100);
                P1OUT |= LED1;
                msDelay(100);
                P1OUT |= LED2;
                msDelay(100);
                P2OUT |= LED3;
                msDelay(100);
                P2OUT |= LED4;
                msDelay(100);
                P2OUT |= LED5;
                P1IFG &= ~BIT2;                     // P1.3 IFG cleared
                testFlag = 0;
                testFlag2 = 0;
                int temp = displayValue;
                displayValue = prevValue;
                char str[5];
                sprintf(str, "%d", displayValue);
                charToPrint = str[0];
                prevValue = temp;
            }
        }
    if(timerMode == 1){
        if(readBeat)  {
            timercount++;                           //Increment timer interrupt count ever half second, 20 is 10 seconds
            if(timercount == 500) {                   //Turn off leds accordingly
                P2OUT &= ~LED5;
            }
            if(timercount == 1000) {
                P2OUT &= ~LED4;
            }
            if(timercount == 1500) {
                P2OUT &= ~LED3;
            }
            if(timercount == 2000) {
                P1OUT &= ~LED2;
            }
            if(timercount == 2500) {
                P1OUT &= ~LED1;
            }

            if(timercount >= 2500) {                  //If timer count is equal to or over 40, display value
                if(heartbeatcount >= 9) {
                    displayValue = heartbeatcount * 6;  //Set display value to be 6x the heartbeat count
                    readBeat = false;
//                    CCR0 = 32000;
                    CCR0 = CCR0Refill;
                    timerMode = 0;
                }
                heartbeatcount = 0;                 //Reset counters
                timercount = 0;
                if(readBeat) {
                    P1OUT |= LED1 + LED2;
                    msDelay(1000);
                    P2OUT |= LED3 + LED4 + LED5;
                    msDelay(1000);
                }
                //if readbeat is still true, set all 5 leds on again
            }
        }
    }
    if(timerMode == 0) {
            printCharacters();
    }

    // testing ISR time end
//    P1OUT &= ~BIT4;

}



void display(int letter){
    displayLine(letter);

}

 void BPM (void)
{

}

//#pragma vector=PORT1_VECTOR
//__interrupt void Port_1 (void)
//{
//
////    if(P1IN&BIT3 == 0) {
//        if(mode == heartbeat) {
//            testFlag = 1;
//            P1OUT &= 0x00;
//            P2OUT &= 0x00;
//            msDelay(1000);
//            P1OUT |= LED1;
//            msDelay(1000);
//            P1OUT |= LED2;
//            msDelay(1000);
//            P2OUT |= LED3;
//            msDelay(1000);
//            P2OUT |= LED4;
//            msDelay(1000);
//            P2OUT |= LED5;
//            P1IFG &= ~BIT3;                     // P1.3 IFG cleared
//            CCR0 = 32000;
//            timerMode = 1;
//            getBPM();
//            testFlag = 1;
//        }
//        if(mode == steps) {
//            P1OUT &= 0x00;
//            P2OUT &= 0x00;
//            msDelay(400);
//            P1OUT |= LED1;
//            msDelay(400);
//            P1OUT |= LED2;
//            msDelay(400);
//            P2OUT |= LED3;
//            msDelay(400);
//            P2OUT |= LED4;
//            msDelay(400);
//            P2OUT |= LED5;
//            P1IFG &= ~BIT3;
//            calibrate();
//        }
////    }
////    else {
////        mode++;
////        P1OUT &= 0x00;
////        P2OUT &= 0x00;
////        msDelay(100);
////        P1OUT |= LED1;
////        msDelay(100);
////        P1OUT |= LED2;
////        msDelay(100);
////        P2OUT |= LED3;
////        msDelay(100);
////        P2OUT |= LED4;
////        msDelay(100);
////        P2OUT |= LED5;
////        P1IFG &= ~BIT2;                     // P1.3 IFG cleared
////        int temp = displayValue;
////        displayValue = prevValue;
////        prevValue = temp;
////    }
//}

void displayLine(int line) {
    int finalPort1 = 0;
    int finalPort2 = 0;
    int myline = 0;
    myline = line;

    if(myline >= 16) {
        P1OUT |= LED1;
        myline -= 16;
    }
    else{
        P1OUT &= ~LED1;
    }
    if(myline >= 8) {
        P1OUT |= LED2;
        myline -= 8;
    }
    else{
        P1OUT &= ~LED2;
    }
    if(myline >= 4) {
        P2OUT |= LED3;
        myline -= 4;
    }
    else{
        P2OUT &= ~LED3;
    }
    if(myline >= 2) {
        P2OUT |= LED4;
        myline -= 2;
    }
    else{
        P2OUT &= ~LED4;
    }
    if(myline >= 1) {
        P2OUT |= LED5;
        myline -= 1;
    }
    else{
        P2OUT &= ~LED5;
    }
//    P1OUT = finalPort1;
//    P2OUT = finalPort2;
}

int getNumArrayIndex(char c) {
    return c - 48;
}

int squareRoot(int n) {
    int counter = 1;
    if(n == 0) {
        return 0;
    }
    while(1) {
        if((counter * counter) == n) {
            return counter;
        }
        else if(n < (counter * counter)) {
            int lowerTotal = (counter - 1) * (counter - 1);
            int higherTotal = counter * counter;
            if(n - lowerTotal < higherTotal - n) {
                return counter - 1;
            }
            return counter;
        }
        counter++;
    }
}

void displayNum(char c) {
    int index = getNumArrayIndex(c);
    int i = 0;
    for(i = 0; i < 5; i++) {
        displayLine(numbers[index][i]);
        _delay_cycles(3000);
    }
    P1OUT &= ~(LED1 + LED2);
    P2OUT &= ~(LED3 + LED4 + LED5);
}

//void displayChar(char* c) {
//    int i = 0;
//    for(i = 0; i < 10; i = i+2) {
//        displayLine(c[i]);
//        _delay_cycles(2000);
//    }
//    P1OUT = 0;
//    P2OUT &= ~(BIT0 + BIT1 + BIT2);
//}
//
//void displayString(char* s) {
//    int i = 0;
//     for (i = 0; i<=strlen(s); i++) {
//       displayChar(s[i]);
//       _delay_cycles(2000);
//     }
//     _delay_cycles(4000);
//}

void displayVal(int value) {
    int i = 0;
    char str[5];
    sprintf(str, "%d", value);
    for(i = 0; i < strlen(str); i++) {
        displayNum(str[i]);
        _delay_cycles(3000);
    }
     _delay_cycles(5000);
//
//    for(i = 0; i < 3; i++) {
//        displayChar(bpm[i]);
//        _delay_cycles(3000);
//    }

     if(mode == heartbeat) {
         for(i = 0; i < 7; i++) {                   //heart
            displayLine(heart[i]);
            _delay_cycles(3000);
         }
     }
     if(mode == steps) {
        for(i = 0; i < 9; i++){                   //shoe
            displayLine(arrow[i]);
            _delay_cycles(5000);
        }
     }
     P1OUT &= ~(LED1 + LED2);
     P2OUT &= ~(LED3 + LED4 + LED5);
    _delay_cycles(30000);
}


int i = 0;
int j = 0;
int k = 0;
int counter = 0;
//codys
//void displayNum(char c){
//    int index = getNumArrayIndex(c);
//    displayLine(numbers[index][j]);
//    j++;
//    if (j >= 5){
//        i++;
//        j = 0;
//        P1OUT &= ~(BIT4 + BIT5);
//        P2OUT &= ~(BIT0 + BIT1 + BIT2);
//    }
//}
//codys displayval
//void displayVal (int value){
//    int tempCounter1 = 0;
//    int tempCounter2 = 0;
//
//    counter ++;
//    char str[5];
//    sprintf(str, "%d", value);
//    if (i < strlen(str)){
//        if (((counter - 1) % 3) == 0){
//            displayNum(str[i]);
//        }
//    }
//    tempCounter1 = 1 + 5 + (strlen(str) * 18);         // initial 1 + 5 between chars and heart + delay after final displayNum + time it takes to complete above code
//    if (counter >= tempCounter1){
//        if (mode == heartbeat){
//            if (k < 7){
//                if ((counter - tempCounter1) % 3 == 0){
//                    displayLine(heart[k]);
//                    k++;
//                    tempCounter2 = counter;
//                }
//            }
//            else if (counter >= (tempCounter2 + 33)){
//                counter = 0;
//                tempCounter1 = 0;
//                tempCounter2 = 0;
//                i = 0;
//                j = 0;
//                k = 0;
//                P1OUT &= ~(BIT4 + BIT5);
//                P2OUT &= ~(BIT0 + BIT1 + BIT2);
//            }
//            else{
//                P1OUT &= ~(BIT4 + BIT5);
//                P2OUT &= ~(BIT0 + BIT1 + BIT2);
//            }
//        }
//        else if (mode == steps){
//            if (k < 9){
//                if ((counter - tempCounter1) % 5 == 0){
//                    displayLine(heart[k]);
//                    k++;
//                    tempCounter2 = counter;
//                }
//            }
//            else if (counter >= (tempCounter2 + 35)){
//                counter = 0;
//                tempCounter1 = 0;
//                tempCounter2 = 0;
//                i = 0;
//                j = 0;
//                k = 0;
//                P1OUT &= ~(BIT4 + BIT5);
//                P2OUT &= ~(BIT0 + BIT1 + BIT2);
//            }
//            else{
//                P1OUT &= ~(BIT4 + BIT5);
//                P2OUT &= ~(BIT0 + BIT1 + BIT2);
//            }
//        }
//    }
//}
