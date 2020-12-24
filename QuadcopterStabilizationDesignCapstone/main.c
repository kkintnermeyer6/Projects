/* Quadcopter Capstone Team
 * Authors:
 * Michael Janicki
 * Karl Kintner-Meyer
 * Nick Kunst
 * Hallvard Ng
 *
 *
 *
 * This code allows the user to control a quadcopter.
 * The code first initializes the ESCs for the motors.
 * Then, the user is prompted "Select Motor."
 * By specifying different cases when prompted, the
 * user can decide whether of not to manually select
 * the speeds of the motors individually or together,
 * or select automatic mode.
 * Modes 0-4 are manual.
 * Mode 5 turns on all controllers and goes to the
 * reference positions written in the code.
 * Mode 6 changes the Roll reference.
 * Mode 7 allows the user to specify the roll
 * reference manually.
 * Mode 9 ends the program.
 *
 *This program also saves buffers containing data on
 *the state estimation, controller outputs,
 *motor speeds, etc. to matlab.
 *
 * Sensor communication is done using I2C
 * communication protocols and the ESC signals
 * are output using PWM signals.
 * */

/* includes */
#include <stdio.h>
#include "MyRio.h"
#include "me477.h"
#include <string.h>
#include "TimerIRQ.h"
#include <pthread.h>
#include "matlabfiles.h"
#include "math.h"
#include "I2C.h"
#include <math.h>
#include <IRQConfigure.h>
#include <unistd.h>
#include "PWM.h"

/*
 * Declare the myRIO NiFpga_Session so that it can be used by any function in
 * this file. The variable is actually defined in myRIO.c.
 */
extern NiFpga_Session myrio_session;

// Biquad declaration for controllers/filters
struct biquad  {
	double  b0; double  b1; double  b2; // numerator
	double  a0; double  a1; double  a2; //denominator
	double  x0; double  x1; double  x2; //input
	double  y1; double  y2; };			//output

// Controllers and filters
#include "PhiController.h"
#include "ThetaController.h"
#include "PsiController.h"
#include "ZController.h"
#include "LowPassFilterPhi.h"
#include "HighPassFilterPhi.h"
#include "LowPassFilterTheta.h"
#include "HighPassFilterTheta.h"
#include "LowPassFilterPsi.h"
#include "HighPassFilterPsi.h"
#include "LowPassFilterZ.h"

//TimeoutValue declaration
uint32_t    timeoutValue = 5000;  // time interval - us; f_s = 100 Hz

// Prototypes
void *Timer_Irq_Thread(void* resource);
double  cascade( double xin,			//input
				struct biquad *fa,		//biquad array
				int ns);				//no. segments

// Definitions
#define PI 3.141592654
#define IMAX 1000
#define GAINPHI 0
#define GAINTHETA 1
#define GAINPSI 0
#define GAINZ 0

typedef struct {
	NiFpga_IrqContext irqContext;   //Context
	NiFpga_Bool irqThreadRdy;		//ready flag
} ThreadResource;

// I2C variable declarations
MyRio_I2c i2cA;

extern NiFpga_Session myrio_session;
NiFpga_Status status;

// PWM variable declarations
MyRio_Pwm pwmA0;
MyRio_Pwm pwmA1;
MyRio_Pwm pwmA2;
MyRio_Pwm pwmB0;
uint8_t selectRegA;
uint8_t selectRegB;
time_t currentTime;
time_t finalTime;

// PWM Control
Pwm_ClockDivider Divider = Pwm_64x;
uint16_t MaxCounter = 3125;

// Set each ESC with their corresponding duty cycles
uint16_t DutyCycleA0 = 625;
uint16_t DutyCycleA1 = 625;
uint16_t DutyCycleA2 = 625;
uint16_t DutyCycleB0 = 625;
float SelectDutyCycle =  0;
int ac = 0;

double stepInput = 0;
double ThetaRef = 0;

#if !defined(LoopDuration)
#define LoopDuration    60  // How long to output the signal, in seconds
#endif

#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x)) //Saturation MACRO

/*--------------------------------------------------------------
Function AngVeltoCDP()
Purpose: Converts angular velocity of motors to servo signal duty cycle
Parameters: (in) - omega - angular velocity
Returns:   (out) - ConvDutyCycle - servo signal to ESCs
*--------------------------------------------------------------*/
double AngVeltoCDP(double omega){
	double ConvDutyCycle;		// Converted Duty Cycle
	double ConvPercThSat;
	double ConvPercTh = 0.000014341525649*omega*omega+0.032702124122*omega-8.1151624437;
	if(omega<0){
		ConvPercTh = 0;
	}
	ConvPercThSat = SATURATE(ConvPercTh,0,95);
	ConvDutyCycle = (20+ConvPercThSat*0.2)*31.25;
	return ConvDutyCycle;
}

/*--------------------------------------------------------------
Function initializePWM()
Purpose: Initializes the pwm output channels
Parameters: (in) - void
Returns:   (out) - void
*--------------------------------------------------------------*/
void initializePWM(void){
    /*
     * Initialize the PWM struct with registers from the FPGA personality.
     */
    pwmA0.cnfg = PWMA_0CNFG;
    pwmA0.cs = PWMA_0CS;
    pwmA0.max = PWMA_0MAX;
    pwmA0.cmp = PWMA_0CMP;
    pwmA0.cntr = PWMA_0CNTR;

    pwmA1.cnfg = PWMA_1CNFG;
    pwmA1.cs = PWMA_1CS;
    pwmA1.max = PWMA_1MAX;
    pwmA1.cmp = PWMA_1CMP;
    pwmA1.cntr = PWMA_1CNTR;

    pwmA2.cnfg = PWMA_2CNFG;
    pwmA2.cs = PWMA_2CS;
    pwmA2.max = PWMA_2MAX;
    pwmA2.cmp = PWMA_2CMP;
    pwmA2.cntr = PWMA_2CNTR;

    pwmB0.cnfg = PWMB_0CNFG;
    pwmB0.cs = PWMB_0CS;
    pwmB0.max = PWMB_0MAX;
    pwmB0.cmp = PWMB_0CMP;
    pwmB0.cntr = PWMB_0CNTR;
}

/*--------------------------------------------------------------
Function PWMset()
Purpose: Sets PWM parameters
Parameters: (in) - void
Returns:   (out) - status
*--------------------------------------------------------------*/
int PWMset(void){
	    /*
	     * Set the waveform, enabling the PWM onboard device.
	     */
	    Pwm_Configure(&pwmA0, Pwm_Invert | Pwm_Mode,
	            Pwm_NotInverted | Pwm_Enabled);
	    Pwm_Configure(&pwmA1, Pwm_Invert | Pwm_Mode,
	            Pwm_NotInverted | Pwm_Enabled);
	    Pwm_Configure(&pwmA2, Pwm_Invert | Pwm_Mode,
	            Pwm_NotInverted | Pwm_Enabled);
	    Pwm_Configure(&pwmB0, Pwm_Invert | Pwm_Mode,
	            Pwm_NotInverted | Pwm_Enabled);

	    /*
	     * Set the clock divider. The internal PWM counter will increments at
	     * f_clk / 4
	     *
	     * where:
	     *  f_clk = the frequency of the myRIO FPGA clock (40 MHz default)
	     */
	    Pwm_ClockSelect(&pwmA0, Divider);
	    Pwm_ClockSelect(&pwmA1, Divider);
	    Pwm_ClockSelect(&pwmA2, Divider);
	    Pwm_ClockSelect(&pwmB0, Divider);

	    /*
	     * Set the maximum counter value. The counter counts from 0 to 1000.
	     *
	     * The counter increments at 40 MHz / 4 = 10 MHz and the counter counts
	     * from 0 to 1000. The frequency of the PWM waveform is 10 MHz / 1000
	     * = 10 kHz.
	     */
	    Pwm_CounterMaximum(&pwmA0, MaxCounter);
	    Pwm_CounterMaximum(&pwmA1, MaxCounter);
	    Pwm_CounterMaximum(&pwmA2, MaxCounter);
	    Pwm_CounterMaximum(&pwmB0, MaxCounter);

	    /*
	     * PWM outputs are on pins shared with other onboard devices. To output on
	     * a physical pin, select the PWM on the appropriate SELECT register. See
	     * the MUX example for simplified code to enable-disable onboard devices.
	     *
	     * Read the value of the SYSSELECTA register.
	     */
	    status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectRegA);
	    MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not read from the SYSSELECTA register!")

	    status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectRegB);
	    MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not read from the SYSSELECTB register!")

	    /*
	     * Set bit 2 and 3 of the SYSSELECTA register to enable PWMA_0 functionality.
	     * The functionality of the bit is specified in the documentation.
	     */

	    selectRegA = selectRegA | (1 << 2) | (1 << 3) | (1 << 4);

	    /*
	     * Write the updated value of the SYSSELECTA register.
	     */
	    status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectRegA);
	    MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not write to the SYSSELECTA register!")

	    selectRegB= selectRegB | (1 << 2);
	    status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectRegB);
	    MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not write to the SYSSELECTB register!")

	    return status;
}

/*--------------------------------------------------------------
Function main
	Purpose: Set up and enable the IRQ interrupt.
	 Signal the Timer Thread to terminate using the irqThreadRdy flag,
	 then wait for ISR to terminate.
	 9 will end the code.
*--------------------------------------------------------------*/
int main(int argc, char **argv) {
	NiFpga_Status status;
	int32_t status2;
    uint8_t selectReg;

    /*
     * Initialize the I2C struct with registers from the FPGA personality.
     */
    i2cA.addr = I2CAADDR;
    i2cA.cnfg = I2CACNFG;
    i2cA.cntl = I2CACNTL;
    i2cA.cntr = I2CACNTR;
    i2cA.dati = I2CADATI;
    i2cA.dato = I2CADATO;
    i2cA.go = I2CAGO;
    i2cA.stat = I2CASTAT;

    status = MyRio_Open();		    /*Open the myRIO NiFpga Session.*/
    if (MyRio_IsNotSuccess(status)) return status;

    MyRio_IrqTimer irqTimer0;  //Initialize the IRQ Timer
    ThreadResource irqThread0; //Initialize the thread resource structure
    pthread_t thread; //Initialize the ISR thread

    /*
     * Enable the I2C functionality on Connector A
     *
     * Read the value of the SELECTA register.
     */
    status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);

    MyRio_ReturnValueIfNotSuccess(status, status,
                "Could not read from the SYSSELECTA register!");

    /*
     * Set bit7 of the SELECT register to enable the I2C functionality. The
     * functionality of this bit is specified in the documentation.
     */
    selectReg = selectReg | (1 << 7);

    /*
     * Write the updated value of the SELECT register.
     */
    status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);

    MyRio_ReturnValueIfNotSuccess(status, status,
                    "Could not write to the SYSSELECTA register!");

    /*
     * Set the speed of the I2C block.
     *
     * Standard mode (100 kbps) = 187.
     * Fast mode (400 kbps) = 51.
     *
     * These values are calculated using the formula:
     *   f_SCL = f_clk / (2 * CNTR) - 4
     *
     * where:
     *   f_SCL = the desired frequency of the I2C transmission
     *   f_clk = the frequency of the myRIO FPGA clock (40 Mhz default)
     *
     * This formula and its rationale can be found in the documentation.
     */
    I2c_Counter(&i2cA, 187);

     /*
      * Enable the I2C block.
      */
    I2c_Configure(&i2cA, I2c_Enabled);

    //Specify IRQ channel settings
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;

    //Configure Timer IRQ. Terminate if not successful
    status2 = Irq_RegisterTimerIrq(
    		&irqTimer0,
        	&irqThread0.irqContext,
        	timeoutValue);
    if (status2 != 0) return status2;		//check the status of the register

    //Set the indicator to allow the new thread
    irqThread0.irqThreadRdy = NiFpga_True;

	initializePWM();
	PWMset();

    //Create new thread to catch the IRQ
    status2 = pthread_create(
        	&thread,
        	NULL,
        	Timer_Irq_Thread,
        	&irqThread0);
    if (status2 != 0) return status2;		//check the status of the register

	float percentage = 0;
    int MotorSelection;
    float ThetaRef1 = 0;

    // Mode loop
    while (percentage!=-1) {
        printf("Select Motor: ");
        scanf("%d",&MotorSelection);
        switch(MotorSelection){
        case 0:		printf("\nSelect Percentage: ");
        			scanf("%e",&percentage);
        			ac = 0;
        			DutyCycleA0=(20+percentage*0.2)*31.25;
        			DutyCycleA1=(20+percentage*0.2)*31.25;
        			DutyCycleA2=(20+percentage*0.2)*31.25;
        			DutyCycleB0=(20+percentage*0.2)*31.25;
        			printf("\n");
        			break;
        case 1:		printf("\nSelect Percentage: ");
       				scanf("%e",&percentage);
       				DutyCycleA0=(20+percentage*0.2)*31.25;
       				printf("\n");
       				break;
       	case 2:  	printf("\nSelect Percentage: ");
       				scanf("%e",&percentage);
       	    		DutyCycleA1=(20+percentage*0.2)*31.25;
       	    		printf("\n");
       	    		break;
       	case 3:  	printf("\nSelect Percentage: ");
       				scanf("%e",&percentage);
       	    		DutyCycleA2=(20+percentage*0.2)*31.25;
       	    		printf("\n");
       	    		break;
       	case 4:  	printf("\nSelect Percentage: ");
       				scanf("%e",&percentage);
       	    		DutyCycleB0=(20+percentage*0.2)*31.25;
       	    		printf("\n");
       	    		break;
       	case 5:		ac = 1;
       				ThetaRef = 0;
       				break;
       	case 6:		ac = 1;
       				ThetaRef = 20.0*PI/180.0;
       	       		break;
       	case 7:		printf("\nSelect Angle: ");
       	       		scanf("%e",&ThetaRef1);
       	       		ThetaRef = (double)ThetaRef1*PI/180.0;
       	       		printf("\n");
       	       		break;
       	case 9:	percentage = -1;
       				break;
       	default: 	printf("\nWrong Selection\n");//
       				break;
       	}
    }
    irqThread0.irqThreadRdy = NiFpga_False; //signal ISR to terminate by setting irqThreadRdy flag in the ThreadResource structure
    pthread_join(thread, NULL);             //wait for interrupt thread to return NULL

    //Unregister the ISR
    status2 = Irq_UnregisterTimerIrq(
        	&irqTimer0,
        	irqThread0.irqContext);

	status = MyRio_Close();	 /*Close the myRIO NiFpga Session. */
	return status;
}

/*--------------------------------------------------------------
Function *Timer_Irq_Thread()
Purpose: To perform any necessary function in response to the interrupt. In this program, the interrupt thread we are reading and
computing a dynamic signal
Parameters: resource - ThreadResource that is declared above
Returns: N/A
*--------------------------------------------------------------*/
void *Timer_Irq_Thread(void* resource) {
	uint8_t data[12] = {0x60,0x00,0x70,0x60,0x50,0x40,0x30,0x20,0x10,0x90,0xA0,0xB0};
	uint8_t AccelStartStop[2] = {0xC0,0x00};
	uint8_t GyroStartStop[2] = {0xC0,0x00};
	uint8_t MagnetStartStop[2] = {0b01011100,0b00010000};
	uint8_t MagnetStartStop2[2] = {0b00001000,0b00000000};
	uint8_t subAddresses[5]={0x28/*Accel Data Out*/,0x0F/*Who Am I*/,0x20/*CtrlReg6*/,0x18/*Gyro Data Out*/,0x10/*CtrlReg1*/};
	uint8_t subAddresses2[3]={0x20/*CtrlReg1*/,0x23/*CtrlReg4*/,0x28/*Magnetometer Data Out*/};
	uint8_t slaveReadAddress = 0x6B;
	uint8_t slaveReadAddress2 = 0x1E;

	//I2C variables for LidarLite Z-sensor
	uint8_t slaveWriteAddress3 = 0x62;
	uint8_t slaveReadAddress3 = 0x62;
	uint8_t subaddress3[5] = {0x02,0x04,0x1C,0x00,0x8F};
	uint8_t values3[4] = {0x80,0x08,0x00,0x04};
	uint8_t data2[6] = {0x40, 0x50,0x60,0x70,0x80,0x90};

	// Controller reference values
	double PhiRef = 0;
	//double ThetaRef = 0;
	double PsiRef = 0;
	double ZRef = 1.4;
	double Zact;

	// Buffer declarations
	double aXBuffer[IMAX];
	double *pointerX = aXBuffer;
	double aYBuffer[IMAX];
	double *pointerY = aYBuffer;
	double aZBuffer[IMAX];
	double *pointerZ = aZBuffer;

	double gXBuffer[IMAX];
	double *pointerGX = gXBuffer;
	double gYBuffer[IMAX];
	double *pointerGY = gYBuffer;
	double gZBuffer[IMAX];
	double *pointerGZ = gZBuffer;

	double mXBuffer[IMAX];
	double *pointerMX = mXBuffer;
	double mYBuffer[IMAX];
	double *pointerMY = mYBuffer;

	double CompxBuffer[IMAX];
	double *compX = CompxBuffer;
	double CompyBuffer[IMAX];
	double *compY = CompyBuffer;
	double CompzBuffer[IMAX];
	double *compZ = CompzBuffer;
	double HeightBuffer[IMAX];
	double *HeightPointer = HeightBuffer;

	double Omega1Buffer[IMAX];
	double *Omega1Pointer = Omega1Buffer;
	double Omega2Buffer[IMAX];
	double *Omega2Pointer = Omega2Buffer;
	double Omega3Buffer[IMAX];
	double *Omega3Pointer = Omega3Buffer;
	double Omega4Buffer[IMAX];
	double *Omega4Pointer = Omega4Buffer;

	double PercentThrust1Buffer[IMAX];
	double *PercentThrust1Pointer = PercentThrust1Buffer;
	double PercentThrust2Buffer[IMAX];
	double *PercentThrust2Pointer = PercentThrust2Buffer;
	double PercentThrust3Buffer[IMAX];
	double *PercentThrust3Pointer = PercentThrust3Buffer;
	double PercentThrust4Buffer[IMAX];
	double *PercentThrust4Pointer = PercentThrust4Buffer;

	double AngleAccXBuffer[IMAX];
	double *AngleAccXPointer = AngleAccXBuffer;
	double AngleAccYBuffer[IMAX];
	double *AngleAccYPointer = AngleAccYBuffer;
	double AngleMagZBuffer[IMAX];
	double *AngleMagZPointer = AngleMagZBuffer;

	double xOut;
	double yOut;
	double zOut;

	int16_t xOutTemp;
	int16_t yOutTemp;
	int16_t zOutTemp;

	double sqrtval;
	double sqrtval2;
	double angle_accelx;
	double angle_accely;
	double angle_magz;
	double CompAnglex;
	double CompAngley;
	double CompAnglez;

	ThreadResource* threadResource = (ThreadResource*) resource;

	// 9dof sensor initialization
    I2c_WriteByteToSub(&i2cA, slaveReadAddress,subAddresses+2,AccelStartStop,1);     // Turn on Accel.
    I2c_WriteByteToSub(&i2cA, slaveReadAddress,subAddresses+4,GyroStartStop,1);      // Turn on Gyro.
	I2c_WriteByteToSub(&i2cA, slaveReadAddress2,subAddresses2,MagnetStartStop,1);    // Turn on Magnetometer
	I2c_WriteByteToSub(&i2cA, slaveReadAddress2,subAddresses2+1,MagnetStartStop2,1); // Turn on Magnetometer

	// Z sensor initialization
	I2c_WriteByteToSub(&i2cA, slaveWriteAddress3,subaddress3,values3,1);
	I2c_WriteByteToSub(&i2cA, slaveWriteAddress3,subaddress3+1,values3+1,1);
	I2c_WriteByteToSub(&i2cA, slaveWriteAddress3,subaddress3+2,values3+2,1);

    MATFILE *MatFile; //initializing MATFILE type
    int err; //initialize the error in saving to MATLAB

    double Omega0 = 1186;
	uint32_t irqAssert = 0;
	while(threadResource->irqThreadRdy == NiFpga_True){ //while the main thread does not signal this thread to stop
		Irq_Wait( threadResource->irqContext,
				  TIMERIRQNO,
				  &irqAssert,
				  (NiFpga_Bool*) &(threadResource->irqThreadRdy));
		NiFpga_WriteU32( myrio_session,
						 IRQTIMERWRITE,
						 timeoutValue );
		NiFpga_WriteBool( myrio_session,
						  IRQTIMERSETTIME,
						  NiFpga_True);
		if(irqAssert){
			//Interrupt service code here

		    //Read Accel--------------------------------------------
			I2c_ReadByteFromSub(&i2cA, slaveReadAddress,subAddresses,data,6);
		    zOutTemp = (data[5]<< 8) | data[4];
		    yOutTemp = (data[3]<< 8) | data[2];
		    xOutTemp = (data[1]<< 8) | data[0];
		    xOut = (double)xOutTemp * -0.000061*9.81;
		    yOut = (double)yOutTemp * 0.000061*9.81;
		    zOut = (double)zOutTemp * -0.000061*9.81;
		    *pointerX++ = xOut;
		    *pointerY++ = yOut;
		    *pointerZ++ = zOut;

		    // Calculate acceleration angles
		    sqrtval = sqrt((yOut*yOut + zOut*zOut));
    		angle_accely = atan2(xOut,sqrtval);//changed accely to accelx and vice versa here
    		sqrtval2 = sqrt((xOut*xOut + zOut*zOut));
    		angle_accelx = atan2(yOut,sqrtval2);
    		angle_accelx = -angle_accelx;

    		// Save acceleration angle data to buffer
	    	*AngleAccXPointer++ = angle_accelx;
	    	*AngleAccYPointer++ = angle_accely;

	    	// Circular buffer
			if(AngleAccXPointer == (AngleAccXBuffer + IMAX)){
				AngleAccXPointer = AngleAccXBuffer;
				AngleAccYPointer = AngleAccYBuffer;
			}

			//Read Magnetometer
			I2c_ReadByteFromSub(&i2cA, slaveReadAddress,subAddresses,data,6);
		    yOutTemp = (data[3]<< 8) | data[2];
		    xOutTemp = (data[1]<< 8) | data[0];
		    xOut = (double)xOutTemp * 0.00014;
		    yOut = (double)yOutTemp * 0.00014;

		    // Save to buffer
			*pointerMX++ = xOut;
			*pointerMY++ = yOut;

			// Calculate Z angle
			angle_magz = atan2(yOut,xOut)*180/PI;


    		// Save magnetometer angle data to buffer
	    	*AngleMagZPointer++ = angle_magz;

	    	// Circular buffer
			if(AngleMagZPointer == (AngleMagZBuffer + IMAX)){
				AngleMagZPointer = AngleMagZBuffer;
			}

		    //Read Gyro--------------------------------------------------
		    I2c_ReadByteFromSub(&i2cA, slaveReadAddress,subAddresses+3,data,6);
		    zOutTemp = (data[5]<< 8) | data[4];
		    yOutTemp = (data[3]<< 8) | data[2];
		    xOutTemp = (data[1]<< 8) | data[0];
		    xOut = (double)xOutTemp * 0.00875*PI/180;
		    yOut = (double)yOutTemp * -0.00875*PI/180;
		    zOut = (double)zOutTemp * 0.00875*PI/180;

		     // Save to buffer
		    *pointerGX++ = xOut;
		   	*pointerGY++ = yOut;
		   	*pointerGZ++ = zOut;

		   	// Circular buffer
			if(pointerX == (aXBuffer + IMAX)){
			    pointerX = aXBuffer;
			    pointerY = aYBuffer;
			    pointerZ = aZBuffer;
				pointerGX = gXBuffer;
				pointerGY = gYBuffer;
				pointerGZ = gZBuffer;
				pointerMX = mXBuffer;
				pointerMY = mYBuffer;
			}

    		//Complementary Filter
			CompAnglex = cascade(angle_accelx,LowPassFilterPhi,LowPassFilterPhi_ns) + cascade(xOut,HighPassFilterPhi,HighPassFilterPhi_ns)+(PI/180.0);
			CompAngley = cascade(angle_accely,LowPassFilterTheta,LowPassFilterTheta_ns) + cascade(yOut,HighPassFilterTheta,HighPassFilterTheta_ns);
			CompAnglez = cascade(angle_magz,LowPassFilterPsi,LowPassFilterPsi_ns)   + cascade(zOut,HighPassFilterPsi,HighPassFilterPsi_ns);

			// Save data to buffers
			*compX++ = CompAnglex;//
    		*compY++ = CompAngley;
			*compZ++ = CompAnglez;
    		// Circular buffer
			if (compY == (CompyBuffer + IMAX)) {
    			compX = CompxBuffer;
    			compY = CompyBuffer;
				compZ = CompzBuffer;
    		}

			//Read Z-Sensor
			I2c_WriteByteToSub(&i2cA, slaveWriteAddress3,subaddress3+3,values3+3,1);
		    I2c_ReadByteFromSubLidarLite(&i2cA,slaveReadAddress3,subaddress3+4,data2,2);

		    uint16_t distance = data2[0]<< 8 | data2[1];
		    double ZactUF = (double)distance;

		    double ZactF = cascade(ZactUF,LowPassFilterZ,LowPassFilterZ_ns);

		    Zact = ZactF*(1.0/100.0)*cos(CompAnglex)*cos(CompAngley);

    		// Save height data to buffer
	    	*HeightPointer++ = Zact;

	    	// Circular buffer
			if(HeightPointer == (HeightBuffer + IMAX)){
				HeightPointer = HeightBuffer;
			}

			// Calculate controller errors
			double ePhi = (PhiRef - CompAnglex);
			double eTheta = (ThetaRef - CompAngley);
			double ePsi = (PsiRef - CompAnglez);
			double eZ = (ZRef - Zact);

			// Calculate controller corrections
			double PhiCorr = GAINPHI*cascade(ePhi,PhiController,PhiController_ns);
			double ThetaCorr = GAINTHETA*cascade(eTheta,ThetaController,ThetaController_ns);
			double PsiCorr = GAINPSI*cascade(ePsi,PsiController,PsiController_ns);
			double ZCorr = GAINZ*cascade(eZ,ZController,ZController_ns);

			// Motor mixing logic
			double Omega1 = -ThetaCorr + PsiCorr + ZCorr + Omega0;
			double Omega2 = -PhiCorr - PsiCorr + ZCorr + Omega0;
			double Omega3 = ThetaCorr + PsiCorr + ZCorr + Omega0;
			double Omega4 = PhiCorr - PsiCorr + ZCorr + Omega0;

			// Save motor speeds to buffer
			*Omega1Pointer++ = Omega1;
			*Omega2Pointer++ = Omega2;
			*Omega3Pointer++ = Omega3;
			*Omega4Pointer++ = Omega4;

			// Circular buffer
			if (Omega1Pointer == (Omega1Buffer + IMAX)) {
				Omega1Pointer = Omega1Buffer;
				Omega2Pointer = Omega2Buffer;
				Omega3Pointer = Omega3Buffer;
				Omega4Pointer = Omega4Buffer;
			}

			if(ac == 1){
			// Change the calculated speed into Duty Cycle
				DutyCycleA0=AngVeltoCDP(Omega2);
				DutyCycleA1=AngVeltoCDP(Omega1);
				DutyCycleA2=AngVeltoCDP(Omega4);
				DutyCycleB0=AngVeltoCDP(Omega3);
			}

			// Save percent thrust to buffers
			*PercentThrust1Pointer++ = ((DutyCycleA1/31.25)-20)/0.2;
			*PercentThrust2Pointer++ = ((DutyCycleA0/31.25)-20)/0.2;
			*PercentThrust3Pointer++ = ((DutyCycleB0/31.25)-20)/0.2;
			*PercentThrust4Pointer++ = ((DutyCycleA2/31.25)-20)/0.2;

			// Circular buffers
			if (PercentThrust1Pointer == (PercentThrust1Buffer + IMAX)) {
				PercentThrust1Pointer = PercentThrust1Buffer;
				PercentThrust2Pointer = PercentThrust2Buffer;
				PercentThrust3Pointer = PercentThrust3Buffer;
				PercentThrust4Pointer = PercentThrust4Buffer;
			}

			// Write duty cylces to ESCs
			Pwm_CounterCompare(&pwmA0, DutyCycleA0);
			Pwm_CounterCompare(&pwmA1, DutyCycleA1);
			Pwm_CounterCompare(&pwmA2, DutyCycleA2);
			Pwm_CounterCompare(&pwmB0, DutyCycleB0);

			//Acknowledge the IRQ has been asserted
			Irq_Acknowledge(irqAssert);
		}
	}

	// 9dof close down sequence
	I2c_WriteByteToSub(&i2cA, slaveReadAddress,subAddresses+2,AccelStartStop+1,1); // Turn off Accel.
	I2c_WriteByteToSub(&i2cA, slaveReadAddress,subAddresses+4,GyroStartStop+1,1); // Turn off Gyro.
	I2c_WriteByteToSub(&i2cA, slaveReadAddress2,subAddresses2,MagnetStartStop+1,1); // Turn off Magnetometer
	I2c_WriteByteToSub(&i2cA, slaveReadAddress2,subAddresses2+1,MagnetStartStop2+1,1); // Turn off Magnetometer

	// Method for saving input and out values to a MATLAB file
	MatFile = openmatfile("DataCollection19.mat", &err);
	if(!MatFile)  {
		printf_lcd("Can’t open mat file %d\n", err);///////
	}

	// Save buffers to matlab
	matfile_addstring(MatFile, "comment", "change of theta ref of +10 deg");///////
	matfile_addmatrix(MatFile, "a_x", aXBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "a_y", aYBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "a_z", aZBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "g_x", gXBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "g_y", gYBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "g_z", gZBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "comp_x", CompxBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "comp_y", CompyBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "comp_z", CompzBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "Height", HeightBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "Omega1", Omega1Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "Omega2", Omega2Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "Omega3", Omega3Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "Omega4", Omega4Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "PercentThrust1", PercentThrust1Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "PercentThrust2", PercentThrust2Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "PercentThrust3", PercentThrust3Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "PercentThrust4", PercentThrust4Buffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "AngleAccX", AngleAccXBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "AngleAccy", AngleAccYBuffer, IMAX, 1, 0);
	matfile_addmatrix(MatFile, "AngleMagZ", AngleMagZBuffer, IMAX, 1, 0);
	matfile_close(MatFile);

	pthread_exit(NULL); //terminate the new thread and return from the function
	return NULL;//
}

/*--------------------------------------------------------------
Function cascade()
Purpose: Calculate the output value of the signal using the biquad cascade method
Parameters: (in) - xin, input signal
			(in) - *fa, biquad array
			(in) - ns, number of segments
			(in) - ymin, minimum saturation voltage
			(in) - ymax, maximum saturation voltage
Returns: y0 - output signal value
*--------------------------------------------------------------*/
double  cascade( double xin, struct biquad *fa, int ns)  {
	double y0 = 0; //initialize the output to zero
	int i;
	struct biquad *f; //initialize pointer for biquad structure
	f = fa; //set the biquad pointer to myFilter
	f->x0 = xin; //set the initial x0 to the input signal
	for (i=0; i < ns; i++)  {
		if (i != 0) { //every biquad except the first
			f->x0 = y0; //set the x0 to the output value of the previous biquad
		}
		y0 = (f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2) /(f->a0); //calculate the biquad cascade
		f->x2 = f->x1; //set x2 equal to x1 of the previous iteration
		f->x1 = f->x0; //set x1 equal to x0 of the previous iteration
		f->y2 = f->y1; //set y2 equal to y1 of the previous iteration
		f->y1 = y0; //set y1 equal to the output y0 of the biquad cascade
		f++; //increment the biquad pointer
	}
	return y0; //return the output value
}
