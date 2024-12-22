/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xiicps.h"
#include "timer_ps.h"
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "testvector.h"
#include <arm_neon.h>
/* I2S Register offsets */
#define I2S_RESET_REG 		0x00
#define I2S_CTRL_REG 		0x04
#define I2S_CLK_CTRL_REG 	0x08
#define I2S_FIFO_STS_REG 	0x20
#define I2S_RX_FIFO_REG 	0x28
#define I2S_TX_FIFO_REG 	0x2C

#define FIFO_ISR ( 0x00)
#define FIFO_IER ( 0x04)
#define FIFO_TDFV ( 0x0C)
#define FIFO_RDFO ( 0x1C)
#define FIFO_TDR ( 0x2C)
#define FIFO_TDFD ( 0x10)
#define FIFO_TLR ( 0x14)


#define FIFO_RLR ( 0x24)
#define FIFO_RDFD ( 0x20)
#define FIFO_RDR ( 0x30)

/* IIC address of the SSM2603 device and the desired IIC clock speed */
#define IIC_SLAVE_ADDR		0b0011010
#define IIC_SCLK_RATE		100000


#define AUDIO_IIC_ID XPAR_XIICPS_0_DEVICE_ID
#define AUDIO_CTRL_BASEADDR XPAR_AXI_I2S_ADI_0_S00_AXI_BASEADDR
#define SCU_TIMER_ID XPAR_SCUTIMER_DEVICE_ID


#define SWI_BASE_ADDR XPAR_AXI_GPIO_2_BASEADDR
#define LED_BASE_ADDR XPAR_AXI_GPIO_1_BASEADDR
#define BUT_BASE_ADDR XPAR_AXI_GPIO_0_BASEADDR

int * swiData = SWI_BASE_ADDR;
int * butData = BUT_BASE_ADDR;
int * ledData = LED_BASE_ADDR;

#define AUDIO_FIFO XPAR_AXI_FIFO_MM_S_0_BASEADDR

#define FIR_FIFO XPAR_AXI_FIFO_MM_S_1_BASEADDR

#define GLOBAL_TMR_BASEADDR XPAR_PS7_GLOBALTIMER_0_S_AXI_BASEADDR
/* ------------------------------------------------------------ */
/*				Low-Pass and High-Pass FIR filter coefficients									*/
/* ------------------------------------------------------------ */
#define N_LP 29
#define coeffLP -0.008747420411798365, -0.01352684070757768, -0.021069157456114974, -0.02821205662046602, -0.03288466862750655, -0.032820056352546804, -0.026015856418133178, -0.011326253746998683, 0.01118086152569252, 0.039926269347420495, 0.07195575020178693, 0.10331516426959793, 0.12972205191226951, 0.14735052987683003, 0.15353880775461448, 0.14735052987683003, 0.12972205191226951, 0.10331516426959793, 0.07195575020178693, 0.039926269347420495, 0.01118086152569252, -0.011326253746998683, -0.026015856418133178, -0.032820056352546804, -0.03288466862750655, -0.02821205662046602, -0.021069157456114974, -0.01352684070757768, -0.008747420411798365
/* Lab 3 HP - corresponds to test */
//#define N_HP 25
//#define coeffHP   0.05946436587252379,   -0.08266255914551396,  -0.032374303236116855,   0.00216595808715192,   0.02865430955587078,   0.045067235048989344,   0.04435253660179216,   0.02016730464237364,   -0.027703198625664668,   -0.0913071374380985, -0.15595705304807897, -0.2038996657538856,   0.7782265468798992,   -0.2038996657538856,   -0.15595705304807897,   -0.0913071374380985,   -0.027703198625664668,   0.02016730464237364,   0.04435253660179216,   0.045067235048989344,   0.02865430955587078,   0.00216595808715192,   -0.032374303236116855,   -0.08266255914551396,   0.05946436587252379

/* own 99 tap HP filter */
//#define N_HP 99
//#define coeffHP 0.00000970670390470049,-0.00009904347568503034,0.00013080079577246106,0.000056425762882671974,-0.0001574771196190662,-0.00012651771047713666,0.0002192740367567676,0.00024952953977045085,-0.0002740676555714267,-0.0004443436792739645,0.0002877658392691507,0.0007214351964558149,-0.00021389813765200507,-0.001075406317750377,-0.000004839792279698105,0.0014763328689236608,0.0004297110984788854,-0.0018624372213330565,-0.001114006890062063,0.0021359861672373384,0.002087444884912193,-0.0021647383185726776,-0.0033382851321141736,0.0017904189318580868,0.0047960400095039685,-0.0008446740147776873,-0.006317531038242502,-0.000828462066058214,0.0076790896713283185,0.00334608397542307,-0.008576216380623435,-0.006759727308986433,0.008629537259983378,0.011032990869819048,-0.007391836288249267,-0.016028083937562408,0.00434313106304681,0.021505009217800672,0.0011552247173753176,-0.027135223923131783,-0.010021444420809463,0.03252965626050076,0.02395883943202481,-0.03727793600193775,-0.047252197725143746,0.04099427409037286,0.09581649011316974,-0.04336258879805745,-0.31480504218237204,0.5441761567954407,-0.31480504218237204,-0.04336258879805745,0.09581649011316974,0.04099427409037286,-0.047252197725143746,-0.03727793600193775,0.02395883943202481,0.03252965626050076,-0.010021444420809463,-0.027135223923131783,0.0011552247173753176,0.021505009217800672,0.00434313106304681,-0.016028083937562408,-0.007391836288249267,0.011032990869819048,0.008629537259983378,-0.006759727308986433,-0.008576216380623435,0.00334608397542307,0.0076790896713283185,-0.000828462066058214,-0.006317531038242502,-0.0008446740147776873,0.0047960400095039685,0.0017904189318580868,-0.0033382851321141736,-0.0021647383185726776,0.002087444884912193,0.0021359861672373384,-0.001114006890062063,-0.0018624372213330565,0.0004297110984788854,0.0014763328689236608,-0.000004839792279698105,-0.001075406317750377,-0.00021389813765200507,0.0007214351964558149,0.0002877658392691507,-0.0004443436792739645,-0.0002740676555714267,0.00024952953977045085,0.0002192740367567676,-0.00012651771047713666,-0.0001574771196190662,0.000056425762882671974,0.00013080079577246106,-0.00009904347568503034,0.00000970670390470049

/* huge 195 tap HP*/
//#define N_HP 175
//#define coeffHP -7.075610847072565e-8,3.465249630663874e-7,-2.1033718264015392e-7,-6.167714993207277e-7,3.4495082217993785e-7,0.0000013317685895862321,-4.014112141416591e-7,-0.0000026151827032748287,7.455062534126152e-8,0.000004649959903407615,0.0000011560493867045021,-0.000007484988617732581,-0.000004074660560012346,0.000010853069504952145,0.0000096970829696863,-0.000013945465078176892,-0.000019113899989121132,0.000015176159721007133,0.000033178179961375696,-0.000012016800472601626,-0.000052015517996142666,9.977208670448284e-7,0.00007438909600169083,0.000022008408519615305,-0.00009700698537259569,-0.00006112049809250714,0.0001139279228366138,0.00011934981986778544,-0.00011628409572361331,-0.00019710051776414923,0.0000925682484434517,0.00029042551606604534,-0.000029720362679339323,-0.0003893257651442169,-0.00008483343032141758,0.00047650712390693325,0.00026018597497222016,-0.0005270891154857476,-0.0004982062107524067,0.0005097590678713487,0.0007895041355866019,-0.0003897597472177696,-0.0011097976099409314,0.00013386952081752415,0.001417553575389527,0.00028280013746796354,-0.001653835200221917,-0.0008687730076503719,0.0017451813892851828,0.0016079402866017694,-0.0016100531362910292,-0.002452395877936809,0.0011689107012622837,0.0033178981951769667,-0.0003573891764755326,-0.004083431858824638,-0.00085859700529033,0.004596019461688248,0.002467549517854464,-0.004681335824531372,-0.004398963379649619,0.004159860883054269,0.006513891451735887,-0.002867378985383058,-0.008601194926026819,0.0006777344354708019,0.010380268638635256,0.0024749508161007443,-0.011509540935975266,-0.006577753408106493,0.011597808215807477,0.01152718617229691,-0.010211924470176185,-0.017126683132655564,0.006867784298940861,0.023094784935407092,-0.0009762822106656304,-0.029084114525048846,-0.008327912332870745,0.03470973049653017,0.022702395221161447,-0.03958403307772793,-0.04640146238429276,0.04335430595479987,0.095329337370141,-0.04573839694352436,-0.31464696210077825,0.5465540878963048,-0.31464696210077825,-0.04573839694352436,0.095329337370141,0.04335430595479987,-0.04640146238429276,-0.03958403307772793,0.022702395221161447,0.03470973049653017,-0.008327912332870745,-0.029084114525048846,-0.0009762822106656304,0.023094784935407092,0.006867784298940861,-0.017126683132655564,-0.010211924470176185,0.01152718617229691,0.011597808215807477,-0.006577753408106493,-0.011509540935975266,0.0024749508161007443,0.010380268638635256,0.0006777344354708019,-0.008601194926026819,-0.002867378985383058,0.006513891451735887,0.004159860883054269,-0.004398963379649619,-0.004681335824531372,0.002467549517854464,0.004596019461688248,-0.00085859700529033,-0.004083431858824638,-0.0003573891764755326,0.0033178981951769667,0.0011689107012622837,-0.002452395877936809,-0.0016100531362910292,0.0016079402866017694,0.0017451813892851828,-0.0008687730076503719,-0.001653835200221917,0.00028280013746796354,0.001417553575389527,0.00013386952081752415,-0.0011097976099409314,-0.0003897597472177696,0.0007895041355866019,0.0005097590678713487,-0.0004982062107524067,-0.0005270891154857476,0.00026018597497222016,0.00047650712390693325,-0.00008483343032141758,-0.0003893257651442169,-0.000029720362679339323,0.00029042551606604534,0.0000925682484434517,-0.00019710051776414923,-0.00011628409572361331,0.00011934981986778544,0.0001139279228366138,-0.00006112049809250714,-0.00009700698537259569,0.000022008408519615305,0.00007438909600169083,9.977208670448284e-7,-0.000052015517996142666,-0.000012016800472601626,0.000033178179961375696,0.000015176159721007133,-0.000019113899989121132,-0.000013945465078176892,0.0000096970829696863,0.000010853069504952145,-0.000004074660560012346,-0.000007484988617732581,0.0000011560493867045021,0.000004649959903407615,7.455062534126152e-8,-0.0000026151827032748287,-4.014112141416591e-7,0.0000013317685895862321,3.4495082217993785e-7,-6.167714993207277e-7,-2.1033718264015392e-7,3.465249630663874e-7,-7.075610847072565e-8

/* Lab 5 99 tap filter */
#define N_HP 99
#define coeffHP 0.009070325134076762,0.010925139635915718,0.014865100117983266,0.017271841382238834,0.017110345805612836,0.013841935730177184,0.00768287542394269,-0.00033570096284880283,-0.00855651230212637,-0.015115420596300298,-0.018474581597192972,-0.01790242926323334,-0.013759770086619688,-0.00744663575478999,-0.0010100562754879574,0.003463916991779191,0.004531288056068554,0.0018722717349563603,-0.0035542663695052566,-0.009796657613985139,-0.014541849551298591,-0.015915628163664194,-0.013167404246309791,-0.007005763514976244,0.0005611702074044075,0.00686695102099133,0.009502563201542347,0.007217454026646569,0.00047005901783785017,-0.008583405252049323,-0.01671855705381644,-0.020713194107472837,-0.01853134409823149,-0.010201862058174558,0.001954856182055768,0.013879813474819404,0.021015679003174942,0.019872032786541934,0.009429442882055768,-0.008098291693112642,-0.027486975958639432,-0.04171078091904579,-0.04389496506780281,-0.029371072656094826,0.0025996976326444813,0.04822047963918811,0.09968235289644688,0.14692306063041927,0.1800952027638827,0.19202814998085552,0.1800952027638827,0.14692306063041927,0.09968235289644688,0.04822047963918811,0.0025996976326444813,-0.029371072656094826,-0.04389496506780281,-0.04171078091904579,-0.027486975958639432,-0.008098291693112642,0.009429442882055768,0.019872032786541934,0.021015679003174942,0.013879813474819404,0.001954856182055768,-0.010201862058174558,-0.01853134409823149,-0.020713194107472837,-0.01671855705381644,-0.008583405252049323,0.00047005901783785017,0.007217454026646569,0.009502563201542347,0.00686695102099133,0.0005611702074044075,-0.007005763514976244,-0.013167404246309791,-0.015915628163664194,-0.014541849551298591,-0.009796657613985139,-0.0035542663695052566,0.0018722717349563603,0.004531288056068554,0.003463916991779191,-0.0010100562754879574,-0.00744663575478999,-0.013759770086619688,-0.01790242926323334,-0.018474581597192972,-0.015115420596300298,-0.00855651230212637,-0.00033570096284880283,0.00768287542394269,0.013841935730177184,0.017110345805612836,0.017271841382238834,0.014865100117983266,0.010925139635915718,0.009070325134076762

#define MAX_N (N_HP > N_LP)? N_HP : N_LP

float LP[]={coeffLP};
float HP[]={coeffHP};
float LPBufferL[N_LP] = {0};
float HPBufferL[N_HP] = {0};
float LPBufferR[N_LP] = {0};
float HPBufferR[N_HP] = {0};

/* ------------------------------------------------------------ */
/*                      Test Filters                            */
/* ------------------------------------------------------------ */

#define FILTERLEN 200

int inputTest250[] = {inputTest_250};
int inputTest500[] = {inputTest_500};
int inputTest1000[] = {inputTest_1000};
int inputTest2000[] = {inputTest_2000};
int inputTest4000[] = {inputTest_4000};
int inputTest6000[] = {inputTest_6000};
int inputTest8000[] = {inputTest_8000};

int outputTest250LP[] = {outputTest_F_250_LP};
int outputTest500LP[] = {outputTest_F_500_LP};
int outputTest1000LP[] = {outputTest_F_1000_LP};
int outputTest2000LP[] = {outputTest_F_2000_LP};
int outputTest4000LP[] = {outputTest_F_4000_LP};
int outputTest6000LP[] = {outputTest_F_6000_LP};
int outputTest8000LP[] = {outputTest_F_8000_LP};

int outputTest250HP[] = {outputTest_F_250_HP};
int outputTest500HP[] = {outputTest_F_500_HP};
int outputTest1000HP[] = {outputTest_F_1000_HP};
int outputTest2000HP[] = {outputTest_F_2000_HP};
int outputTest4000HP[] = {outputTest_F_4000_HP};
int outputTest6000HP[] = {outputTest_F_6000_HP};
int outputTest8000HP[] = {outputTest_F_8000_HP};


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

XIicPs Iic;		/* Instance of the IIC Device */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

int AudioRegSet(XIicPs *IIcPtr, u8 regAddr, u16 regData)
{
	int Status;
	u8 SendBuffer[2];

	SendBuffer[0] = regAddr << 1;
	SendBuffer[0] = SendBuffer[0] | ((regData >> 8) & 0b1);

	SendBuffer[1] = regData & 0xFF;

	Status = XIicPs_MasterSendPolled(IIcPtr, SendBuffer,
				 2, IIC_SLAVE_ADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("IIC send failed\n\r");
		return XST_FAILURE;
	}
	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while (XIicPs_BusIsBusy(IIcPtr)) {
		/* NOP */
	}
	return XST_SUCCESS;

}
/***	AudioInitialize(u16 timerID,  u16 iicID, u32 i2sAddr)
**
**	Parameters:
**		timerID - DEVICE_ID for the SCU timer
**		iicID 	- DEVICE_ID for the PS IIC controller connected to the SSM2603
**		i2sAddr - Physical Base address of the I2S controller
**
**	Return Value: int
**		XST_SUCCESS if successful
**
**	Errors:
**
**	Description:
**		Initializes the Audio demo. Must be called once and only once before calling
**		AudioRunDemo
**
*/
int AudioInitialize(u16 timerID,  u16 iicID, u32 i2sAddr) //, u32 i2sTransmAddr, u32 i2sReceivAddr)
{
	int Status;
	XIicPs_Config *Config;
	u32 i2sClkDiv;

	TimerInitialize(timerID);

	/*
	 * Initialize the IIC driver so that it's ready to use
	 * Look up the configuration in the config table,
	 * then initialize it.
	 */
	Config = XIicPs_LookupConfig(iicID);
	if (NULL == Config) {
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XIicPs_SelfTest(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the IIC serial clock rate.
	 */
	Status = XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	/*
	 * Write to the SSM2603 audio codec registers to configure the device. Refer to the
	 * SSM2603 Audio Codec data sheet for information on what these writes do.
	 */
	Status = AudioRegSet(&Iic, 15, 0b000000000); //Perform Reset
	TimerDelay(75000);
	Status |= AudioRegSet(&Iic, 6, 0b000110000); //Power up
	Status |= AudioRegSet(&Iic, 0, 0b000010111);
	Status |= AudioRegSet(&Iic, 1, 0b000010111);
	Status |= AudioRegSet(&Iic, 2, 0b101111001);
	Status |= AudioRegSet(&Iic, 4, 0b000010000);
	Status |= AudioRegSet(&Iic, 5, 0b000000000);
	Status |= AudioRegSet(&Iic, 7, 0b000001010); //Changed so Word length is 24
	Status |= AudioRegSet(&Iic, 8, 0b000000000); //Changed so no CLKDIV2
	TimerDelay(75000);
	Status |= AudioRegSet(&Iic, 9, 0b000000001);
	Status |= AudioRegSet(&Iic, 6, 0b000100000);
	Status = AudioRegSet(&Iic, 4, 0b000010000);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	i2sClkDiv = 1; //Set the BCLK to be MCLK / 4
	i2sClkDiv = i2sClkDiv | (31 << 16); //Set the LRCLK's to be BCLK / 64

	Xil_Out32(i2sAddr + I2S_CLK_CTRL_REG, i2sClkDiv); //Write clock div register

	Xil_Out32(AUDIO_CTRL_BASEADDR + I2S_RESET_REG, 0b110); //Reset RX and TX FIFOs
	Xil_Out32(AUDIO_CTRL_BASEADDR + I2S_CTRL_REG, 0b011); //Enable RX Fifo and TX FIFOs, disable mute
	return XST_SUCCESS;
}

void FifoWrite (u32 fifoBaseAddr, u32 data)
{

	Xil_Out32(fifoBaseAddr + 0x10, data); // write DATA
    Xil_Out32(fifoBaseAddr + 0x14, 4);    // write the length of the DATA (4 bytes)

	while ((Xil_In32(fifoBaseAddr + FIFO_ISR)&0x08000000)!=0x08000000){;}  // waits for the transmission completes
	Xil_Out32(fifoBaseAddr + FIFO_ISR, 0x08000000);  // ack the transmission complete


}

u32 FifoRead (u32 fifoBaseAddr)
{

	while (Xil_In32(fifoBaseAddr + FIFO_RDFO)==0){;} // waits for a sample in the FIFO
	int data = Xil_In32(fifoBaseAddr + FIFO_RDFD);   // read the sample from the FIFO
return data;

}

void initialize_FIFO(u32 fifoAddr){
	Xil_Out32(AUDIO_FIFO + 0x2c, 0);

	    // init
	    xil_printf("FIFO_ISR:  0x%08x\n",Xil_In32(fifoAddr + FIFO_ISR));
	    print("write FIFO_ISR\n\r");
	    Xil_Out32(fifoAddr + FIFO_ISR, 0xFFFFFFFF);
	    xil_printf("FIFO_ISR:  0x%08x\n",Xil_In32(fifoAddr + FIFO_ISR));
	    xil_printf("FIFO_IER:  0x%08x\n",Xil_In32(fifoAddr + FIFO_IER));
	    xil_printf("FIFO_TDFV: 0x%08x\n",Xil_In32(fifoAddr + FIFO_TDFV));
	    xil_printf("FIFO_RDFO: 0x%08x\n",Xil_In32(fifoAddr + FIFO_RDFO));

	    print("Write IER\n\r");
	    Xil_Out32(fifoAddr + FIFO_IER, 0x0C000000);

	    print("Write TDR\n\r");
	    Xil_Out32(fifoAddr + FIFO_TDR, 0x00000000);


	    xil_printf("FIFO_ISR:  0x%08x\n",Xil_In32(fifoAddr + FIFO_ISR));
		print("write FIFO_ISR\n\r");
		Xil_Out32(fifoAddr + FIFO_ISR, 0xFFFFFFFF);
		xil_printf("FIFO_ISR:  0x%08x\n",Xil_In32(fifoAddr + FIFO_ISR));
		xil_printf("FIFO_IER:  0x%08x\n",Xil_In32(fifoAddr + FIFO_IER));
		xil_printf("FIFO_TDFV: 0x%08x\n",Xil_In32(fifoAddr + FIFO_TDFV));
		xil_printf("FIFO_RDFO: 0x%08x\n",Xil_In32(fifoAddr + FIFO_RDFO));


	    print("write FIFO_IER\n");
	    Xil_Out32(fifoAddr + FIFO_IER, 0x04100000);
	    xil_printf("FIFO_ISR:  0x%08x\n",Xil_In32(fifoAddr + FIFO_ISR));
	    print("write FIFO_ISR\n");
	    Xil_Out32(fifoAddr + FIFO_ISR, 0x00100000);



}

/* neon dot product */
float dot_product_intrinsic(float * vec1, float *  vec2, int n)
{
	float32x4_t vec1_q, vec2_q;
	float32x4_t sum_q = {0.0, 0.0, 0.0, 0.0};
	float32x2_t tmp[2];
	float result;
	for( int i=0; i<( n & ~3); i+=4 )
	{
		vec1_q=vld1q_f32(&vec1[i]);
		vec2_q=vld1q_f32(&vec2[i]);
		sum_q = vmlaq_f32(sum_q, vec1_q, vec2_q );
	}

	tmp[0] = vget_high_f32(sum_q);
	tmp[1] = vget_low_f32 (sum_q);
	tmp[0] = vpadd_f32(tmp[0], tmp[1]);
	tmp[0] = vpadd_f32(tmp[0], tmp[0]);
	result = vget_lane_f32(tmp[0], 0);
	return result;
}

#define TIMESAMPLES 50
#define CLOCKFREQ 333000000 // 333 MHz

//#define TESTFILTER /* toggles testing the filter function */
/* Note: Test does only work with the corresponding LP and HP filters and breaks with optimization >= -O2 */

#define FILTER /* toggles implementation of filters (own function and accelerator) */
/*Note: deactivating this allows to measure sample frequency without buffer writing */

//#define FREQTIMER /* measure the sample frequency */
//#define FILTERTIMER /* measure the time needed for the filter (buffer appending + convolution) */
#define CONVOTIMER /* measure the convolution time in the filter */
/* Note: only one timer can be active */

//#define NEONINTRINSICS /* use neon intrinsic functions in filter convolution */

void rShiftFloat(float* array, int len);
void rShiftInt(int* array, int len);
void filterWrite(int inputL, int inputR);
int filterApply(int filterType, int * outL, int * outR);
int filterTest();

/* Functionality:
 * Switch 1: toggle LP filter
 * Switch 2: toggle HP filter
 * Switch 3: toggle FIR accelerator
 * ( only one filter can be active at a time )
 * Button 1: stop filtering and output measured timer (if defined)
 */
int main()
{
    init_platform();

    print("Started!\n\r");


	int audio_status = AudioInitialize(SCU_TIMER_ID, AUDIO_IIC_ID, AUDIO_CTRL_BASEADDR);

	/* check for successful audio initialization -> prevent audio output problem (static noise) with -O2 optimization */
	if (audio_status != XST_SUCCESS){
		xil_printf("Failed Audio Initialize");
		exit(EXIT_FAILURE);
	}


	initialize_FIFO(AUDIO_FIFO);
	initialize_FIFO(FIR_FIFO);

#ifdef TESTFILTER
	if (filterTest() == -1){
		xil_printf("Failed Test");
		exit(EXIT_FAILURE);
	}
	print("Finished Test\n");
#endif

	//Xil_ICacheDisable();
	//Xil_DCacheDisable();

	int sampleL, sampleR;
	int outputL, outputR;

#if defined(FILTERTIMER) || defined(FREQTIMER) || defined(CONVOTIMER)
	TimerInitialize(SCU_TIMER_ID);
	int tCounter = 0;
	int tSamples[TIMESAMPLES] = {0};
	int t1;
	int t2;
	int clockTicsMean = 0;
#endif
	while (1){

		sampleL = FifoRead(AUDIO_FIFO);
		sampleR = FifoRead(AUDIO_FIFO);

#if defined(FREQTIMER) && !defined(FILTERTIMER)
		if (tCounter > 600 && tCounter % 2 == 1){
			t1 = Xil_In32(GLOBAL_TMR_BASEADDR);
		}
		else if (tCounter > 600 && tCounter % 2 == 0) {
			t2 = Xil_In32(GLOBAL_TMR_BASEADDR);
			rShiftInt(tSamples, TIMESAMPLES-1); // leave null termination in the end
			tSamples[0] = t2-t1;
		}
		tCounter++;
#endif
#if defined(FILTERTIMER) && !defined(FREQTIMER)
		t1 = Xil_In32(GLOBAL_TMR_BASEADDR);
#endif

#ifdef FILTER /* start exec filter */

		filterWrite(sampleL, sampleR); /* add new samples to the corresponding buffers */

#if defined(CONVOTIMER) && !defined(FREQTIMER) && !defined(FILTERTIMER)
		t1 = Xil_In32(GLOBAL_TMR_BASEADDR);
#endif

		if (*swiData == 1) { /* apply LP filter */
			*ledData = 1;
			filterApply(1, &outputL, &outputR);
		}
		else if (*swiData == 2) { /* apply HP filter */
			*ledData = 2;
			filterApply(2, &outputL, &outputR);
		}
		else if (*swiData == 4) { /* use FIR accelerator */
			*ledData = 4;
			FifoWrite(FIR_FIFO, sampleL);
			FifoWrite(FIR_FIFO, sampleR);
			outputL = FifoRead(FIR_FIFO);
			outputR = FifoRead(FIR_FIFO);
		}
		else { /* don't filter input */
			*ledData = 0;
			outputL = sampleL;
			outputR = sampleR;
		}
		/* end exec filter */
#endif

#ifndef FILTER
		outputL = sampleL;
		outputR = sampleR;
#endif

#if defined(FILTERTIMER) || defined(CONVOTIMER) && !defined(FREQTIMER)
		t2 = Xil_In32(GLOBAL_TMR_BASEADDR);
		rShiftInt(tSamples, TIMESAMPLES - 1); // leave null termination in the end
		tSamples[0] = t2-t1;
		tCounter++;
#endif
		FifoWrite(AUDIO_FIFO, outputL);
		FifoWrite(AUDIO_FIFO, outputR);
		/* Stop on button-press and receive timer data */
		if (*butData != 0) {
			break;
		}
	}
#if defined(FREQTIMER) || defined(FILTERTIMER) || defined(CONVOTIMER)
	int len = 0;
	for (int i = 0; tSamples[i]; i++){ /* collect all time samples (ending with null termination) */
		clockTicsMean += tSamples[i];
		len++;
	}
	clockTicsMean /= len;
	int freq = CLOCKFREQ / clockTicsMean;
	char strbuf[10];
	print("Tics:\n");
	print(itoa(clockTicsMean, strbuf, 10));
	print("\n");
	print("Freq:\n");
	print(itoa(freq, strbuf, 10));
#endif

	cleanup_platform();
	return 0;
}

void rShiftFloat(float* array, int len){
	for (int i = len-1; i > 0; i--){ // Note: leaves the first index the same
		array[i] = array[i-1];
	}
}

void rShiftInt(int* array, int len){
	for (int i = len-1; i > 0; i--){ // Note: leaves the first index the same
		array[i] = array[i-1];
	}
}

void filterWrite(int inputL, int inputR) { /* write to the filter buffers */
	/* make space for new input */
	rShiftFloat(&LPBufferL[0], N_LP);
	rShiftFloat(&HPBufferL[0], N_HP);
	rShiftFloat(&LPBufferR[0], N_LP);
	rShiftFloat(&HPBufferR[0], N_HP);
	/* append values in buffer as float */
	LPBufferL[0] = (float)inputL;
	HPBufferL[0] = (float)inputL;
	LPBufferR[0] = (float)inputR;
	HPBufferR[0] = (float)inputR;
}

int filterApply(int filterType, int * outL, int * outR){
	/* filterType: 1 == "L"; 2 == "H" */
	float *activeFilterL, *activeFilterR;
	int activeFilterLen;
	float *activeFilterCoeff;
	if (filterType == 1){
		activeFilterL = &LPBufferL[0];
		activeFilterR = &LPBufferR[0];
		activeFilterLen = N_LP;
		activeFilterCoeff = &LP[0];
	}
	else if (filterType == 2) {
		activeFilterL = &HPBufferL[0];
		activeFilterR = &HPBufferR[0];
		activeFilterLen = N_HP;
		activeFilterCoeff = &HP[0];
	}
	else {
		return -1;
	}

	/* apply filter */
	float outputL = 0;
	float outputR = 0;
#ifdef NEONINTRINSICS
	outputL = dot_product_intrinsic(activeFilterCoeff, activeFilterL, activeFilterLen);
	outputR = dot_product_intrinsic(activeFilterCoeff, activeFilterR, activeFilterLen);
#else
	for(int i = 0; i < activeFilterLen; i++){
			outputL += activeFilterCoeff[i] * activeFilterL[i];
			outputR += activeFilterCoeff[i] * activeFilterR[i];
		}
#endif


	/* write to outputs */
	*outL = (int) outputL;
	*outR = (int) outputR;

	return 1;
}
int filterTest(){
	int in, out;
	int *testFilterIn = inputTest250;
	int *testFilterOutLP = outputTest250LP;
	int *testFilterOutHP = outputTest250HP;
	for (int i = 0; i < FILTERLEN; i++) {
		in = testFilterIn[i];
		filterWrite(in, 0);
		filterApply(1, &out, NULL);
		/* check with LP filter */
		if (out != testFilterOutLP[i]){
			return -1;
		}
		filterApply(2, &out, NULL);
		/* check with HP filter */
		if (out != testFilterOutHP[i]){
			return -1;
		}
	}
	return 1;
}
