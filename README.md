## Assignment Audio Labs

### Task 1, 2 and 3
audio_labs.c contains the code for all the audio labs.
#### Concerning task 1
Without changes, audio_labs.c uses the own convolution function for the filter with float buffers:
```cpp
float LP[]={coeffLP};
float HP[]={coeffHP};
float LPBufferL[N_LP] = {0};
float HPBufferL[N_HP] = {0};
float LPBufferR[N_LP] = {0};
float HPBufferR[N_HP] = {0};
```
```cpp
int filterApply(int filterType, int * outL, int * outR){
	.
    /* switch active buffer and coefficients depending on filterType
    **      1 -> LPBuffer ; 2 -> HPBuffer
    **/
    .
    /* convolution */
	for(int i = 0; i < activeFilterLen; i++){
        outputL += activeFilterCoeff[i] * activeFilterL[i];
        outputR += activeFilterCoeff[i] * activeFilterR[i];
    }
    .
    .

	return 1;
}
```

#### Concerning task 2
In order to use NEON with intrinsics `NEONINTRINSICS` has to be defined (line 385), resulting in the function changing to:
```cpp
int filterApply(int filterType, int * outL, int * outR){
	.
    /* switch active buffer and coefficients depending on filterType
    **      1 -> LPBuffer ; 2 -> HPBuffer
    **/
    .
	outputL = dot_product_intrinsic(activeFilterCoeff, activeFilterL, activeFilterLen);
	outputR = dot_product_intrinsic(activeFilterCoeff, activeFilterR, activeFilterLen);
    .
    .

	return 1;
}
```

#### Concerning Task 3
The FIR accelerator is always accessible by turning on switch 3:
```cpp
/* main */
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
```

### Task 4 and 5
A more detailed table of the measured execution times can be found in `time-measurements.xlsx`

### Task 6
The commented version of the NEON dot_product_intrinsic implementation can be found in `dot_product_intrinsic-commented`