/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"

#define M_PI 3.14159265358979323846f

#define SAMPLE_SIZE       2056
#define FEATURE_SIZE      3

#define CLAP_THRESHOLD    0.95f  
#define ENERGY_THRESHOLD  0.2f  
#define PRE_CALIBRATION_THRESHOLD 0.12f

uint16_t adc_buffer[SAMPLE_SIZE];
float audio_buffer[SAMPLE_SIZE];
float features[FEATURE_SIZE];

char message[100];

float clap_template[FEATURE_SIZE] = {
    0.2f,   // RMS
    5.0f,   // Crest
    0.15f   // ZCR
};

//uint8_t clap_calibrated = 1;

enum {
    STATE_NORMAL
} system_state = STATE_NORMAL;
uint8_t clap_detected = 0;

uint32_t last_button_press = 0;
uint32_t last_clap_detect = 0;
#define DEBOUNCE_TIME 300
#define CLAP_DEBOUNCE_TIME 500

//keypad variables
int8_t current_row = -1, current_col = -1;
uint16_t row = 0;
int delay = 500;
int increment = 1;
int num1 = -1, num2 = -1, answer = -1;
int keypad_pressed = 0;
int debounce_time = 100;
int last_button_pressed = 0;
int key_pressed = 0;
uint8_t button =0;
int user_flag = 1;
uint8_t waiting_for_keypress = 0;
uint8_t processed_keypad_input = 0;
char detected_key = '\0';

// Function prototypes
void extract_features(float* buffer, float* features);
int is_clap(float* features);
void calibrate_clap(void);
void process_keypad_input(int row, int col);
void send_sinewave(uint32_t Freq, uint16_t Duration,uint16_t FS);
void play_tracks(void);
void send_bell(uint16_t Freq, uint16_t Duration, uint16_t FS);
void send_sinwave_vibrato(uint16_t Freq, uint16_t Duration, uint16_t FS);
void send_sinewave_distorted(uint16_t Freq, uint16_t Duration, uint16_t FS);

void upload_track(uint8_t * array, uint8_t size, uint8_t type);
uint32_t notes[16] = {0, 277,294, 311, 330, 349,370,392,415,440,466,493,523,554,587,622};
uint8_t tracks[4][10] = {0};

int main(void)
{
    /* Reset of all peripherals */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC3_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_DAC_Init();
		  /* Initialize ROW outputs */
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
	GPIO_PinState state[4] = {GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET};
	
	uint8_t signal_directive = 0;
	//go_piano corresponds to 4
	//go_vibrato corresponds to 2
	//go_bells corresponds to 3
	//go_distorted corresponds to 1
	//go_menu corresponds to 0
	
	uint8_t input[10]={0};
	uint8_t track_index=0;
	uint8_t inputlength =0;
	uint8_t recorded=0;
	uint8_t recording=0;
	uint8_t recordingtype=0;

	const int transient_delay = 50;
	const int debounce_delay = 50;
    print_msg("Clap Detector Started\r\n");
    print_msg("Press USER button to calibrate a clap\r\n");

				while (1)
		{
				
				 
				if (clap_detected) {
            
					state[row] = GPIO_PIN_RESET;
		row = (row+1)%4;
		state[row] = GPIO_PIN_SET;
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, state[0]);
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, state[1]);
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, state[2]);
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, state[3]);
		HAL_Delay(50);
		if(recording==1&&button!=0){
				input[inputlength] = button-1;
				sprintf(message, "recorded %d\n",input[inputlength]);
				print_msg(message);
				if(recordingtype==4){
					send_sinewave(notes[input[inputlength]],1,5000);
				}
				else if (recordingtype==3){
					send_bell(notes[input[inputlength]],1,5000);
				}
				else if (recordingtype==2){
					send_sinwave_vibrato(notes[input[inputlength]],1,5000);
				}
				else if (recordingtype==1){
					send_sinewave_distorted(notes[input[inputlength]],1,5000);
				}
				inputlength+=1;
				button=0;
				if(inputlength==10){
					inputlength=0;
					recording=0;
					recorded=1;
					sprintf(message, "done recording\n");
					print_msg(message);
					sprintf(message, "signal_sent go_piano\n");//go piano has the same background as the others!
					print_msg(message);
				}
		}
		else if(button!=0){
			if(signal_directive==0){//main menu
				if(button==4){//go to piano notes
					signal_directive=4;//go to piano
					sprintf(message, "signal_sent go_piano\n");
					print_msg(message);
				}
				if(button==3){//go to bell notes
					signal_directive=3;//go to bells
					sprintf(message, "signal_sent go_bells\n");
					print_msg(message);
				}
				if(button==2){//go to vibrato notes
					signal_directive=2;//go to vibrato
					sprintf(message, "signal_sent go_vibrato\n");
					print_msg(message);
				}
				if(button==1){//go to distorted notes
					signal_directive=1;//go to distorted
					sprintf(message, "signal_sent go_distorted\n");
					print_msg(message);
				}
				if(button==13){
					play_tracks();
				}
				//else{
					//send_bell(notes[button-1],1,5000);
				//}
			}
			else if(signal_directive>0){//in the piano directive
					if(button==14){
						recorded=0;
						recording=1;
						for(int i=0; i<10;i++){
							input[i]=0;
						}
						sprintf(message, "signal_sent go_notes\n");
						print_msg(message);
						sprintf(message, "started recording\n");
						print_msg(message);
						inputlength=0;
						recordingtype=signal_directive;
					}
					
					else if(button==15&&recorded==1){
						//input is which notes are being pressed, 10 is # of notes, and 4 is the type(piano)
						
						for(int i=0; i<10; i++){
							tracks[track_index][i] = input[i];
							input[i]=0;
						}
						upload_track(tracks[track_index],10,4);
						track_index = (track_index+1) % 4;
						recorded=0;
							//recorded=0;
					}
					else if(button==13){
						play_tracks();
					}
					else if(button!=16){//plays notes
						if(signal_directive==4){
							send_sinewave(notes[button-1],1,5000);
						}
						else if(signal_directive==3){
							send_bell(notes[button-1],1,5000);
						}
						else if(signal_directive==2){
							send_sinwave_vibrato(notes[button-1],1,5000);
						}
						else if(signal_directive==1){
							send_sinewave_distorted(notes[button-1],1,5000);
						}
						
					}
					else{//goes back to main menu
						signal_directive=0;
						sprintf(message, "signal_sent go_menu\n");
						print_msg(message);
					}
			}
			button=0; 
		}
				
        

    }else {
			uint32_t current_tick = HAL_GetTick();
				
				if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) && 
						(current_tick - last_button_press > DEBOUNCE_TIME)) {
						
						last_button_press = current_tick;
						HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
						print_msg("Button pressed. LED toggled.\r\n");
				}
				
				for (uint32_t i = 0; i < SAMPLE_SIZE; i++)
				{
						HAL_ADC_Start(&hadc3);
						HAL_ADC_PollForConversion(&hadc3, 10);
						adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
						
						audio_buffer[i] = ((float)adc_buffer[i] - 2048.0f) / 2048.0f;
				}
				
				extract_features(audio_buffer, features);
				
				current_tick = HAL_GetTick();
				
				if (current_tick - last_clap_detect > CLAP_DEBOUNCE_TIME) {
						if (features[0] > ENERGY_THRESHOLD) {  
								int result = is_clap(features);
								
								if (result == 1) {
										clap_detected = 1;
										print_msg("Clap detected!\r\n");
										last_clap_detect = current_tick;
										waiting_for_keypress = 1;
										key_pressed = 1;
									/*
										for(int i = 0; i < SAMPLE_SIZE; i++) {
											sprintf(message, "%d \r\n", adc_buffer[i]);
											print_msg(message);
										}*/
										// Toggle an LED when a clap is detected
										HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
								}
						}
				}
		}
				
				
				
				//HAL_Delay(50); // Short delay between processing cycles
			}
				
				
		
}

void extract_features(float* buffer, float* features)
{
    // RMS
    float sum = 0.0f;
		float peak = 0.0f;
    for (uint32_t i = 0; i < SAMPLE_SIZE; i++)
    {
        sum += buffer[i] * buffer[i];
				if (fabsf(buffer[i]) > peak) {
							peak = fabsf(buffer[i]);
				}
    }
    features[0] = sqrtf(sum / SAMPLE_SIZE);
    
    // Crest factor
    features[1] = (features[0] > 0.01f) ? peak / features[0] : 0.0f; 
        
    // Zero crossing rate
    int zero_crossings = 0;
    for (uint32_t i = 1; i < SAMPLE_SIZE; i++) {
        if ((buffer[i-1] > 0 && buffer[i] < 0) || (buffer[i-1] < 0 && buffer[i] > 0)) {
            zero_crossings++;
        }
    }
    features[2] = (float)zero_crossings / SAMPLE_SIZE;
}


int is_clap(float* features)
{

    sprintf(message, "Features: Energy=%.3f, Crest=%.3f, ZCR=%.3f\r\n", 
            features[0], features[1], features[2]);
    print_msg(message);
    
	//tune
    int energy_match = (features[0] >= 0.30f);
    int crest_match = 0;
    int zcr_match = 0;

	int match_count = 0;
	if (energy_match) {
		int crest_match = (features[1] >= 1.5f);
		int zcr_match = (features[2] >= 0.005f);
		
		match_count = energy_match + crest_match + zcr_match;
	}
	if (match_count == 3) {
		return 1;
	}
	else {
		return 0;
	}
}


void send_sinewave(uint32_t Freq, uint16_t Duration,uint16_t FS){
	//uint16_t Freq = 440;
	//uint16_t FS = 5000;
	//format is "sinewave" to indicate a sinewave, 440 to indicate frequency of the sine wave
	//2 is the duration in seconds
	//5000 is the sampling frequency to detect it with
	sprintf(message, "sending_sinewave %d %d %d\n",Freq,Duration, FS);
	print_msg(message);
	if(Freq!=0){
		float increment_value = 2.0*M_PI *Freq*1.0/(FS*1.0);
		for(float i=0; i<M_PI*2; i+=increment_value){
			uint16_t value = (uint16_t) ((sin(i)+1.0)*2048.0);		
			sprintf(message, "%d\n",value);
			print_msg(message);
		}
	}
	print_msg("sinewave_complete\n");
}

void send_bell(uint16_t Freq, uint16_t Duration, uint16_t FS){
	sprintf(message, "sending_bell %d %d %d\n", Freq, Duration, FS);
	print_msg(message);
	if(Freq!=0){
		uint16_t samples = FS/Freq;
		for(uint16_t i=samples-1; i>0; i--){
			uint16_t sample = (uint16_t) (2048+(float)(2048.0/(1.0*(float)(samples))*i));
			sprintf(message, "%d\n",sample);
			print_msg(message);
		}
	}
	sprintf(message, "bell_complete %d\n", FS);
	print_msg(message);
}
void play_tracks(void){
	sprintf(message, "play_tracks\n");
	print_msg(message);
	HAL_Delay(10000);
	sprintf(message, "track_played\n");
	print_msg(message);
}
void upload_track(uint8_t * array, uint8_t size, uint8_t type){
	
	sprintf(message, "start_uploading_track %d\n",type);
	print_msg(message);
	for(int i=0; i<10; i++){
		if(type==4){
			send_sinewave(notes[array[i]],1,5000);
		}
		else if(type==3){
			send_bell(notes[array[i]],1,5000);
		}
	}	
	sprintf(message, "done_uploading_track\n");
	print_msg(message);
}

void send_sinwave_vibrato(uint16_t Freq, uint16_t Duration, uint16_t FS){
	sprintf(message, "sending_vibratosinewave %d %d %d\n",Freq,Duration,FS);
	print_msg(message);
	if(Freq!=0){
		uint32_t total_samples = Duration*FS;
		float LFO_freq = 5.0f;
		float tremolo_depth = 0.5f;
		for(uint32_t n=0; n<total_samples; n++){
			float t = (float)n / FS;
			float sample = sinf(2.0f*M_PI*Freq *t);
			float envelope  = (1.0f - tremolo_depth)+tremolo_depth * ((sinf(2.0f *M_PI *LFO_freq * t) + 1.0f)/2.0f);
			float modulated_sample = sample*envelope;

			uint16_t value = (uint16_t)((modulated_sample+1.0f)*2048.0f);
			sprintf(message, "%d\n",value);
			print_msg(message);
		}
	}
	print_msg("vibratosinewave_completed\n");
}


void send_sinewave_distorted(uint16_t Freq, uint16_t Duration, uint16_t FS){ 
    //uint16_t Freq = 440; 
    //uint16_t FS = 5000; 
    //format is "sinewave" to indicate a sinewave, 440 to indicate frequency of the sine wave
    //2 is the duration in seconds
    //5000 is the sampling frequency to detect it with
    
    sprintf(message, "sending_distortedsinewave %d %d %d\n", Freq, Duration, FS); 
    print_msg(message); 
    
    if(Freq!=1){
			float drive = 3.0f;       
			float threshold = 0.6f;  
    
			float increment_value = 2.0 * M_PI * Freq * 1.0 / (FS * 1.0);
			float total_samples = Duration * FS; 
			float cycles_needed = Freq * Duration; 
    
			float max_radians = 2.0 * M_PI * cycles_needed;
    
			for(float i = 0; i < max_radians; i += increment_value) {
        // Generate basic sine wave
        float sample = sin(i);
        
        if(sample > threshold) {
           sample = threshold + (sample - threshold) / drive;
        }else if(sample < -threshold) {
           sample = -threshold + (sample + threshold) / drive;
        }
        
        uint16_t value = (uint16_t)((sample + 1.0) * 2048.0); 
        
        sprintf(message, "%d\n", value); 
        print_msg(message); 
			}
		}
    print_msg("distortedsinewave_complete\n"); 
}