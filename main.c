/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Frequency Analysis and I2S Output
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

#define SAMPLE_SIZE       2048  // Changed to power of 2 for FFT
#define FEATURE_SIZE      3
#define FFT_SIZE          1024  // FFT size for frequency analysis
#define I2S_BUFFER_SIZE   512   // I2S circular buffer size

#define CLAP_THRESHOLD    0.95f  
#define ENERGY_THRESHOLD  0.2f  
#define PRE_CALIBRATION_THRESHOLD 0.12f

// Audio buffers
uint16_t adc_buffer[SAMPLE_SIZE];
float audio_buffer[SAMPLE_SIZE];
float features[FEATURE_SIZE];

// FFT and frequency analysis buffers
float32_t fft_input[FFT_SIZE * 2];  // Complex input (real + imaginary)
float32_t fft_output[FFT_SIZE];     // Magnitude output
float32_t fft_magnitude[FFT_SIZE/2]; // Only positive frequencies
arm_rfft_fast_instance_f32 fft_instance;

// I2S audio output buffers
uint16_t i2s_tx_buffer[I2S_BUFFER_SIZE];
uint16_t i2s_rx_buffer[I2S_BUFFER_SIZE];
volatile uint8_t i2s_tx_half_complete = 0;
volatile uint8_t i2s_tx_complete = 0;

// Audio generation state
typedef struct {
    float phase;
    float frequency;
    float amplitude;
    uint8_t waveform_type; // 0=sine, 1=bell, 2=vibrato, 3=distorted
    uint8_t active;
} audio_oscillator_t;

audio_oscillator_t main_osc = {0};
float sample_rate = 44100.0f; // I2S sample rate

char message[100];

float clap_template[FEATURE_SIZE] = {
    0.2f,   // RMS
    5.0f,   // Crest
    0.15f   // ZCR
};

enum {
    STATE_NORMAL
} system_state = STATE_NORMAL;
uint8_t clap_detected = 0;

uint32_t last_button_press = 0;
uint32_t last_clap_detect = 0;
#define DEBOUNCE_TIME 300
#define CLAP_DEBOUNCE_TIME 500

// Keypad variables
int8_t current_row = -1, current_col = -1;
uint16_t row = 0;
int delay = 500;
int increment = 1;
int num1 = -1, num2 = -1, answer = -1;
int keypad_pressed = 0;
int debounce_time = 100;
int last_button_pressed = 0;
int key_pressed = 0;
uint8_t button = 0;
int user_flag = 1;
uint8_t waiting_for_keypress = 0;
uint8_t processed_keypad_input = 0;
char detected_key = '\0';

// Function prototypes
void extract_features(float* buffer, float* features);
int is_clap(float* features);
void calibrate_clap(void);
void process_keypad_input(int row, int col);

// Enhanced audio functions with I2S output
void init_audio_system(void);
void generate_audio_sample(float* sample);
void fill_i2s_buffer(uint16_t* buffer, uint32_t size);
void start_audio_tone(float freq, uint8_t waveform_type);
void stop_audio_tone(void);

// Frequency analysis functions
void init_frequency_analyzer(void);
void analyze_frequency_spectrum(float* input_buffer);
float find_dominant_frequency(void);
void print_frequency_analysis(void);

// Legacy compatibility functions
void send_sinewave(uint32_t Freq, uint16_t Duration, uint16_t FS);
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
    MX_I2S3_Init(); // Initialize I2S for audio output
    
    // Initialize audio system
    init_audio_system();
    init_frequency_analyzer();
    
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
    
    print_msg("Enhanced Audio Synthesizer Started\r\n");
    print_msg("Features: Frequency Analysis + I2S Output\r\n");
    print_msg("Press USER button to calibrate a clap\r\n");

    // Start I2S transmission
    HAL_I2S_Transmit_DMA(&hi2s3, i2s_tx_buffer, I2S_BUFFER_SIZE);

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
            
            if(recording==1 && button!=0){
                input[inputlength] = button-1;
                sprintf(message, "recorded %d\n",input[inputlength]);
                print_msg(message);
                
                // Start audio tone with I2S output
                if(recordingtype==4){
                    start_audio_tone(notes[input[inputlength]], 0); // sine
                }
                else if (recordingtype==3){
                    start_audio_tone(notes[input[inputlength]], 1); // bell
                }
                else if (recordingtype==2){
                    start_audio_tone(notes[input[inputlength]], 2); // vibrato
                }
                else if (recordingtype==1){
                    start_audio_tone(notes[input[inputlength]], 3); // distorted
                }
                
                HAL_Delay(500); // Play tone for 500ms
                stop_audio_tone();
                
                inputlength+=1;
                button=0;
                if(inputlength==10){
                    inputlength=0;
                    recording=0;
                    recorded=1;
                    sprintf(message, "done recording\n");
                    print_msg(message);
                    sprintf(message, "signal_sent go_piano\n");
                    print_msg(message);
                }
            }
            else if(button!=0){
                if(signal_directive==0){//main menu
                    if(button==4){//go to piano notes
                        signal_directive=4;
                        sprintf(message, "signal_sent go_piano\n");
                        print_msg(message);
                    }
                    if(button==3){//go to bell notes
                        signal_directive=3;
                        sprintf(message, "signal_sent go_bells\n");
                        print_msg(message);
                    }
                    if(button==2){//go to vibrato notes
                        signal_directive=2;
                        sprintf(message, "signal_sent go_vibrato\n");
                        print_msg(message);
                    }
                    if(button==1){//go to distorted notes
                        signal_directive=1;
                        sprintf(message, "signal_sent go_distorted\n");
                        print_msg(message);
                    }
                    if(button==13){
                        play_tracks();
                    }
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
                        for(int i=0; i<10; i++){
                            tracks[track_index][i] = input[i];
                            input[i]=0;
                        }
                        upload_track(tracks[track_index],10,4);
                        track_index = (track_index+1) % 4;
                        recorded=0;
                    }
                    else if(button==13){
                        play_tracks();
                    }
                    else if(button!=16){//plays notes
                        // Play notes with I2S output
                        if(signal_directive==4){
                            start_audio_tone(notes[button-1], 0); // sine
                        }
                        else if(signal_directive==3){
                            start_audio_tone(notes[button-1], 1); // bell
                        }
                        else if(signal_directive==2){
                            start_audio_tone(notes[button-1], 2); // vibrato
                        }
                        else if(signal_directive==1){
                            start_audio_tone(notes[button-1], 3); // distorted
                        }
                        
                        HAL_Delay(300); // Play for 300ms
                        stop_audio_tone();
                    }
                    else{//goes back to main menu
                        signal_directive=0;
                        sprintf(message, "signal_sent go_menu\n");
                        print_msg(message);
                    }
                }
                button=0; 
            }

        } else {
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
            
            
            analyze_frequency_spectrum(audio_buffer);
            float dominant_freq = find_dominant_frequency();
            
            if (dominant_freq > 100.0f) { 
                sprintf(message, "Dominant frequency: %.1f Hz\r\n", dominant_freq);
                print_msg(message);
            }
            
            extract_features(audio_buffer, features);
            
            current_tick = HAL_GetTick();
            
            if (current_tick - last_clap_detect > CLAP_DEBOUNCE_TIME) {
                if (features[0] > ENERGY_THRESHOLD) {  
                    int result = is_clap(features);
                    
                    if (result == 1) {
                        clap_detected = 1;
                        print_msg("Clap detected!\r\n");
                        print_frequency_analysis(); // Print detailed frequency analysis
                        last_clap_detect = current_tick;
                        waiting_for_keypress = 1;
                        key_pressed = 1;
                        
                        // Toggle an LED when a clap is detected
                        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                    }
                }
            }
        }
        
        
        if (i2s_tx_half_complete) {
            fill_i2s_buffer(i2s_tx_buffer, I2S_BUFFER_SIZE/2);
            i2s_tx_half_complete = 0;
        }
        
        if (i2s_tx_complete) {
            fill_i2s_buffer(&i2s_tx_buffer[I2S_BUFFER_SIZE/2], I2S_BUFFER_SIZE/2);
            i2s_tx_complete = 0;
        }
    }
}

// audio system
void init_audio_system(void)
{
    main_osc.phase = 0.0f;
    main_osc.frequency = 0.0f;
    main_osc.amplitude = 0.0f;
    main_osc.waveform_type = 0;
    main_osc.active = 0;
    
    memset(i2s_tx_buffer, 0, sizeof(i2s_tx_buffer));
    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
}

void init_frequency_analyzer(void)
{
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    
    memset(fft_input, 0, sizeof(fft_input));
    memset(fft_output, 0, sizeof(fft_output));
    memset(fft_magnitude, 0, sizeof(fft_magnitude));
}

void analyze_frequency_spectrum(float* input_buffer)
{
    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        fft_input[i] = (i < SAMPLE_SIZE) ? input_buffer[i] : 0.0f;
    }
    
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
    
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE/2);
}

float find_dominant_frequency(void)
{
    uint32_t max_index = 0;
    float max_magnitude = 0.0f;
    
    for (uint32_t i = 1; i < FFT_SIZE/2; i++) {
        if (fft_magnitude[i] > max_magnitude) {
            max_magnitude = fft_magnitude[i];
            max_index = i;
        }
    }
    
    
    float frequency = (float)max_index * sample_rate / (float)FFT_SIZE;
    
    return frequency;
}

// Printed analysis, test on logic analyzer
void print_frequency_analysis(void)
{
    sprintf(message, "=== Frequency Analysis ===\r\n");
    print_msg(message);
    
    for (int rank = 0; rank < 5; rank++) {
        uint32_t max_index = 0;
        float max_magnitude = 0.0f;
        
        for (uint32_t i = 1; i < FFT_SIZE/2; i++) {
            if (fft_magnitude[i] > max_magnitude) {
                max_magnitude = fft_magnitude[i];
                max_index = i;
            }
        }
        
        if (max_magnitude > 0.01f) { 
            float frequency = (float)max_index * sample_rate / (float)FFT_SIZE;
            sprintf(message, "Peak %d: %.1f Hz (magnitude: %.3f)\r\n", 
                    rank+1, frequency, max_magnitude);
            print_msg(message);
        }
        
        fft_magnitude[max_index] = 0.0f;
    }
    
    sprintf(message, "========================\r\n");
    print_msg(message);
}


void start_audio_tone(float freq, uint8_t waveform_type)
{
    main_osc.frequency = freq;
    main_osc.amplitude = 0.5f; 
    main_osc.waveform_type = waveform_type;
    main_osc.active = 1;
    main_osc.phase = 0.0f;
    
    sprintf(message, "Playing %.1f Hz, type %d\r\n", freq, waveform_type);
    print_msg(message);
}

void stop_audio_tone(void)
{
    main_osc.active = 0;
    main_osc.amplitude = 0.0f;
}

void generate_audio_sample(float* sample)
{
    if (!main_osc.active) {
        *sample = 0.0f;
        return;
    }
    
    float output = 0.0f;
    
    switch (main_osc.waveform_type) {
        case 0: // Sine wave
            output = sinf(main_osc.phase);
            break;
            
        case 1: // Bell (exponential decay)
            output = sinf(main_osc.phase) * expf(-main_osc.phase / (2.0f * M_PI));
            break;
            
        case 2: // Vibrato (AM modulation)
            {
                float vibrato_freq = 5.0f;
                float mod_depth = 0.3f;
                float mod = 1.0f + mod_depth * sinf(main_osc.phase * vibrato_freq / main_osc.frequency);
                output = sinf(main_osc.phase) * mod;
            }
            break;
            
        case 3: // Distorted (soft clipping)
            {
                float clean = sinf(main_osc.phase);
                float drive = 3.0f;
                output = tanhf(clean * drive) / drive;
            }
            break;
            
        default:
            output = sinf(main_osc.phase);
            break;
    }
    
    *sample = output * main_osc.amplitude;
    
    main_osc.phase += 2.0f * M_PI * main_osc.frequency / sample_rate;
    if (main_osc.phase >= 2.0f * M_PI) {
        main_osc.phase -= 2.0f * M_PI;
    }
}


void fill_i2s_buffer(uint16_t* buffer, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        float sample;
        generate_audio_sample(&sample);
        
        buffer[i] = (uint16_t)((sample + 1.0f) * 32767.5f);
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3) {
        i2s_tx_half_complete = 1;
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3) {
        i2s_tx_complete = 1;
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

void send_sinewave(uint32_t Freq, uint16_t Duration, uint16_t FS)
{
    sprintf(message, "Legacy: sending_sinewave %d %d %d\n", Freq, Duration, FS);
    print_msg(message);
    print_msg("sinewave_complete\n");
}

void send_bell(uint16_t Freq, uint16_t Duration, uint16_t FS)
{
    sprintf(message, "Legacy: sending_bell %d %d %d\n", Freq, Duration, FS);
    print_msg(message);
    sprintf(message, "bell_complete %d\n", FS);
    print_msg(message);
}

void play_tracks(void)
{
    sprintf(message, "play_tracks\n");
    print_msg(message);
    HAL_Delay(10000);
    sprintf(message, "track_played\n");
    print_msg(message);
}

void upload_track(uint8_t * array, uint8_t size, uint8_t type)
{
    sprintf(message, "start_uploading_track %d\n", type);
    print_msg(message);
    for(int i=0; i<10; i++){
        if(type==4){
            send_sinewave(notes[array[i]], 1, 5000);
        }
        else if(type==3){
            send_bell(notes[array[i]], 1, 5000);
        }
    }    
    sprintf(message, "done_uploading_track\n");
    print_msg(message);
}

void send_sinwave_vibrato(uint16_t Freq, uint16_t Duration, uint16_t FS)
{
    sprintf(message, "Legacy: sending_vibratosinewave %d %d %d\n", Freq, Duration, FS);
    print_msg(message);
    print_msg("vibratosinewave_completed\n");
}

void send_sinewave_distorted(uint16_t Freq, uint16_t Duration, uint16_t FS)
{
    sprintf(message, "Legacy: sending_distortedsinewave %d %d %d\n", Freq, Duration, FS);
    print_msg(message);
    print_msg("distortedsinewave_complete\n");
}