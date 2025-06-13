# Audio-Processing-and-Synth
An audio input and output processing project developed on the STM32 NUCLEO board.
This project implements a clap-activated audio synthesizer using the Nucleo board. Claps are detected using an electret microphone, ADC, and time & frequency domain analysis. Once activated, users can press keys to play sounds with 4 effects including tremolo and distortion, record up to four 10-second tracks, and play them back simultaneously. The board generates sine waves at specific frequencies, and sends the waveform information and duration via USART to a Python script which plots real-time audio graphs, plays sounds, and displays a GUI.

Frequency analysis capabilities of the board are shown with CMSIS-DSP library. Audio is captured with microphone and is analyzed. Effects are applied and can be output through I2S protocol.
