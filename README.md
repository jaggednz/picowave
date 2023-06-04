# WARNING: Work in Progress

This Repository is a work in progress and may not build or compile. It does not yet contain key information such as schematic diagrams or hardware information and makes no garantees about the fitness or quality of the code.

# PicoWave2040
PicoWave2040 hybrid polyphonic wavetable synth.

Project aim is to create a low part count Midi & USB Midi controllable, low part count, hybrid polyphonic synthisizer.
To acheive this many shortcuts will be taken! The aim is not to create the highest possible fidality, but rather the best fidality within the hardware limitations. 

It uses 4 hardware PWM slices to generate 8 individual 12-bit voices at ~32Khz.
Seperate PWM slices and some demuxing will be used to implement 8 individual VCA and VCF cutoff signals.
VCF Resonance is not planned to be controlled via the Microcontroller, but this could change (PIO PWM could be used?)
VCA and VCF designs are yet to be prototyped or finallised. VCA is likely to be LM13700 based.

The raw PWM signal from the Pico has a lot of digital noise, therefore it is best to isolate the PWM signal via octal buffer(s) e.g. SN74F245 powered via a seperatly regulated supply.

Wavetables are currently copied from Jacajacks uSynth and sourced from the original PPG eproms. However I intend to use littleFS to replace these with WAV format wavetables from WaveEdit software. These will be naively downconverted from 16-bit to 12-bit.
