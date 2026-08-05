# ESP32_AM_FM_SSB_HFTX
ESP32 with Si5351 HAM radio TX with AM, FM, SSB modulation controlled via WiFi
This is an experimental project about how can ESP32 board together with just one Si5351 board and an BS170 mosfet work as an all mode transceiver.
The project goals were:
- Use minimal hardware
- Just generate frequencies with all modulation modes in HAM HF specter (0.4Mhz to 30Khz with 1 Hz resolution is currently achieved)
- Have initial power about 100mW and no filters (just to test operation on an HF radio for experimental work using short 30cm wire, to keep range only a few 10 meters away)

The project depends on modified-extended Si5351 library attached in this repository. It remains fully compatible with standard SI5351Arduino library. So you can copy it in Arduino/libraries folder an have it convenient to work with other Arduino Si5351 based projects.
The project is ongoing, so further details about connection and necessary components will follow.
From 5th August 2026 improved version (Ver.09) is now working with all modulation modes better for all HAM bands (Ver.08 was a first try and it worked well only in 7Mhz band.)

I was inspired for this project with USDX, a popular pocket-sized, 5-watt, 5-band multi-mode QRP amateur radio transceiver. It was originally conceived by Guido (PE1NNZ) and co-designed into the popular commercialized form by Manuel (DL2MAN). I expected ESP32 could be faster and so with improved sampling frequency it will gain the audio quality. I found out that the limiting element is not a microcontroller, but the Si5351 itself, so improvement is not dramatic.  
