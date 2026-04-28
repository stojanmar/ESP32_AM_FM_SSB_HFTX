# ESP32_AM_FM_SSB_HFTX
ESP32 with Si5351 HAM radio TX with AM, FM, SSB modulation controlled via WiFi
This is an experimental project about how can ESP32 board together with just one Si5351 board work as an all mode transceiver.
The project goals were:
- Use minimal hardware
- Just generate frequencies with all modulation modes in HAM HF specter (0.4Mhz to 30Khz with 1 Hz resolution is currently achieved)
- Have initial power about 100mW and no filters (just to test operation on an HF radio for experimental work using short 30cm wire, to keep range only a few 10 meters away)

The project depends on modified-extended Si5351 lybrary attached in thi repository. It remains fully compatible with standard SI5351Arduino library. So you can copy it in Arduino/libraries folder an have it convenient to work with other Arduino Si5351 based projects.
The project is ongoing, so further details about connection and necessary components will follow.
