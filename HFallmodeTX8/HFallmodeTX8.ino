/*Za kontrolo VFO TX preko WiFija
 * dela na esp32 dev module
 * var01 5.1.23 try to establish AM modulation first
 * created by S52UV
 * To control si5351 as transmiter for all HAM modulations
 * Vsebuje ze intrupt testno en pin vsako 1 sekundo.
 * dodam se wifi vnos gaina in moči na dac2
 * ver08 predelam v fast adc iz simplefoc ne dela dobro prekinja dac delovanje
 * naredim z interuptom ver10
 * ver upl2 umaknem interrupt v loop
 * ver ulp3 uvedem fm po principu usdx
 * ver ulp4 fm s CORRECTION na Si5351
 * ver ulp5 popravim obseg FM popacenje
 * ver ulp5a korigiram napako
 * ver ulp5b eliminiram statični IP
 * HFallmodeTX ver01 dodam SSB
 * Ver02 skaliranje izračuna magnitude za envelope
 * Ver03 prilagojeno za 4,8kHz sampling za SSB in po Redcorner USDX oudiolinija ne dela
 * Ver04 uredim audio linijo
 * Ver05 uredim Hilbert oz. filter
 * Ver05a uvedem dc decupling se v FM
 * Ver05b uvedem kontrol over delay fazna zamaknitev vzorca za DAC
 * Ver05c phase shift ready for LSB + usdx ssb process s 15 koeficienti
 * Ver05d za LSB poiskusim z optimiranjem amplitude tudi po delnem shiftu
 * Ver05e nazaj brez shifta amplitude in uvedem treshold pri ssb
 * Ver05f treshold ukrepa na pin27 open drain na gate od BS170 preko R=1K
 * Ver07 pohitrena verzija funkcij za Si5351
 * Ver07a intervali za AM 12kHz LSB/USB 5,8kHz in FM  6 kHz + novi LPF filtri
 * Ver07b brez filtra za SSB in zmanjšam sampling na 5,4khz  in 5,7kHz za FM
 * Ver07c poskusim uvest clk1 on/off for low amplitude pri ssb prepocasi zato raje dif stran od frekvence
 * Ver07d namesto dif stran, raje uvedem mute pin
 * Ver08 clean some comments and WiFi credentials
 */
 /********************************************************************
   FAST Si5351 correction update for ESP32 SSB/FM generator
   Replaces slow set_correction2()
   Idea:
   - Precompute PLL register bytes for correction range once
   - Runtime only table lookup + I2C write
   Suggested range:
   -3000 Hz ... +3000 Hz   => 6001 entries
   Use one table for each mode if needed:
   USB / LSB / FM  USB/LSB became identical at the end
********************************************************************/

#include <Arduino.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include "driver/adc.h"
#include "driver/dac.h"
#include <soc/rtc.h>
#include "driver/gpio.h"
#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <ArduinoOTA.h>
#include <si5351.h>
#include <Wire.h>  //defaut sta pina 21 SDA in 22 SCL

#define FS_TX     5000      // audio sample rate
#define UA        16384     // unit angle (2π = 16384)
//#define MAX_DP    (UA/6)    // limit phase jumps
//#define MAX_DP  ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define MAX_DP  _UA/2
#define SI5351_REF     25000000UL  //change this to the frequency of the crystal on your si5351’s PCB

//#define BAUD_RATE 9600
#ifndef STASSID
#define STASSID "xxxxxxxxx"
#define STAPSK  "xxxxxxxxx"
#endif
//#define DELAY_BETWEEN_COMMANDS 1000
#define CORRECTION 115555 // See how to calibrate your Si5351A (0 if you do not want).162962 je dalo 1280 Hz prenizko
#define SCOPE_PIN 14 //18
#define MUTE_PIN 27 //zapira gate od BS170 z open drain konfiguracijo preko 1K
#define magn(i, q) (abs(i) > abs(q) ? abs(i) + (abs(q) / 4) : abs(q) + (abs(i) / 4)) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB
#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#define _F_SAMP_TX  5750 //4800
#define MORE_MIC_GAIN   1       // 1.02w adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#define PHASEBUF 8
/*#define DIF_MIN   -3000  //prenešeno v Si5351 library
#define DIF_MAX    3000
#define DIF_SIZE  (DIF_MAX - DIF_MIN + 1)*/
// The Si5351 instance.
Si5351 si5351;

const char* ssid = STASSID;
const char* password = STAPSK;

const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "input4";
const char* PARAM_INPUT_5 = "input5";
const char* PARAM_INPUT_6 = "input6";
const char* PARAM_INPUT_7 = "input7";
const char* PARAM_INPUT_8 = "input8";

const int bufferSize = 1024;
const int bufferOffset = 1024;
const int indexOffset = bufferOffset - 1;
const int samplingRate = 20000;

volatile int dacout = 0;
volatile int txmode = 3;  //1 LSB, 2 USB, 3 AM, 4 FM .....
//String izhodonoff = "Off"; // On or Off
//String inString = "";    // string to hold input
int offsetadc = 0;
int dcoffset = 127;  // offset na dac izhodu za power level
int powerdac = 0;   //kot dodatni offset za povečat dac izhod v + ali -
int fmbase = 50; //bias napetost v FM načinu na DAC1 ali DAC2?
volatile int dif = 0;
volatile int currentIndex = 0;
volatile int indexphase = 0;
volatile int phaseofst = 0;
volatile int32_t cal_factor = 0;  //uporabim za FM
volatile float mojgain = 1.0;
volatile float zacasna;
volatile float napaka;
volatile int16_t adc;
volatile int16_t df;
volatile uint16_t mag;
volatile uint16_t ssbtrsh = 20;
volatile int16_t I_last, Q_last;
volatile uint8_t drive = 1;  // 2;   // kako močno modulira audio path
volatile uint8_t tx = 1;  // lahko je 0,1, ali 255
volatile uint16_t amp;
volatile uint16_t hamp;  //namesto _amp
//7 tap Hilbert FIR start simple
#define HLEN 7
int16_t hbuf[HLEN];
uint16_t amplit[PHASEBUF];
int16_t dfphase[PHASEBUF];
volatile int anvhod34;
volatile int anvhod34sh;
volatile int micoffset = 1348;  // odvisno od srednje napetosti iz mikrofona
volatile int dacvalue = 0; // vrednost za na DAC izhod
//int frekadd = 0;
unsigned long timenow = 0;
unsigned long timedif = 0;
unsigned long previousMicros = 0UL;
unsigned long previouser = 0UL;
unsigned long interval = 84UL; //12kHz  za AM
volatile double frekvin = 7123000; //27125000; // 1xHz
//volatile double frekvdf = 7123000; //27125000; // 1xHz
volatile double frekvinfm;
double stepin = 1000; // za 1kHz

char buf1[10] = "27125000";
static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
//#ifdef QCX
static uint8_t pwm_max = 255;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;
//#else
//static uint8_t pwm_max = 160;  // PWM value for which PA reaches its maximum:  128 for biasing BS170 directly, 160 for IRFI510G
//#endif

uint8_t lut[256];
//uint8_t dacout = 0;
String strgain = "1.00";
String frekvenca = "200000000";  //do 200Mhz gre
String stepinc = "1000000"; //bi bil increment 1Mhz oz. 250Hz pri SDR
String strpowerdac = "255";
String SSBphase = "0";
String strssbtrsh = "255";

//SSB related
#define CORDIC_ITER 15

static const int16_t cordic_atan[CORDIC_ITER] = {
    4096, 2418, 1273, 647, 325, 163, 81, 41,
    20, 10, 5, 3, 1, 1, 0
};

void build_lut()
{
    for (uint16_t i = 0; i < 256; i++) {
        lut[i] = (uint8_t)(
            ((uint32_t)i * (uint32_t)(pwm_max - pwm_min)) / 255
            + pwm_min
        );
    }
}

int16_t cordic_atan2(int16_t y, int16_t x)
{
    int32_t xi = x;
    int32_t yi = y;
    int32_t zi = 0;

    if (xi < 0) {
        xi = -xi;
        yi = -yi;
        zi = (y >= 0) ? (UA/2) : -(UA/2);
    }

    for (int i = 0; i < CORDIC_ITER; i++) {
        int32_t x_new;
        if (yi > 0) {
            x_new = xi + (yi >> i);
            yi = yi - (xi >> i);
            zi += cordic_atan[i];
        } else {
            x_new = xi - (yi >> i);
            yi = yi + (xi >> i);
            zi -= cordic_atan[i];
        }
        xi = x_new;
    }
    return zi;
}

inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
	int16_t r;
	if (abs(q) > abs(i))
		r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
	else
		r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
	r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
	return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

int16_t hilbert_q(void)
{
     return (
        -2*hbuf[0]
        +9*hbuf[2]
        -9*hbuf[4]
        +2*hbuf[6]
    ) >> 4;  // for 5kHz sampling

}
//int16_t I = hbuf[HLEN/2];
//int16_t Q = hilbert_q();
//Core SSB function po vzoru redcorner usdx
int16_t ssbusdx(int16_t in)
{
	static int16_t dc, z1;

	int16_t i, q;
	uint8_t j;
	static int16_t v[16];
	for (int j = 0; j < 15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
	//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
#ifdef DIG_MODE
	int16_t ac = in;
	dc = (ac + (7) * dc) / (7 + 1);  // hpf: slow average
	v[15] = (ac - dc) / 2;           // hpf (dc decoupling)  (-6dB gain to compensate for DC-noise)
#else
	int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
	ac = ac + z1;                    // lpf
	z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
	dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
	v[15] = (ac - dc);               // hpf (dc decoupling)
#endif //DIG_MODE
	i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
	q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
  
  uint16_t _amp = magn(i, q) >> 2;  //ali je to isto kot ta spodaj?
	//uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
	//dc += (in - dc) / 2;       // fast moving average
	dc = (in + dc) / 2;        // average
	int16_t ac = (in - dc);   // DC decoupling
	//v[15] = ac;// - z1;        // high-pass (emphasis) filter
	v[15] = (ac + z1);// / 2;           // low-pass filter with notch at Fs/2
	z1 = ac;

	i = v[7];
	q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

	uint16_t _amp = magn(i, q);
#endif  // MORE_MIC_GAIN
	_amp = _amp << (drive);
	_amp = (_amp > 255) ? 255 : _amp; // clip to max
	amp = (tx) ? lut[_amp] : 0;

	static int16_t prev_phase;
	int16_t phase = arctan3(q, i);

	int16_t dp = phase - prev_phase;  // phase difference and restriction
	//dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
	prev_phase = phase;

	if (dp < 0)
		dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband


#ifdef MAX_DP
	if (dp > MAX_DP) { // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
		prev_phase = phase - (dp - MAX_DP);  // substract restdp
		dp = MAX_DP;
	}
#endif
	if (txmode == 1) //LSB
		//return dp * (_F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
    return dp * (-_F_SAMP_TX / _UA);
	else if (txmode == 2)  //USB
		//return dp * (-_F_SAMP_TX / _UA);
    return dp * (_F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
}


int16_t ssb_process(int16_t audio)  // ni uporabljena
{
    static int16_t prev_phase = 0;

    // shift FIR buffer
    memmove(&hbuf[0], &hbuf[1], (HLEN-1)*sizeof(int16_t));
    hbuf[HLEN-1] = audio;

    /*int16_t I = hbuf[HLEN/2];
    int16_t Q = hilbert_q();*/
    I_last = hbuf[HLEN/2];
    Q_last = hilbert_q();

    int16_t phase = cordic_atan2(Q_last, I_last);
    int16_t dp = phase - prev_phase;
    prev_phase = phase;

    if (dp < 0) dp += UA;
    if (dp > MAX_DP) dp = MAX_DP;

    return (dp * FS_TX) / UA;  // frequency deviation in Hz
}

static inline uint16_t magnitude_scaled(int16_t i, int16_t q)
{
    i = abs(i);
    q = abs(q);

    // approx sqrt(i^2 + q^2)
    uint16_t mag = (i > q) ? (i + (q >> 1)) : (q + (i >> 1));

    // SCALE DOWN (this is the missing step)
    mag >>= 2;   // ÷4 → adjust later

    if (mag > 255) mag = 255;
    return mag;
}

void output_envelope(uint16_t mag)
{
    if (mag > 255) mag = 255;
    /*if (mag > 220)
    mag = 220 + ((mag - 220) >> 2);*/ //a soft envelope limiter (recommended)
    //dacWrite(25, mag);
    dac_output_voltage(DAC_CHANNEL_1, mag);
}

// DC blocker for Fs = 5 kHz
static int16_t dc = 0;
static inline int16_t dc_block(int16_t x)
{
    dc += (x - dc) >> 4;   // fc ≈ 50–80 Hz @ 5 kHz Fs
    return x - dc;
}

static int32_t dc_am = 0;
static inline int16_t dc_block_am(int16_t x)
{
    dc_am += ((int32_t)x - (dc_am >> 8));
    return x - (dc_am >> 8);
}

String readfrekvenca() {
  //int webfrekvenca = frekvin / 4;  // nazaj v readable frekvenco
    //Serial.println(frekvin); 
  return String(frekvin);
}

// HTML web page to handle 3 input fields (input1, input2, input3), 5 inputs
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Esp32 remote VFO</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <style>
body {
   background-color: #bb8899;
   font-family: Arial, Helvetica, sans-serif;
}
h1 {
   color: navy;
   margin-left: 20px;
   font-size: 150
}
meter {
  width: 240px;
  height: 25px; 
}
input[type=submit] {
  width: 80px;
  height: 33px;
  background-color: #128F76;
  border: none;
  color: white;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 4px 2px;
  cursor: pointer;
  border-radius: 5px;
}
input[type="text"] {
   margin: 0;
   height: 28px;
   background: white;
   font-size: 16px;
   width: 120px;
   appearance: none;
   box-shadow: none;
   border-radius: 5px;
   -webkit-border-radius: 5px;
   -moz-border-radius: 5px;
   -webkit-appearance: none;
   border: 1px solid black;
   border-radius: 10px;
}
</style>
<center>
  <h1>**VFO Allm TX v08**</h1>
  <p>Augain: <meter  max="100" value="50" align="left"></meter></p>
  <form action="/get">
    Frekvenca : <input type="text" name="input1">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    Stepincdec: <input type="text" name="input2" value="1000">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    VFO power: <input type="text" name="input3" value="0">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    SSBtresh+: <input type="text" name="input4" value="20">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    VFOon/off: <input type="text" name="input5" value="On">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    Modulation: <input type="text" name="input6" value="A">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    Augainf 1.0 <input type="text" name="input7" value="1.0">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    SSB phase: <input type="text" name="input8" value="0">
    <input type="submit" value="Submit">
  </form>
   <br>
   <span id="resultstr">%FREKVENCA%</span>
   <input type="text" width="200px" size="31" id="resultstr" placeholder="frekvenca v Hz"><br>
   <p>Freq and step in Hz, Step = Up or Down</p>
   <p>Stran kreirana Apr 2026 : Stojan Markic </p><br>
  </center>
</body>
<script>
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("resultstr").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/resultstr", true);
  xhttp.send();
}, 2000) ;
</script>
</html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
AsyncWebServer server(80);

String processor(const String& var){
  //Serial.println(var);
  if(var == "FREKVENCA"){
    return readfrekvenca();
  }
  return String();
}

void startULPADC()
{
  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

  //initialize DAC
  adc1_config_width(ADC_WIDTH_BIT_9);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  //pin34
  adc1_ulp_enable();

  int loopCycles = 90;
  //Serial.print("Real RTC clock: ");
  //Serial.println(rtc_fast_freq_hz);
  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;
  
  const ulp_insn_t adcProg[] = {
  I_MOVI(R0, 0),  //immediately 0 in R0, R0=0
  I_MOVI(R3, 0),  //R3=0
  I_DELAY(dt),                // 6 + dt
  I_ADC(R1, 0, 6), // 0 pomeni ADC1  1 bi bilo ADC2  in 6 pomeni pin34 store to R1
  //store sample
  I_ST(R1, R0, bufferOffset), // 8 spravi R1 v memory naslov R0+bufferOffset
  //increment addres
  I_ADDI(R0, R0, 1),          // 6 inkrement R0
  I_ANDI(R0, R0, 1023),       // 6 anda R0 to be 1023 max
  //store index
  I_ST(R0, R3, indexOffset),  // 8 spravi R0-inkrementiran R0 v naslov R3+1023
  //jump to start
  I_BXI(2)};                  // 4 jump to second instruction

  size_t load_addr = 0;
  size_t size = sizeof(adcProg)/ sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, adcProg, &size);
//  this is how to get the opcodes
//  for(int i = 0; i < size; i++)
//    Serial.println(RTC_SLOW_MEM[i], HEX);
    
  //start
  RTC_SLOW_MEM[indexOffset] = 0;
  ulp_run(0);  
}

void setup() {
  digitalWrite(MUTE_PIN, HIGH);
  pinMode(MUTE_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(MUTE_PIN, HIGH);  //Put in high impedance

  pinMode(SCOPE_PIN, OUTPUT);
  digitalWrite(SCOPE_PIN, LOW);
  Serial.begin(115200);
  delay(800);
  Serial.println("Setup process.");
  delay(100); //da se napetosti stabilizirajo
  /*adcTimer = timerBegin(3, 80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer, 100, true); // Interrupts when counter == 45, i.e. 22.222 times a second
  timerAlarmEnable(adcTimer);*/
  // Enable DAC1 Channel's Output
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  
  Wire.begin(32, 33, 800000);  // DAT,CLK Fast mode ali Fm+ mode do 800000 1mhz ne dela sigurno
  //analogValue = analogRead(34);  // samo zato da nastavi pin za ADC funkcijo
  Serial.println("Setup WiFi");
  
  wifiMulti.addAP("xxxxxxxx", password);   // add Wi-Fi networks you want to connect to
  wifiMulti.addAP("xxxxxxxx", password);
  //wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");
  Serial.println("Connecting");
  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(800);
    Serial.print('.');
  }
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer

    if (MDNS.begin("HFallTX")) {
    Serial.println("MDNS Responder Started");
  }

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/resultstr", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readfrekvenca().c_str());
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      frekvenca = inputMessage;
      frekvin = frekvenca.toInt(); // morda funkcija atol() tudi dela
        if (txmode == 4) {
        napaka = frekvin * 0.00037f + 12.5093f;
        frekvinfm = frekvin + (double)napaka;
        //frekvinfm = frekvin;
        si5351.set_freq(frekvinfm * 100, SI5351_CLK1);
        si5351.initFastPLLtables(CORRECTION);
        }
        else if (txmode == 3) {
        si5351.set_freq(frekvin * 100, SI5351_CLK1);
        //si5351.initFastPLLtables(CORRECTION);
        }
        else if (txmode == 2) {
        si5351.set_freq(frekvin * 100, SI5351_CLK1);
        si5351.initFastPLLtables(CORRECTION);
        }
        else if (txmode == 1) {
        si5351.set_freq(frekvin * 100, SI5351_CLK1);
        si5351.initFastPLLtables(CORRECTION);
        }  
              
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
      stepinc = inputMessage;
      stepin = stepinc.toInt();
            
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3;
      strpowerdac = inputMessage;
      powerdac = strpowerdac.toInt();
      powerdac = constrain(powerdac, 0, 255);
      //dacWrite(26, powerdac);
      dac_output_voltage(DAC_CHANNEL_2, powerdac);          
    }
    // GET input4 value on <ESP_IP>/get?input4=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_4)) {
      inputMessage = request->getParam(PARAM_INPUT_4)->value();
      inputParam = PARAM_INPUT_4;
      strssbtrsh = inputMessage;
      ssbtrsh = strssbtrsh.toInt();
      ssbtrsh = constrain(ssbtrsh, 0, 255);

    }
    // GET input5 value on <ESP_IP>/get?input5=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_5)) {
      inputMessage = request->getParam(PARAM_INPUT_5)->value();
      inputParam = PARAM_INPUT_5;
      if (inputMessage == "Off") {
        powerdac = 0;
        si5351.output_enable(SI5351_CLK1, 0);  // ugasnem CLK1     
      } // Si5351 CLK ugasnjen
      if (inputMessage == "On") {
        si5351.output_enable(SI5351_CLK1, 1);  // vklopim CLK1
        } // Si5351 CLK vklopljen power spet nastavi ročno
 
    }
    // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_6)) {
      inputMessage = request->getParam(PARAM_INPUT_6)->value();
      inputParam = PARAM_INPUT_6;
      //if (inputMessage == "L") {swSer.println("L");} // LSB mode 1
      //if (inputMessage == "U") {swSer.println("U");} // USB mode 2
      if (inputMessage == "L") {
         txmode = 1;
         //si5351.set_correction(CORRECTION, SI5351_PLL_INPUT_XO);   // zamika frekvenco?
         interval = 184; //208UL;  //5,4kHz; 208 je priblizno 4800
         //frekadd = 0;   // no add frequency
         napaka = frekvin * 0.00037f + 12.5093f;
         frekvinfm = frekvin + (double)napaka;
         //frekvinfm = frekvin; 
         si5351.set_freq(frekvinfm * 100, SI5351_CLK1);
         si5351.initFastPLLtables(CORRECTION);
         } // LSB mode 1, sampling 12,5kHz 8khz zdaj
      if (inputMessage == "U") {
         txmode = 2;
         //si5351.set_correction(CORRECTION, SI5351_PLL_INPUT_XO);   // zamika frekvenco?
         interval = 184; //208UL;  //80UL;
         //frekadd = 0;   // no add frequency
         napaka = frekvin * 0.00037f + 12.5093f;
         frekvinfm = frekvin + (double)napaka;
         //si5351.set_correction(CORRECTION, SI5351_PLL_INPUT_XO);   // zamika frekvenco?
         //frekvinfm = frekvin; 
         si5351.set_freq(frekvinfm * 100, SI5351_CLK1);
         si5351.initFastPLLtables(CORRECTION);
         } // USB mode 2, sampling 5kHz
      if (inputMessage == "A") {
         txmode = 3;
         si5351.set_correction(CORRECTION, SI5351_PLL_INPUT_XO);   // zamika frekvenco?
         //frekadd = 0;   // no add frequency for AM
         si5351.set_freq(frekvin * 100, SI5351_CLK1); //traja 1350 us
         digitalWrite(MUTE_PIN, HIGH);  //Put pin in high impedance to enable clk na gate
         interval = 84UL;  //80UL;  //84 = cca 12kHz
         //si5351.output_enable_fast(1);
         } // AM mode 3, sampling 12,5kHz
      if (inputMessage == "F")
        {
        txmode = 4;
        si5351.set_correction(CORRECTION, SI5351_PLL_INPUT_XO);   // zamika frekvenco?
        interval = 175UL;  //5,7khz lahko gre do 6khz
        //frekadd = 10000;  // necessary to add 10kHz
        napaka = frekvin * 0.00037f + 12.5093f;
        frekvinfm = frekvin + (double)napaka;
        //frekvinfm = frekvin; 
        si5351.set_freq(frekvinfm * 100, SI5351_CLK1);
        si5351.initFastPLLtables(CORRECTION);
        digitalWrite(MUTE_PIN, HIGH);  //Put pin in high impedance to enable clk na gate
        dac_output_voltage(DAC_CHANNEL_1, fmbase);
        //si5351.output_enable_fast(1);
        } // FM-N mode 4, sampling 5kHz
      
    }
    // GET input7 value on <ESP_IP>/get?input7=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_7)) {
      inputMessage = request->getParam(PARAM_INPUT_7)->value();
      inputParam = PARAM_INPUT_7;
      strgain = inputMessage;
      mojgain = strgain.toFloat();
  
    }
    // get SSBphase
    else if (request->hasParam(PARAM_INPUT_8)) {
      inputMessage = request->getParam(PARAM_INPUT_8)->value();
      inputParam = PARAM_INPUT_8;
      SSBphase = inputMessage;
      phaseofst = SSBphase.toInt();
      phaseofst = constrain(phaseofst, -4, 4);
      //dacWrite(26, powerdac);
      //dac_output_voltage(DAC_CHANNEL_2, powerdac);          
    }
    
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    
      request->send_P(200, "text/html", index_html);                                                                
  });
  server.onNotFound(notFound);
  server.begin();
  Serial.println("HTTP Server Started");

  // Initiating the Signal Generator (si5351)
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, SI5351_REF, CORRECTION); //nastavi tudi wire na 100kHz
  
  si5351.set_freq(frekvin * 100, SI5351_CLK1);
  
  si5351.set_phase(SI5351_CLK1, 0);
  // We need to reset the PLL before they will be in phase alignment
  si5351.pll_reset(SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA); // Set for max power default is 2MA  max je 8
  
  si5351.output_enable(SI5351_CLK2, 0);  // ugasnem tetji izhod

  si5351.update_status();
  // Show the initial system information
  Serial.println("Module initializated");

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  startULPADC();
  
  build_lut();  //naredi look up tabelo

  Serial.println("Clock speed");
  Serial.println(Wire.getClock());
  //si5351.set_freq(frekvinfm * 100, SI5351_CLK1); or si5351.set_freq(frekvin * 100, SI5351_CLK1);
  si5351.initFastPLLtables(CORRECTION);
  //Wire.getClock();
} // konec setup


void loop() {
  //static long oldT = 0;
  unsigned long currentMicros = micros();

  if(currentMicros - previousMicros > interval)
  {
    previousMicros = currentMicros;
    GPIO.out_w1ts = (1 << SCOPE_PIN);  // HIGH (fast) to scope for measure sampling frequ
  switch(txmode)
           {
    case 1: {  //LSB modulacija sampling 12,5kHz zdaj 4.8khz zacetek
      //timenow = micros();
  currentIndex = (RTC_SLOW_MEM[indexOffset] + 100) & 1023;   //1024 - 100 samples away
  anvhod34 = RTC_SLOW_MEM[currentIndex + bufferOffset] & 0xffff;  //9 bitno
  //anvhod34sh = RTC_SLOW_MEM[currentIndex + bufferOffset + phaseofst] & 0xffff;  //9 bitno
  //anvhod34 = anvhod34>>1;  //8 bitno
    adc = anvhod34 - 255; // postane -256 do 256
    adc = dc_block(adc);
    adc = (int16_t)(adc * mojgain);
    adc = constrain(adc, -511, 511);

    static int16_t ssb_lpf = 0;  //LPF filter je že v ssbusdx funkciji? ni dovolj
    adc = (adc + ssb_lpf) >> 1;
    ssb_lpf = adc;
    
    df = ssbusdx(adc); //procesiranje po redcornerusdx
    output_envelope(amp);
    
    dif = df*1; // deviacija frekvence od nosilca 
    /*cal_factor = (int32_t)(CORRECTION + 369236 - dif*139);  // 369236 is to correct widened dif, dif must never be negative    
    si5351.set_correction2(cal_factor, SI5351_PLL_INPUT_XO);*/   // zamika frekvenco?
    si5351.setCorrectionFastLSB(dif);
    //si5351.output_enable_fast(amp >= ssbtrsh);
    if(amp >= ssbtrsh) {   
    digitalWrite(MUTE_PIN, HIGH);  //Put pin in high impedance         
    } else {
    digitalWrite(MUTE_PIN, LOW);  //Shunt BS170 gate with 1K
    }
    
    //timedif = micros() - timenow;
    break; }
  case 2: {   //USB modulacija sampling 12,5kHz zdaj 4,8khz zacetek
    //timenow = micros();
  currentIndex = (RTC_SLOW_MEM[indexOffset] + 100) & 1023;   //1024 - 100 samples away
  anvhod34 = RTC_SLOW_MEM[currentIndex + bufferOffset] & 0xffff;  //9 bitno
  //anvhod34sh = RTC_SLOW_MEM[currentIndex + bufferOffset + phaseofst] & 0xffff;  //9 bitno
  //anvhod34 = anvhod34>>1;  //8 bitno
    adc = anvhod34 - 255; // postane -256 do 256
    adc = dc_block(adc);
     
    adc = (int16_t)(adc * mojgain);
    adc = constrain(adc, -511, 511);
    static int16_t ssb_lpf = 0;
    adc = (adc + ssb_lpf) >> 1;
    ssb_lpf = adc;
    //df = ssbusdx(adc);
    df = ssbusdx(adc); //procesiranje po redcornerusdx
    /*uint16_t amptrsh = amp;
    if(amptrsh <= ssbtrsh) {
      amptrsh = 0;
      dac_output_voltage(DAC_CHANNEL_2, 0);
    } else {
      dac_output_voltage(DAC_CHANNEL_2, powerdac);
    }*/
    output_envelope(amp);
       
    dif = df*1; // deviacija frekvence od nosilca
    /*cal_factor = (int32_t)(CORRECTION + 369236 - dif*136);  // 369236 is to correct widened dif, dif must never be negative   
    si5351.set_correction2(cal_factor, SI5351_PLL_INPUT_XO);*/   // zamika frekvenco?
    si5351.setCorrectionFastUSB(dif);
    //si5351.output_enable_fast(amp >= ssbtrsh);
    if(amp >= ssbtrsh) {   
    digitalWrite(MUTE_PIN, HIGH);  //Put pin in high impedance         
    } else {
    digitalWrite(MUTE_PIN, LOW);  //Shunt BS170 gate with 1K
    }
    /*if(amp < ssbtrsh) {
      //dac_output_voltage(DAC_CHANNEL_2, 0);
      //si5351.output_enable(SI5351_CLK1, 0);  // ugasnem CLK1
      digitalWrite(MUTE_PIN, LOW);  //Shunt BS170 gate with 1K 
    } else {
      //dac_output_voltage(DAC_CHANNEL_2, powerdac);
      //si5351.output_enable(SI5351_CLK1, 1);  // vklopim CLK1
      digitalWrite(MUTE_PIN, HIGH);  //Put pin in high impedance 
    }*/
    //timedif = micros() - timenow;
    break;}

  case 3: {   //AM modulacija sampling 12kHz
    //timenow = micros();
  currentIndex = (RTC_SLOW_MEM[indexOffset] + 100) & 1023;   //1024 - 100 samples away
  anvhod34 = RTC_SLOW_MEM[currentIndex + bufferOffset] & 0xffff;  //9 bitno
  //anvhod34 = anvhod34>>1;  //8 bitno
    adc = anvhod34 - 255; // postane -256 do 256
    //adc = dc_block(adc);
    adc = dc_block_am(adc);     // DC removal FIRST
    adc = (int16_t)(adc * mojgain);
    /*static int16_t am_lpf = 0;
    adc = (adc + 3 * am_lpf) >> 2;  // ~3 kHz LPF @ 12 kHz Fs
    am_lpf = adc;*/
    static int16_t am_lpf = 0;
    adc = (adc + am_lpf) >> 1;
    am_lpf = adc;
    
    //am_lpf = constrain(am_lpf, -255, 255);
     int am_lpf2 = constrain(am_lpf, -255, 255);

   dacvalue = ((int)am_lpf2*0.5f) + dcoffset; // + powerdac; // dcofset postavi na sredino powerdc doda + ali -
  //dacvalue = constrain(dacvalue, 0, 255);
   dac_output_voltage(DAC_CHANNEL_1, dacvalue);
   dac_output_voltage(DAC_CHANNEL_2, powerdac);
    //timedif = micros() - timenow;
    break;}
  case 4: {  //FM modulacija se spremeni v sampling 5kHz
    //timenow = micros();  
    currentIndex = (RTC_SLOW_MEM[indexOffset] + 100) & 1023;   //100 samples away
    anvhod34 = RTC_SLOW_MEM[currentIndex + bufferOffset] & 0xffff;  //9 bitno
   //anvhod34 = anvhod34>>1;  //8 bitno
    adc = anvhod34 - 255; //257; // postane -256 do 256
    //adc = dc_block_am(adc);     // DC removal FIRST
    adc = dc_block(adc);
    // ~2 kHz LPF at 5 kHz Fs signal must be ≤ 2 kHz
    static int16_t lpf_zl = 0;
    adc = (adc + 2*lpf_zl) / 3;
    lpf_zl = adc;

    /*static int16_t fm_lpf = 0;
    adc = (adc + fm_lpf) >> 1;
    fm_lpf = adc;*/

    adc = (int16_t)(adc * mojgain);
    adc = constrain(adc, -511, 511);
    //adc = constrain(adc, -1000, 1000);
    
    dif = (int) adc; // deviacija frekvence od nosilca
    /*cal_factor = (int32_t)(dif*200 + CORRECTION + 369236);  // 369236 is to correct widened dif, dif must never be negative
    //timenow = micros();
    //GPIO.out_w1ts = (1 << SCOPE_PIN);  // HIGH (fast) to scope for measure sampling frequ
    si5351.set_correction2(cal_factor, SI5351_PLL_INPUT_XO);*/   // zamika frekvenco?
    si5351.setCorrectionFastFM(dif);
    dac_output_voltage(DAC_CHANNEL_2, powerdac);
    
    //timedif = micros() - timenow;
    break;}

  default:
    // statements
    break;
           }
    GPIO.out_w1tc = (1 << SCOPE_PIN);  // LOW
  }
    //delay(1);
}  //end of loop

// Function finding zero offsets of the ADC
/*void findoffset(){
    const int calibration_rounds = 1000;     
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
      offsetadc = offsetadc + local_adc1_read(ADC1_CHANNEL_6); // - micoffset;  //Je za pin 34 mikrofon vhod
        delay(1);
    }
    // calculate the mean offsets
    offsetadc = offsetadc / calibration_rounds;    
}*/
