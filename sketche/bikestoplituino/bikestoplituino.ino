//----------------------------------------------------------------------
//!\file	FreinVelo.ino
//!\brief	
//!\author	Julien C	
//!\date	June 2014
//---------------------------------------------------------------------- 
#include <JeeLib.h>
#include <SoftwareSerial.h>
SoftwareSerial oledSerial(6, 16); // RX, TX   Jeenode PORT3 : RX on Analog and TX on Digital

#include "displayshield4d.h"
#include <EEPROM.h>
#include <MsTimer2.h>

#define PI 3.141592653
#define MASTER_LED_PIN 5
#define SLAVE_LED_PIN 3
#define DBG_LED_PIN 4
#define N_FILTER_CHANNEL 1
#define ACC_X_PIN 0
#define ACC_Y_PIN 1
#define ACC_Z_PIN 3

#define QUANTUM 3300.0 / 1024.0				// mv/bit
#define DEFAULT_SCALE_0 300.0				// mv/g
#define DEFAULT_SCALE_1 300.0				// mv/g
#define DEFAULT_SCALE_2 300.0				// mv/g
#define DEFAULT_MID_0 (1650.0 / QUANTUM)	// 512 , zero g, mid-voltage
#define DEFAULT_MID_1 (1650.0 / QUANTUM)	// 512 , zero g, mid-voltage
#define DEFAULT_MID_2 (1650.0 / QUANTUM)	// 512 , zero g, mid-voltage

#define INTERVAL_DEBUG	50
#define INTERVAL_JERK	100


// OLED variables
unsigned int textcolorR = 255;	//
unsigned int textcolorG = 255;	//
unsigned int textcolorB = 255;	//
char *textlcd = "Hello world !";
//flags
boolean bflag_update_lcd = false;
boolean bflag_debug = true;
boolean bflag_timer_debug = false;
boolean bflag_timer_jerk = false;
boolean out_state;

double DEFAULT_MIDS[] = {
  DEFAULT_MID_0,
  DEFAULT_MID_1,
  DEFAULT_MID_2};
double DEFAULT_SCALES[] = {
  DEFAULT_SCALE_0,
  DEFAULT_SCALE_1,
  DEFAULT_SCALE_2};

double midpoints[N_FILTER_CHANNEL];
double scales[N_FILTER_CHANNEL];
//************************************************************
// low pass butterworth filter coefficients
// Auto generated from plot.py
// Low pass filter to smooth accel data, BW=1.000000
#define N_TAP 3
double in_taps[N_TAP * N_FILTER_CHANNEL];
double out_taps[N_TAP * N_FILTER_CHANNEL];
// feed forward
double ff[] = {
  5.04365902957e-06,
  1.00873180591e-05,
  5.04365902957e-06
};
// feed back
double fb[] = {
  1.0,
  -1.99363781299,
  0.993657987627
};

// Slower filter for down traking, BW=0.050000
#define N_DOWN_TAP 2
double down_in_taps[N_TAP * N_FILTER_CHANNEL];
double down_out_taps[N_TAP * N_FILTER_CHANNEL];
// feed forward
double down_ff[] = {
  9.42388980798e-05,
  9.42388980798e-05
};
// feed back
double down_fb[] = {
  1.0,
  -0.999811522204
};
//************************************************************

int channels[] = {
  ACC_X_PIN, ACC_Z_PIN, ACC_Y_PIN};

// variable will store raw a2d values, one for each channel
int raw[N_FILTER_CHANNEL];

// used for loop timer
unsigned long last_time = 0;

// variable for filtered accel data
double cooked[N_FILTER_CHANNEL];

// variable for filtered down data
double down[N_FILTER_CHANNEL];

float x, xmin, fjerk, fjerkmin, fx_p;
double delta;
char chr;
int i, j, count;


/*******************************************************
	Constructor
 ********************************************************/
DisplayShield4d::DisplayShield4d() {
	// Constructor
}

/*******************************************************
	Function: 	
		Init
	Description:	
		Initialise display. Prior to this function
		you must start serial by oledSerial.begin()
		For example: oledSerial.begin(57600);
	Params:	None	
	Return:	0
 ********************************************************/
uint8_t DisplayShield4d::Init()
{
	pinMode(OLED_RESETPIN, OUTPUT);
	Reset();  
	delay(OLED_INITDELAYMS);
	oledSerial.write(OLED_DETECT_BAUDRATE); 
	delay(10);
	//GetDeviceInfo();
	return 0;
}

/*******************************************************
	Function:	
		Reset 
	Description:
		Phisicaly reset the display. D7 jumper must be conected
		Also is a good thing to have pinMode(7, OUTPUT) set in Setup function
	Params:	None
	Return: No return
 ********************************************************/
uint8_t DisplayShield4d::Reset() {
	digitalWrite(OLED_RESETPIN, LOW);
	delay(10);                  
	digitalWrite(OLED_RESETPIN, HIGH);
	delay(10); 
}

/*******************************************************
	Function: 
		Clear
	Description:
		Clear entire display
	Params:	None
	Return: Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::Clear() {
	oledSerial.write(OLED_CLEAR);
	return GetReply();
}

/*******************************************************
	Function: 
		GetReply
	Description:
		Used to get command response
	Params:	None
	Return: Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::GetReply() {
	byte incomingByte = OLED_ACK;
	while (!oledSerial.available()) { delayMicroseconds(150); }
	incomingByte = oledSerial.read();
	return incomingByte;
}

/*******************************************************
	Function: 
		RGB
	Description:
		Returns correct int format for color parm
	Params:	red, green, blue - From 0 to 254
	Return:	None
 ********************************************************/
unsigned int DisplayShield4d::RGB(uint8_t red, uint8_t green, uint8_t blue) 
{
	char text[255];
	int outR = ((red * 31) / 255);
	int outG = ((green * 63) / 255);
	int outB = ((blue * 31) / 255);
	return (outR << 11) | (outG << 5) | outB;
}

/*******************************************************
	Function: 
		SetBackground
	Description:
		changes the current background colour. Once this command is sent, only the background colour will change.

	Params:	color
	Return:	Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::SetBackground(unsigned int color)
{
	oledSerial.write(OLED_SETBACKGROUND);
	// Color
	oledSerial.write(color >> 8);				// MSB
	oledSerial.write(color & 0xFF);				// LSB
	return GetReply();
}

/*******************************************************
	Function: 
		SetContrast
	Description:
		Set current contrast

	Params:	cval - 0 to 15
	Return:	Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::SetContrast(char val)
{
	oledSerial.write(OLED_COMMAND_CONTROL);
	// Set contrast
	oledSerial.write(val); // 0-15
	return GetReply();
}

/*******************************************************
	Function: 
		SetState
	Description:
		This command changes some of the display settings

	Params:	state - Can be COMMAND_DISPLAY_ON, COMMAND_DISPLAY_OFF, OLED_COMMAND_SHUTDOWN, OLED_COMMAND_POWEROFF (for low power save state)
	Return:	Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::SetState(char state)
{
	oledSerial.write(OLED_COMMAND_CONTROL);
	// Set contrast
	oledSerial.write(state);
	return GetReply();
}

/*******************************************************
	Function: 
		SetState
	Description:
		Puts GOLDELOX-SGC chip in to low power mode and optionally waits for certain conditions to wake it up

	Params:	wake_cond - Can be OLED_COMMAND_STOP_SD, OLED_COMMAND_WAKEONKOYSTICK or OLED_COMMAND_WAKEONSERIAL
	Return:	Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::Sleep(char wake_cond)
{
	oledSerial.write(OLED_COMMAND_SLEEP);
	// Set wakeup condition
	oledSerial.write(wake_cond);
	return GetReply();
}

/*******************************************************
	Function: 
		setfont
	Description:
		This  command selects  one of  the available internal  fonts.
	Params:
		font_type: OLED_FONT5x7, OLED_FONT8x8 or OLED_FONT8x12
	Return:	
		Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::setfont(uint8_t font_type)
{
	oledSerial.write(OLED_SETFONT); 
	oledSerial.write(font_type);
	return GetReply();
}

/*******************************************************
	Function: 
		setfontmode
	Description:
		This command will  change the attribute of the text so that an object behind the text can either be blocked or transparent.
	Params:
		font_mode: OLED_FONT_TRANSPARENT or OLED_FONT_OPAQUE
	Return:	
		Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::setfontmode(uint8_t font_mode)
{
    oledSerial.write(OLED_SETFONTMODE); 
	oledSerial.write(font_mode);
	return GetReply();
}



/*******************************************************
	Function: 
		placetext
	Description:
		This command will  display an ascii string text
	Params:
		font_type: OLED_FONT5x7, OLED_FONT8x8 or OLED_FONT8x12
		text is 256 bytes MAX !
	Return:	
		Return OLED_ACK is done or OLED_NAK if not
 ********************************************************/
uint8_t DisplayShield4d::placetext(uint8_t x, uint8_t y, uint8_t font, unsigned int color, char *text)
{
	oledSerial.write(OLED_PLACE_TEXT);
	oledSerial.write(x);
	oledSerial.write(y);
	oledSerial.write(font);
	// Color
	oledSerial.write(color >> 8);		// MSB			
	oledSerial.write(color & 0xFF);		// LSB
	// Serial.print("strlen de textlcd = ");
	// Serial.println(strlen(text));
	for (int i=0 ; i<(strlen(text)-1) ; i++)	{
		oledSerial.write(text[i]);
	}
	oledSerial.write(OLED_STRINGTERMINATOR, 1); // String terminator
	return GetReply();
}


DisplayShield4d  oled;


// write a double to EEPROM at given address
int writeDouble(double d, int addr){
  byte tab, i, *tab_p;
  for(i = 0; i < 4; i++){
    tab_p = (byte *)&d + i;
    tab = *tab_p;
    EEPROM.write(addr + i, tab);
  }
  return addr + 4;
}

// read a double to EEPROM at given address
double readDouble(int addr){
  /*
   bytes 0-11  xyz midpoints
   bytes 12-23 xyz scales
   */
  byte tab[4], i;
  double out;
  double *val_p;

  for(i = 0; i < 4; i++){
    tab[i] = EEPROM.read(addr + i);
  }
  val_p = (double *)&tab;
  out = *val_p;

  return out;
}

void clearEEPROM(void){
  int i;
  for(i = 0; i < 256; i++){
    EEPROM.write(i, 255);
  }
}

// apply IIR filter with coeff from _ff and _fb
// to each of N_FILTER_CHANNEL accel channels
double apply_filter(int _raw, double *_in_taps, double *_out_taps, int n_tap, double *_ff, double *_fb){
  double filtered = 0;
  byte i;

  // shift taps
  for(i = n_tap - 1; i > 0; i--){
    _in_taps[i] = _in_taps[i - 1];
  }
  _in_taps[0] = _raw;

  // out = ibuff . b - obuff . a
  for(i = 0; i < n_tap; i++){
    filtered += _in_taps[i] * _ff[i];
  }
  for(i = 0; i < n_tap - 1; i++){
    filtered -= _out_taps[i] * _fb[i + 1];
  }
  // shift taps
  for(i = n_tap - 2; i > 0; i--){
    _out_taps[i] = _out_taps[i - 1];
  }
  _out_taps[i] = filtered;

  return filtered;
}

// get raw data form a2d converters
// called by get_cooked
void get_raw(){
  int i;
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    raw[i]  = analogRead(channels[i]);
  }
}

// make call to get_raw() and apply IIR filter
void get_cooked(){
  int i;
  get_raw();
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    cooked[i] = apply_filter(raw[i], in_taps + i * N_TAP, out_taps + i * N_TAP, N_TAP, ff, fb);
	// cooked[i] = raw[i];
  }
}

// determine min and max ADC values.  
// follow directions on serial port
// index corresponds to "channels" pin
// cal values stored in EEPROM since EEPROM data is is reatained
// on power off.
void calibrate_one(byte ind){
  double max_val = -1., min_val = 1024.;
  int j, i, addr;

  for(i = 0; i < 1000; i++){
    // clear the filter
    get_cooked();
  }	
  while(Serial.available() == 0){
    for(j = 0; j < 1000; j++){
      get_cooked();
      if(cooked[ind] < min_val){
        min_val = cooked[ind];
      }
      if(cooked[ind] > max_val){
        max_val = cooked[ind];
      }
    }
    Serial.print(min_val);
    Serial.print(" ");
    Serial.print(cooked[ind]);
    Serial.print(" ");
    Serial.println(max_val);
  }
  midpoints[ind] = (max_val + min_val) / 2.;
  scales[ind] = QUANTUM * (max_val - min_val) / 2.;
  addr = ind * 4;
  writeDouble(midpoints[ind], addr);

  addr = (3 + ind) * 4;
  writeDouble(scales[ind], addr);

  Serial.println(midpoints[ind]);
  Serial.println(scales[ind]);
  Serial.println(ind, DEC);
  Serial.println((3 + ind) * 4, DEC);
}

// print contents of EEPROM
// midpoint (mid) and scale (sca) for three ADC channels.
void printEEPROM(){
  int i;
  Serial.println(" *******  BEGIN EEPROM ********");
  for(i = 0; i < 3; i++){
    Serial.print("mid ");
    Serial.print(readDouble(i * 4));
    Serial.print(" sca ");
    Serial.print(readDouble((i + 3) * 4));
    Serial.println("");
  }
  Serial.println("bytes:");
  for(i = 0; i < 24; i++){
    Serial.print(EEPROM.read(i), DEC);
  }
  Serial.println(" *******  END EEPROM ********");
}

void process_input(){
  int i;
  char chr;

  if(Serial.available()){
    for(i = 0; i < Serial.available(); i++){
      chr = Serial.read();
      if(chr == 'X'){
        calibrate_one(0);
      }
      else if(chr == 'C'){
        clearEEPROM();
      }
      else if(chr == 'D'){
        bflag_debug = !bflag_debug;
      }	  
    }
  }
}

//----------------------------------------------------------------------
//!\brief	background task that drive the OLED LCD display
//---------------------------------------------------------------------- 
void tache_affiche_lcd(void){
		
	if( bflag_update_lcd == true){
		// clear screen
		oled.Clear();
		// set font size
		oled.setfont(OLED_FONT5x7);
		// set transparency
		oled.setfontmode(OLED_FONT_TRANSPARENT);
		// write text
		// Serial.print("textlcd = ");
		// Serial.println(textlcd);
		char xchar[8], xminchar[8], jerkchar[9], jerkminchar[9], deltachar[9];
		dtostrf(x, 6, 2, xchar);
		dtostrf(xmin, 6, 2, xminchar);
		dtostrf(fjerk, 6, 3, jerkchar);
		dtostrf(fjerkmin, 6, 3, jerkminchar);
		dtostrf(delta, 6, 3, deltachar);
		oled.placetext(0, 0, 1, oled.RGB(textcolorR, textcolorG, textcolorB), xchar);
		oled.placetext(6, 0, 1, oled.RGB(255, 0, 0), xminchar);
		oled.placetext(0, 1, 1, oled.RGB(0, 255, 0), jerkchar);
		oled.placetext(6, 1, 1, oled.RGB(255, 0, 0), jerkminchar);
		oled.placetext(0, 2, 1, oled.RGB(0, 0, 255), jerkminchar);
				
		// flag reset because we have juste update the lcd
		bflag_update_lcd = false;
	}
}		

//----------------------------------------------------------------------
//!\brief         compute jerk (variation of acceleration)
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void compute_jerk(void){
	x = (float) (cooked[0] - midpoints[0]) * QUANTUM / scales[0];

	// count++;
	
	// if(count > 0){	// test 1000
		fjerk = x - fx_p;
		// delta = delta + fjerk;
		//store value
		fx_p = x;
		
		// bflag_timer_jerk = false;
		// count = 0;
	// }
	
	// if( x < xmin)
		// xmin = x;
	// if( fjerk < fjerkmin)
		// fjerkmin = fjerk;
}

//----------------------------------------------------------------------
//!\brief         compare jerk with trig values 
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void trig_jerk(void){
	// if state is between high and low -- no change!
	// try to keep timing the same between branches
	// (no short circuiting)
	if(fjerk < -0.00005){
		out_state = true;
	}
	if(fjerk > -0.000025){
		out_state = false;
	}
}

//----------------------------------------------------------------------
//!\brief         drive led on or off
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void drive_led(void){
  digitalWrite(MASTER_LED_PIN, out_state);
}  

//----------------------------------------------------------------------
//!\brief         Display debug if user as specified it
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void debug(void){
 
 	if( bflag_debug == true && bflag_timer_debug == true){
		// Serial.print(millis() - last_time);
		Serial.print(millis());
		Serial.print(", ");   
		Serial.print(cooked[0]); 
		Serial.print(", ");
		Serial.print(midpoints[0]);   
		Serial.print(", ");		
		Serial.print(x,6);
		Serial.print(", ");   
		Serial.print(fjerk,6);  
		Serial.println(""); 
		// last_time = millis();

		bflag_update_lcd = true;
		bflag_timer_debug = false;
	}
}
//----------------------------------------------------------------------
//!\brief         Task
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void TimerDebug(){
	bflag_timer_debug = true;
}

//----------------------------------------------------------------------
//!\brief         Task
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void TimerJerk(){
	bflag_timer_jerk = true;
}


//----------------------------------------------------------------------
//!\brief         SETUP function
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
// start serial
// read EEPROM for cal data if present, use defaults otherwise
// print diagnostics
void setup(){
	int i, j, ind;
	byte addr = 0, byte_val;
	double val;
  
	// set the data rate for the SoftwareSerial port used for Oled display
	oledSerial.begin(9600);
	oled.Init();
	// clear screen
	oled.Clear();
	// set this flag for displaying default text (once)
	bflag_update_lcd = true;

	analogReference(DEFAULT);
	Serial.begin(115200);
	delay(100);
	Serial.println("ACC setup() ");
	pinMode(MASTER_LED_PIN, OUTPUT);
	pinMode(SLAVE_LED_PIN, OUTPUT);
	pinMode(DBG_LED_PIN, OUTPUT);
	digitalWrite(MASTER_LED_PIN, false);
	digitalWrite(SLAVE_LED_PIN, false);
	digitalWrite(DBG_LED_PIN, false);

 
	
  // read cal data
  printEEPROM();
  for(i = 0; i < 3; i++){
    // byte_val = EEPROM.read(i * 4);
    // if(byte_val != 255){
      // midpoints[i] = readDouble(i * 4);
    // }
    // else{
      // midpoints[i] = DEFAULT_MIDS[i];
    // }
	midpoints[i] = 512.0;
  }  
  for(i = 0; i < 3; i++){
    byte_val = EEPROM.read(3 * 4 + i * 4);
    if(byte_val != 255){
      scales[i] = readDouble(3 * 4 + i * 4);
    }
    else{
      scales[i] = DEFAULT_SCALES[i];
    }
  }
  for(ind = 0; ind < 3; ind++){
    Serial.print("ind ");
    Serial.print(ind);
    Serial.print(" ");
    Serial.print("mid ");
    Serial.print(midpoints[ind]);
    Serial.print(" sca ");
    Serial.print(scales[ind]);
    Serial.println("");
  }
  // prime filter taps
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    val = analogRead(channels[i]);
    Serial.println(val);
    for(j = 0; j < N_TAP; j++){
      in_taps[i * N_TAP + j] = val;
      out_taps[i * N_TAP + j] = val;
    }
    for(j = 0; j < N_DOWN_TAP; j++){
      down_in_taps[i * N_DOWN_TAP + j] = val;
      down_out_taps[i * N_DOWN_TAP + j] = val;
      Serial.println(i * N_DOWN_TAP + j, DEC);
    }
  }
  	Serial.println(" Structure DEBUG : ");
	Serial.println("millis,cooked[0], midpoints[0], x, fjerk");  
	
	MsTimer2::set(INTERVAL_DEBUG, TimerDebug);
	// MsTimer2::set(INTERVAL_JERK, TimerJerk);
	MsTimer2::start();
  
}

//----------------------------------------------------------------------
//!\brief         LOOP function
//!\param[in]     -
//!\return        -
//---------------------------------------------------------------------- 
void loop(){

	process_input();  
	get_cooked();
	compute_jerk();
	trig_jerk();
	debug();
	drive_led();
	// tache_affiche_lcd(); 
}
