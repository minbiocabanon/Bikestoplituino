#include <EEPROM.h>

#define PI 3.141592653
#define MASTER_LED_PIN 2
#define SLAVE_LED_PIN 3
#define DBG_LED_PIN 4
#define N_FILTER_CHANNEL 2
#define ACC_X_PIN 3
#define ACC_Y_PIN 4
#define ACC_Z_PIN 5
#define DEFAULT_MID_0 515.30
#define DEFAULT_MID_1 500.53
#define DEFAULT_MID_2 500.53
#define DEFAULT_SCALE_0 186.92
#define DEFAULT_SCALE_1 191.17
#define DEFAULT_SCALE_2 191.

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

// loop index -- used for pseudo time
long long count = 0;

// light state false = off, true = on
boolean state = false;

// used for loop timer
unsigned long last_time = 0;

// variable for filtered accel data
double cooked[N_FILTER_CHANNEL];

// variable for filtered down data
double down[N_FILTER_CHANNEL];

// historesis thresholds
// val < lo ==> off
// lo < val < hi ==> no state change
// val > hi ==> on
double lo = 0.03;
double hi = 0.05;


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
    down[i] = apply_filter(raw[i], down_in_taps + i * N_DOWN_TAP, down_out_taps + i * N_DOWN_TAP, N_DOWN_TAP, down_ff, down_fb);
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
  scales[ind] = (max_val - min_val) / 2.;
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
      else if(chr == 'Z'){
        calibrate_one(1);
      }
      else if(chr == 'C'){
        clearEEPROM();
      }
    }
  }
}

// start serial
// read EEPROM for cal data if present, use defaults otherwise
// print diagnostics
void setup(){
  int i, j, ind;
  byte addr = 0, byte_val;
  double val;

  analogReference(EXTERNAL);
  Serial.begin(9600);
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
    byte_val = EEPROM.read(i * 4);
    if(byte_val != 255){
      midpoints[i] = readDouble(i * 4);
    }
    else{
      midpoints[i] = DEFAULT_MIDS[i];
    }
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
}

// read ADCs
// filter data
// threshold check
// LED update
void loop(){
  char chr;
  int i, j;
  float x, y, z, norm, theta, down_x, down_z, val;
  boolean out_state;

  count++;
  if(count < 0){
    count = 0;
  }

  process_input();  
  get_cooked();
  // delay(1);

  x = (cooked[0] - midpoints[0]) / scales[0];
  z = (cooked[1] - midpoints[1]) / scales[1];
  down_x = (down[0] - midpoints[0]) / scales[0];
  down_z = (down[1] - midpoints[1]) / scales[1];

  //ahead = [down_z, -down_x]
  val = -x * down_z + z * down_x;

  // norm = sqrt(z*z + x*x) - 1;
  // theta = atan2(z, x) * 180 / PI;

  // units of x are gs, low pass filtered
  // if state is between high and low -- no change!
  // try to keep timing the same between branches
  // (no short circuiting)
  if(val > hi){
    state = true;
  }
  if(val < lo){
    state = false;
  }
  if(count % 300 < 10 ){
    out_state = true;
  }
  else{
    out_state = state;
  }

  digitalWrite(MASTER_LED_PIN, out_state);   
  /*
   * Uncomment the following line for independent control of slave
   * light.
   */
  // digitalWrite(SLAVE_LED_PIN, out_state);   

  /*
   * Uncomment the following line to turn DBG signal on.
   * DBG signal is approx 1 Hz blink on pin DIP pin 6 (arduino pin 4)
   */
  // digitalWrite(DBG_LED_PIN, ((count % 1200) < 600));   
  if(count % 1000 == 0){
    Serial.print(millis() - last_time);
    Serial.print(" ");   
    Serial.print(x);   
    Serial.print(" ");   
    Serial.print(z);   
    Serial.print(" - ");   
    Serial.print(down_x);   
    Serial.print(" ");   
    Serial.print(down_z);   
    Serial.print(" ");   
    Serial.print(val);   
    Serial.println(" ");   
    last_time = millis();
  }
}