//
// Record sound as .wav on a SD card
// Interface to Campbell datalogger
//
// Loggerhead Instruments
// 2016
// David Mann
// 
// Modified from PJRC code
// Requires the audio shield:S
//   http://www.pjrc.com/store/teensy3_audio.html
//
// Two pushbuttons need to be connected:
//   Record Button: pin 0 to GND
//   Stop Button:   pin 1 to GND


/* To Do: 
 * reset on 'r' from campbell
 * Time stamp on file
 * 
*/

#include <SerialFlash.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
//#include <SdFat.h>
#include <datafile32.h>
#include <Snooze.h>
#include <TimeLib.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

//#define OLED_RESET 4
//Adafruit_SSD1306 display(OLED_RESET);
//#define BOTTOM 55

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

unsigned long baud = 115200;

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

#define MODE_NORMAL 0
#define MODE_DIEL 1

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;

// Pin Assignments
const int reset_pin = 4;

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
time_t startTime;
time_t stopTime;
time_t t;
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean imuFlag = 0;
boolean pressureFlag = 0;
boolean audioFlag = 1;
boolean CAMON = 0;

int AccelAddressInt = 0x53;  //with pin 12 grounded; board accel
int CompassAddress = 0x1E; 
int accel_x_int;
int accel_y_int;
int accel_z_int;
int magnetom_x;
int magnetom_y;
int magnetom_z;
int gyro_x;
int gyro_y;
int gyro_z;
int gyro_temp;

float pressure_period = 1.0;
float imu_period = 0.1;
float audio_srate = 44100.0;
float audio_period = 1/audio_srate;

int recMode = MODE_NORMAL;
long rec_dur = 10;  // 10 s
long rec_int = 710; // 710 = every 12 minutes
int wakeahead = 12;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;

long file_count;
char filename[20];
SnoozeBlock snooze_config;

// The file where data is recorded
File frec;

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;
unsigned int rms;
float hydroCal = -164;

unsigned char prev_dtr = 0;

void setup() {
  Serial.begin(baud);
  pinMode(6, OUTPUT);  
  digitalWrite(reset_pin, HIGH);
  pinMode(reset_pin, OUTPUT);
  delay(1000);
  HWSERIAL.begin(baud); // communication to hardware serial  SERIAL_8N1
  HWSERIAL.transmitterEnable(6);

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  t = getTeensy3Time();
  if (t < 1451606400) Teensy3Clock.set(1451606400);
  startTime = getTeensy3Time();
  stopTime = startTime + rec_dur;
  
  Serial.println("Barnacle");
  HWSERIAL.println("Barnacle");

  
  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(SD.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    HWSERIAL.println("Unable to access SD card");
    while (1) {
      delay(1000);
    }
  }
  SdFile::dateTimeCallback(file_date_time);
  
  t = getTeensy3Time();
  if (startTime < t)
  {  
    startTime = t + 30;
    stopTime = startTime + rec_dur;  // this will be set on start of recording
  }
  
  if (recMode==MODE_DIEL) checkDielTime();  
  
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0);
  long ss = rec_int - wakeahead;
  if (ss<0) ss=0;
  snooze_hour = floor(ss/3600);
  ss -= snooze_hour * 3600;
  snooze_minute = floor(ss/60);
  ss -= snooze_minute * 60;
  snooze_second = ss;

  Serial.print("rec dur ");
  Serial.println(rec_dur);
  Serial.print("rec int ");
  Serial.println(rec_int);
  Serial.print("Current Time: ");
  printTime(t);
  Serial.print("Start Time: ");
  printTime(startTime);
  
  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(150);
  AudioInit();
        
  mode = 0;
}

//
// MAIN LOOP
//
void loop() {
 
  t = getTeensy3Time();
  
  // Standby mode
  if(mode == 0)
  {
      if(t >= startTime){      // time to start?
        Serial.println("Record Start.");
        
        stopTime = getTeensy3Time() + rec_dur;
        startTime = stopTime + rec_int;
        if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

        mode = 1;
        
        startRecording();
      }
      else{
        // check for request from Campbell
        CheckSerial();  //will check for commands and send last file recorded
        delay(100);
      }
  }
  
  // Record mode
  if (mode == 1) {
    continueRecording();  // download data
    if(buf_count >= nbufs_per_file){       // time to stop?
      stopRecording();
      mode = 0;
    }
  }
}

void startRecording() {
  Serial.println("startRecording");
  FileInit();
  if (frec) {
    buf_count = 0;
    queue1.begin();
  }
  else
  {
    Serial.println("could not open file");
  }
}

void continueRecording() {
  if (queue1.available() >= 2) {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    frec.write(buffer, 512);
    buf_count += 1;
  }
}

void stopRecording() {
  Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  Serial.print("Audio Memory Max");
  Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  while (queue1.available() > 0) {
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
  }
  AudioMemoryUsageMaxReset();
  //frec.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  frec.close();
  delay(100);
  calcRMS();
}

void CheckSerial(){
  unsigned char dtr;
  int rd, wr, n;
  char buffer[80];
  // check if any data has arrived on the hardware serial port
  rd = HWSERIAL.available();
  if (rd > 0) {
      // read data from the hardware serial port
      n = HWSERIAL.readBytes((char *)buffer, 1);

      if (buffer[0] == 'a'){
        delay(10);
        Serial.println("Got A");
     
        // load file; HWSERIAL out  first n bytes
        byte wavbuffer[512];
        frec = SD.open(filename);
        long shortbufs = floor(nbufs_per_file/2);

        HWSERIAL.println("a");
        HWSERIAL.println(filename);
        HWSERIAL.println("BYTES");
        HWSERIAL.println((512*shortbufs) + 44);
        Serial.println(filename);
        if(frec){
          frec.read(&wavbuffer, 44);  // read wav header
          HWSERIAL.write(wavbuffer, 44);
          for (long i = 0; i<shortbufs; i++){
            frec.read(&wavbuffer, 512);
            HWSERIAL.write(wavbuffer, 512);
          }
          frec.close();
        }
        else{
          HWSERIAL.println("Could not open file");
        }
        HWSERIAL.println("N");
        HWSERIAL.flush();
      }
      if (buffer[0] == 'b'){
        Serial.println("Got B");
        // load file; HWSERIAL out last n bytes
        byte wavbuffer[512];
        frec = SD.open(filename);
        long shortbufs = floor(nbufs_per_file/2);
        HWSERIAL.println("b");
        HWSERIAL.println(filename);
        HWSERIAL.println("BYTES");
        HWSERIAL.println(512*shortbufs);
        Serial.println(filename);
        if(frec){
          frec.seek(sizeof(wav_hdr) + (shortbufs * 512));  // skip wav header and first half
          for (long i = 0; i<shortbufs; i++){
            frec.read(&wavbuffer, 512);
            HWSERIAL.write(wavbuffer, 512);
          }
          frec.close();
        }
        else{
          HWSERIAL.println("Could not open file");
        }
        HWSERIAL.println("N");
        HWSERIAL.flush();
      }
      if (buffer[0] == 'd'){
        Serial.println("Got d");
        HWSERIAL.println(filename);
        HWSERIAL.println("RMS");
        HWSERIAL.println(rms);
        HWSERIAL.println("BYTES");
        HWSERIAL.println((nbufs_per_file * 512) + 44);
        HWSERIAL.flush();
        Serial.println(filename);
        Serial.println("RMS");
        Serial.println(rms);
        Serial.println("BYTES");
        Serial.println((nbufs_per_file * 512) + 44);
        delay(100);
      }
      if(buffer[0] == 'r'){ //reset
        Serial.println("Rebooting...");
        CPU_RESTART
      }
      if(buffer[0] == 't'){ //set time
        Serial.println("Got t");
        // format YYYYMMDDHHMMSS
        int yr,mo,dy,hr,mn,sc;
        rd = HWSERIAL.available();
        int readcounter=0;
        while(rd<14){
          readcounter++;
          rd = HWSERIAL.available();
          if(readcounter>100) break;  //give 10 s to get values
          delay(100);
        }
        
        if(readcounter<100){
          n = HWSERIAL.readBytes((char *)buffer, 14);
          sscanf(buffer, "%4d%2d%2d%2d%2d%2d", &yr, &mo, &dy, &hr, &mn, &sc);
          if(yr>2050 | yr<2016 | mo<1 | mo>12 | dy<1 | dy>31 | hr<0 | hr>23 | mn<0 | mn>59 | sc<0 | sc>59){
            Serial.println("Bad date");
            return;
          }
          
          setTeensyTime(yr, mo, dy, hr, mn, sc);
          startTime = getTeensy3Time() + rec_int;
          stopTime = startTime + rec_dur;
          Serial.println("Time set");
          printTime(getTeensy3Time());
        }
        else
        Serial.println("Timeout: Time not set");
      }
      if(buffer[0] == 'g'){  //get time
        printTime(getTeensy3Time());
        time_t tt = getTeensy3Time();
        HWSERIAL.print(year(tt));
        char sMonth[2], sDay[2], sHour[2], sMinute[2], sSecond[2];
        sprintf(sMonth, "%02d", month(tt));
        sprintf(sDay, "%02d", day(tt));
        sprintf(sHour, "%02d", hour(tt));
        sprintf(sMinute, "%02d", minute(tt));
        sprintf(sSecond, "%02d", second(tt));
        HWSERIAL.print(sMonth);
        HWSERIAL.print(sDay);
        HWSERIAL.print(sHour);
        HWSERIAL.print(sMinute);
        HWSERIAL.println(sSecond);
        HWSERIAL.flush();


      }
  }
}

void FileInit()
{
   t = getTeensy3Time();
   // open file 
   sprintf(filename,"%02d%02d%02d%02d.wav", month(t), day(t), hour(t), minute(t));  //filename is MMDDHHMM.SS
   frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);
   Serial.println(filename);
   delay(100);
   
   while (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    frec = SD.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(10);
   }
  
  //intialize .wav file header
  sprintf(wav_hdr.rId,"RIFF");
  wav_hdr.rLen=36;
  sprintf(wav_hdr.wId,"WAVE");
  sprintf(wav_hdr.fId,"fmt ");
  wav_hdr.fLen=0x10;
  wav_hdr.nFormatTag=1;
  wav_hdr.nChannels=1;
  wav_hdr.nSamplesPerSec=audio_srate;
  wav_hdr.nAvgBytesPerSec=audio_srate*2;
  wav_hdr.nBlockAlign=2;
  wav_hdr.nBitsPerSamples=16;
  sprintf(wav_hdr.dId,"data");
  wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
  wav_hdr.dLen = nbufs_per_file * 256 * 2;
  t = getTeensy3Time();

  frec.write((uint8_t *)&wav_hdr,44);
  Serial.print("Buffers: ");
  Serial.println(nbufs_per_file);
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
 /* t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
  *
   */
}

void AudioInit(){
  // Enable the audio shield, select input, and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(3);
  //  0: 3.12 Volts p-p
//  1: 2.63 Volts p-p
//  2: 2.22 Volts p-p
//  3: 1.87 Volts p-p
//  4: 1.58 Volts p-p
//  5: 1.33 Volts p-p
//  6: 1.11 Volts p-p
//  7: 0.94 Volts p-p
//  8: 0.79 Volts p-p
//  9: 0.67 Volts p-p
// 10: 0.56 Volts p-p
// 11: 0.48 Volts p-p
// 12: 0.40 Volts p-p
// 13: 0.34 Volts p-p
// 14: 0.29 Volts p-p
// 15: 0.24 Volts p-p
  sgtl5000_1.autoVolumeDisable();
  sgtl5000_1.audioProcessorDisable();
}

void checkDielTime(){
  unsigned int startMinutes = (startHour * 60) + (startMinute);
  unsigned int endMinutes = (endHour * 60) + (endMinute );
  unsigned int startTimeMinutes =  (hour(startTime) * 60) + (minute(startTime));
  
  tmElements_t tmStart;
  tmStart.Year = year(startTime) - 1970;
  tmStart.Month = month(startTime);
  tmStart.Day = day(startTime);
  // check if next startTime is between startMinutes and endMinutes
  // e.g. 06:00 - 12:00 or 
  if(startMinutes<endMinutes){
     if ((startTimeMinutes < startMinutes) | (startTimeMinutes > endMinutes)){
       // set startTime to startHour startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
       }
     }
  else{  // e.g. 23:00 - 06:00
    if((startTimeMinutes<startMinutes) & (startTimeMinutes>endMinutes)){
      // set startTime to startHour:startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
    }
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void setTeensyTime(int yr, int mo, int dy, int hr, int mn, int sc){
  tmElements_t tm;
  tm.Year = yr - 1970;
  tm.Month = mo;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;
  time_t newtime;
  newtime = makeTime(tm); 
  Teensy3Clock.set(newtime); 
}


unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 
  
// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
    int i;
    unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    unsigned long Ticks = 0;

    long yearsSince = tm->year + 30; // Years since 1970
    long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated

    if((!(tm->year%4)) && (tm->month>2))
            Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

    // Calculate Year Ticks
    Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
    Ticks += numLeaps * SECONDS_IN_LEAP;

    // Calculate Month Ticks
    for(i=0; i < tm->month-1; i++){
         Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
    }

    // Calculate Day Ticks
    Ticks += (tm->mday - 1) * SECONDS_IN_DAY;

    // Calculate Time Ticks CHANGES ARE HERE
    Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
    Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
    Ticks += tm->sec;

    return Ticks;
}

void printTime(time_t t){
  Serial.print(year(t));
  Serial.print('-');
  Serial.print(month(t));
  Serial.print('-');
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  Serial.print(':');
  Serial.print(minute(t));
  Serial.print(':');
  Serial.println(second(t));
}
