/*  
 *   Pulse Program by Kenny Lozowski for Paul Berkenbosch's
 *   HeartBurn Flaming Heart Piece, version 0.2 -- Summer 2016
 *   kennylozowski@gmail.com
*/

#define RUNNING_AVERAGE_SIZE 8 //the number of samples to include in the running average
#define HB_AVERAGE_SIZE 3 //number of samples to include in the heartbeat average
#define SAMPLE_FREQUENCY_DIVISOR 50 // the interrupt clock is set to 500 times by default, I found it more straightforward to sample only ten times a second
#define PULSE_PIN 0 // Pulse Sensor purple wire connected to analog pin 0
#define DEBUG_PLOT_RAW false //turn this on to check a debug plot of the raw signal using the serial plotter, you should turn all other debugging off to do so
#define DEBUG_PLOT_FILTERED false //turn this on to check a debug plot of the filtered signal the divisor using the serial plotter, you should turn all other debugging off to do so
#define DEBUG_PLOT_DIVISOR true //turn this on to check a debug plot of the heartbeat after the divisor using the serial plotter, you should turn all other debugging off to do so
#define DEBUG_PULSE false // turn this on to debug when we read pulses
#define DEBUG_HEARTBEAT_LOSTFOUND false //turn this on to debug when we find and lose the heartbeat;
#define DEBUG_HEARTBEAT false //turn this on to print debug text on the heartbeat itself
#define MIN_PULSE_HEIGHT 150 //the minimum acceptable pulse height to consider a pulse
#define MAX_PULSE_HEIGHT 700 // maximum acceptable pulse height;
#define MIN_HEARTBEAT 65 // minimum acceptable heartbeat
#define MAX_HEARTBEAT 180 //maximum acceptable heartbeat
#define HEARTBEAT_EVALUATE_FREQUENCY 100 //how often to evaluate heartbeat on off
#define HEARTBEAT_EVALUATE_DURATION 3000 //duration of evaluation of heartbeat on off
#define NUM_HEARTBEAT_TIMES 10 // number of heartbeats to keep in array
#define PRIMARY_BEAT_RELAY_PIN 4 // primary beat pin for evil mad scientist relay shield
#define SECONDARY_BEAT_RELAY_PIN 8 // secondary beat pin for evil mad scientist relay shield
#define TRIGGER_RELAYS true // do we want to actually trigger relays?
#define HAS_SECONDARY_BEAT true // do we want a secondary beat
#define SECONDARY_BEAT_OFFSET 0.25 //secondary beat offset as a ratio to first beat duration
#define PRIMARY_BEAT_DURATION 70 //primary beat duration in milliseconds
#define SECONDARY_BEAT_DURATION 140 //secondary beat duration in milliseconds

volatile int rawSignal;
int filteredSignal, thisSignal, lastSignal;
unsigned long lastPulseTime, thisPulseTime, lastEvaluateTime,lastSwitchTime, lastPrimaryBeat;
unsigned long heartbeatTimes[NUM_HEARTBEAT_TIMES];
int heartbeatLengths[NUM_HEARTBEAT_TIMES];
int heartbeatTimesIndex = 0;
bool primaryRelayOn = false;
bool secondaryRelayOn = false;
bool hasHeartbeat = false;
bool readySecondary = false;
int signalFrequencyCounter = 0;
int avgHBDuration;               

void setup() {
  pinMode(PRIMARY_BEAT_RELAY_PIN,OUTPUT);         // sets the relay pins to output
  pinMode(SECONDARY_BEAT_RELAY_PIN,OUTPUT);         // sets the relay pins to output
  Serial.begin(115200);
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
  thisSignal = 0;
  thisPulseTime = 0;
  lastEvaluateTime = 0;
  lastSwitchTime = 0;
  lastPrimaryBeat = 0;
  for (int i=0;i<NUM_HEARTBEAT_TIMES;i++) {
    heartbeatTimes[i] = 0;
  }
  Serial.println("/*");  
  Serial.println("*   Pulse Program by Kenny Lozowski for Paul Berkenbosch's");
  Serial.println("*   HeartBurn Flaming Heart Piece, 2016");
  Serial.println("*   kennylozowski@gmail.com");
  Serial.println("*/\n\n");
}

void loop() {
  long thisTime = millis();
  if ((thisTime - lastEvaluateTime) > HEARTBEAT_EVALUATE_FREQUENCY) evaluateHeartbeat();
  if (hasHeartbeat && (avgHBDuration > 100) &&  ((thisTime - lastPrimaryBeat) > avgHBDuration)) primaryBeat();
  if (hasHeartbeat && primaryRelayOn && ((thisTime - lastPrimaryBeat) > PRIMARY_BEAT_DURATION)) primaryBeatOff();
  if (hasHeartbeat && readySecondary && HAS_SECONDARY_BEAT && (avgHBDuration > 100) && ((thisTime - lastPrimaryBeat) > (int)(avgHBDuration * SECONDARY_BEAT_OFFSET))) secondaryBeat();
  if (hasHeartbeat && HAS_SECONDARY_BEAT && secondaryRelayOn && ((thisTime - lastPrimaryBeat) > ((int)(avgHBDuration * SECONDARY_BEAT_OFFSET)+ SECONDARY_BEAT_DURATION))) secondaryBeatOff();
}

void primaryBeat() {
  if (TRIGGER_RELAYS) digitalWrite(PRIMARY_BEAT_RELAY_PIN,HIGH);
  if (DEBUG_HEARTBEAT) { 
    Serial.print("PRIMARY  (");
    Serial.print(60000/avgHBDuration);
    Serial.println(" BPM)");
  }
  lastPrimaryBeat = millis();
  readySecondary = true;
  primaryRelayOn = true;
}

void primaryBeatOff() {
   if (TRIGGER_RELAYS) digitalWrite(PRIMARY_BEAT_RELAY_PIN,LOW);
   primaryRelayOn = false;
}

void secondaryBeat() {
  if (TRIGGER_RELAYS) digitalWrite(SECONDARY_BEAT_RELAY_PIN,HIGH);
  if (DEBUG_HEARTBEAT) { 
    Serial.println("secondary");
  }
  readySecondary = false;
  secondaryRelayOn = true;
}

void secondaryBeatOff() {
   if (TRIGGER_RELAYS) digitalWrite(SECONDARY_BEAT_RELAY_PIN,LOW);
   primaryRelayOn = false;
   secondaryRelayOn = false;
}

void evaluateHeartbeat() {
  lastEvaluateTime = millis();
  int pulseCount = 0;
  for (int i=0;i<NUM_HEARTBEAT_TIMES;i++) {
    if ((lastEvaluateTime - heartbeatTimes[i]) < HEARTBEAT_EVALUATE_DURATION) pulseCount++;
  }
  bool heartbeatValid = ((pulseCount >= (((float)HEARTBEAT_EVALUATE_DURATION * (float)MIN_HEARTBEAT) / 60000.0)) && (pulseCount <= (((float)HEARTBEAT_EVALUATE_DURATION * (float)MAX_HEARTBEAT) / 60000.0)));
  if (heartbeatValid && !hasHeartbeat && ((lastEvaluateTime - lastSwitchTime) > (HEARTBEAT_EVALUATE_DURATION+1000))) {
    hasHeartbeat = true;
    if (DEBUG_HEARTBEAT_LOSTFOUND) Serial.println("FOUND HEARTBEAT");
    lastSwitchTime = lastEvaluateTime;
  }
  if (!heartbeatValid && hasHeartbeat && ((lastEvaluateTime - lastSwitchTime) > (HEARTBEAT_EVALUATE_DURATION+1000))) {
    hasHeartbeat = false;
    if (DEBUG_HEARTBEAT_LOSTFOUND) Serial.println("LOST HEARTBEAT");
    lastSwitchTime = lastEvaluateTime;
    heartbeatTimesIndex = 0;
    avgHBDuration = hbRunningAverage(0,true);
  }
  pulseCount = 0;
 
}

void interruptSetup() {     
  TCCR2A = 0x02;  
  TCCR2B = 0x06; 
  OCR2A = 0X7C;    
  TIMSK2 = 0x02;     
  sei();
} 

ISR(TIMER2_COMPA_vect) {                         
  cli();                               
  rawSignal = analogRead(PULSE_PIN);
  filteredSignal = filterRunningAverage(rawSignal);
  if (DEBUG_PLOT_RAW) Serial.println(rawSignal);
  if (DEBUG_PLOT_FILTERED) Serial.println(filteredSignal);
  signalFrequencyCounter++;
  if (signalFrequencyCounter == SAMPLE_FREQUENCY_DIVISOR) {
    lastSignal = thisSignal; 
    thisSignal = filteredSignal;
    recordSignal();
    signalFrequencyCounter = 0;
  }
  sei();                                   
}

int filterRunningAverage(int M) {
  static int LM[RUNNING_AVERAGE_SIZE];    
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % RUNNING_AVERAGE_SIZE;
  if (count < RUNNING_AVERAGE_SIZE) count++;
  return sum / count;
}

int hbRunningAverage(long M, bool avReset) {
  static long LM[HB_AVERAGE_SIZE];    
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;
  if (avReset) {
    index = 0;
    sum = 0;
    count = 0;
    for (int i=0;i<HB_AVERAGE_SIZE;i++) {
      LM[i] = 0;
    }
    return 0;
  } else {
    sum -= LM[index];
    LM[index] = M;
    sum += LM[index];
    index++;
    index = index % HB_AVERAGE_SIZE;
    if (count < HB_AVERAGE_SIZE) count++;
    return sum / count;
  }
}

void recordSignal() {
  if (DEBUG_PLOT_DIVISOR) Serial.println(thisSignal);
  int signalHeight = thisSignal - lastSignal;
  if ((signalHeight >= MIN_PULSE_HEIGHT) && (signalHeight <= MAX_PULSE_HEIGHT)) {
    lastPulseTime = thisPulseTime;
    thisPulseTime = millis();
    if (hasHeartbeat) avgHBDuration = hbRunningAverage((thisPulseTime - lastPulseTime),false);
    heartbeatTimes[heartbeatTimesIndex] = thisPulseTime;
    heartbeatTimesIndex++;
    if (heartbeatTimesIndex >= NUM_HEARTBEAT_TIMES) heartbeatTimesIndex = 0;
    if (DEBUG_PULSE) Serial.println("PULSE");
  }
}





