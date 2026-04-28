/* 
* HC-12 Transmitter: 8-Button Priority Encoder (Ambulance & Firetruck) 
* Packet Structure (8 bytes): $ | Type | Priority | Lane | C1 | C2 | \r | \n 
* Type: 'A' (Ambulance), 'F' (Firetruck) 
* Priority: '3' (High for Amb), '2' (Lower for FT) 
*/ 
#include <SoftwareSerial.h> 
// --- GLOBAL CONSTANTS & DEFINITIONS --- 
#define PACKET_SIZE 8  
SoftwareSerial HC12(10, 11); // RX (D10), TX (D11) 
// --- AMBULANCE BUTTONS (Type 'A', Priority '3') --- 
#define BTN_A_N 2  // Lane 1 
#define BTN_A_E 3  // Lane 2 
#define BTN_A_S 4  // Lane 3
#define BTN_A_W 5  // Lane 4 
// --- FIRETRUCK BUTTONS (Type 'F', Priority '2') --- 
#define BTN_F_N 6  // Lane 1 
#define BTN_F_E 7  // Lane 2 
#define BTN_F_S 8  // Lane 3 
#define BTN_F_W 9  // Lane 4 
// --- FIXED PACKET DATA --- 
const char START_DELIMITER = '$'; 
const char TYPE_AMBULANCE = 'A'; 
const char PRIO_AMBULANCE = '3'; // Higher Priority 
const char TYPE_FIRETRUCK = 'F'; 
const char PRIO_FIRETRUCK = '2'; // Lower Priority 
const char STOP_DELIMITER = '\n';  
const char CR_DELIMITER = '\r';    
const int LOOP_DELAY = 10;  
char transmissionState = '0';  
// --- Function Prototypes --- 
byte calculateChecksum(char type, char lane, char priority); 
void sendPacket(char lane, char priority, char type); 
void setup() { 
Serial.begin(9600); 
Serial.println("HC-12 TX: 8-Button Priority Encoder Ready."); 
HC12.begin(9600);  
// Configure all 8 buttons with internal pull-ups (Active LOW) 
for (int i = 2; i <= 9; i++) { 
pinMode(i, INPUT_PULLUP); 
} 
} 
void loop() { 
char currentDirection = '0';  
char currentType = '0';       
char currentPriority = '0';   
// 1. Check Ambulance Buttons 
if (digitalRead(BTN_A_N) == LOW) { currentDirection = '1'; currentType = TYPE_AMBULANCE; currentPriority = 
PRIO_AMBULANCE; }  
else if (digitalRead(BTN_A_E) == LOW) { currentDirection = '2'; currentType = TYPE_AMBULANCE; currentPriority 
= PRIO_AMBULANCE; }  
else if (digitalRead(BTN_A_S) == LOW) { currentDirection = '3'; currentType = TYPE_AMBULANCE; currentPriority 
= PRIO_AMBULANCE; }  
                    
  else if (digitalRead(BTN_A_W) == LOW) { currentDirection = '4'; currentType = TYPE_AMBULANCE; currentPriority 
= PRIO_AMBULANCE; }  
   
  // 2. Check Firetruck Buttons 
  else if (digitalRead(BTN_F_N) == LOW) { currentDirection = '1'; currentType = TYPE_FIRETRUCK; currentPriority = 
PRIO_FIRETRUCK; } 
  else if (digitalRead(BTN_F_E) == LOW) { currentDirection = '2'; currentType = TYPE_FIRETRUCK; currentPriority = 
PRIO_FIRETRUCK; } 
  else if (digitalRead(BTN_F_S) == LOW) { currentDirection = '3'; currentType = TYPE_FIRETRUCK; currentPriority = 
PRIO_FIRETRUCK; } 
  else if (digitalRead(BTN_F_W) == LOW) { currentDirection = '4'; currentType = TYPE_FIRETRUCK; currentPriority = 
PRIO_FIRETRUCK; } 
   
  // 3. Single-Send State Machine 
  if (currentDirection != '0') { 
    if (transmissionState == '0') { 
      sendPacket(currentDirection, currentPriority, currentType); 
      transmissionState = '1'; // Lock the flag 
    } 
  } else { 
    transmissionState = '0'; // Unlock when button is released 
  } 
   
  delay(LOOP_DELAY);  
} 
 
// -------------------- Checksum Calculation -------------------- 
byte calculateChecksum(char type, char lane, char priority) { 
    return (byte)(type + lane + priority); 
} 
 
// -------------------- Build and Send Packet -------------------- 
void sendPacket(char lane, char priority, char type) { 
    byte checksum = calculateChecksum(type, lane, priority); 
     
    // Convert the checksum byte into two ASCII Hex characters (e.g., 165 -> "A5") 
    char checksumHex[3]; 
    sprintf(checksumHex, "%02X", (int)checksum);  
     
    // --- Assemble the 8-byte Transmittable Array --- 
    // Correctly declare the packet as a char array of size 8 
    char packet[PACKET_SIZE];  
     
    // Indexing corrected: [0] is START_DELIMITER, [1] is Type, [2] is Priority, etc. 
    packet[0] = START_DELIMITER; 
    packet[1] = type;        // Index 1: 'A' or 'F' 
    packet[2] = priority;    // Index 2: '3' or '2' 
    packet[3] = lane;        // Index 3: '1', '2', '3', or '4' 
                    
    packet[4] = checksumHex[0]; 
    packet[5] = checksumHex[1]; 
    packet[6] = CR_DELIMITER;    
    packet[7] = STOP_DELIMITER;  
     
    // Transmit the 8 essential bytes  
    HC12.write((uint8_t*)packet, PACKET_SIZE); 
     
    // Debug output:  
    Serial.print("TX: Type:"); 
    Serial.print(type); 
    Serial.print(", Lane:"); 
    Serial.print(lane); 
    Serial.print(", Prio:"); 
    Serial.print(priority); 
    Serial.println(); 
} 