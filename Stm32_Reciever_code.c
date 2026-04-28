/* USER CODE BEGIN Header */ 
/** 
: main.c 
****************************************************************************** 
* @file           
* @brief          
* @attention      
: Full 4-Way Traffic Control with Priority Emergency Preemption. 
: **USART2 is Debug/Putty (PA2/PA3 @ 115200), USART1 is HC-12 Rx (PA9/PA10 @ 9600)** 
* NOTE: TIM2 must be configured in CubeMX for 1ms interrupt and its global interrupt enabled. 
* Priority is determined by the character at packet index 2 ('3' > '2'). 
****************************************************************************** 
* 
* Cycle Logic: 
* Default Sequence: Green -> Yellow (Clearing Lane) -> Red+Yellow (Next Lane Preparation) -> Next Green 
****************************************************************************** 
*/ 
/* USER CODE END Header */ 
/* Includes ------------------------------------------------------------------*/ 
#include "main.h" 
/* Private includes ----------------------------------------------------------*/ 
/* USER CODE BEGIN Includes */ 
#include <string.h> 
#include <stdio.h> 
#include <stdbool.h> 
#include <limits.h> 
#include "stm32f1xx_hal_tim.h" 
/* USER CODE END Includes */ 
/* Private typedef -----------------------------------------------------------*/ 
/* USER CODE BEGIN PTD */ 
// --- HC-12 Communication Definitions --- 
#define START_DELIMITER '$' 
#define STOP_DELIMITER  '\n' 
#define PACKET_SIZE     8 
// --- Traffic FSM Definitions --- 
typedef enum { 
STATE_DEFAULT_N, 
STATE_DEFAULT_E, 
STATE_DEFAULT_S, 
STATE_DEFAULT_W, 
STATE_DEFAULT_CLEAR,    // Current Lane Yellow 
STATE_DEFAULT_PREPARE,  // Next Lane Red + Yellow 
STATE_EMERGENCY_CLEAR_TO_RED, 
STATE_EMERGENCY_GREEN, 
STATE_EMERGENCY_YELLOW, 
STATE_EMERGENCY_RESTORE 
} TrafficState; 
// --- Emergency Request Queue --- 
typedef struct { 
char lane_id; 
char priority;          
// Holds the character from the packet ('3' or '2') 
uint32_t arrival_time; 
bool is_active; 
} EmergencyRequest; 
/* USER CODE END PTD */ 
/* Private define ------------------------------------------------------------*/ 
/* USER CODE BEGIN PD */ 
// --- Time Definitions (ms) --- 
#define DEFAULT_GREEN_TIME      8000 
#define DEFAULT_YELLOW_TIME     2000 
#define DEFAULT_ALL_RED_TIME    1000 // Time used for Red+Yellow warning 
#define EMERGENCY_PASS_TIME     10000 
// --- GPIO Definitions (Assumed PB0-PB11 for R,Y,G) --- 
#define N_RED_Port  GPIOB 
#define N_RED_Pin   GPIO_PIN_0 
#define N_YEL_Port  GPIOB 
#define N_YEL_Pin   GPIO_PIN_1 
#define N_GRN_Port  GPIOB 
#define N_GRN_Pin   GPIO_PIN_2 
#define E_RED_Port  GPIOB 
#define E_RED_Pin   GPIO_PIN_3 
#define E_YEL_Port  GPIOB 
#define E_YEL_Pin   GPIO_PIN_4 
#define E_GRN_Port  GPIOB 
#define E_GRN_Pin   GPIO_PIN_5 
#define S_RED_Port  GPIOB 
#define S_RED_Pin   GPIO_PIN_6 
#define S_YEL_Port  GPIOB 
#define S_YEL_Pin   GPIO_PIN_7 
#define S_GRN_Port  GPIOB 
#define S_GRN_Pin   GPIO_PIN_8 
#define W_RED_Port  GPIOB 
#define W_RED_Pin   GPIO_PIN_9 
#define W_YEL_Port  GPIOB 
#define W_YEL_Pin   GPIO_PIN_10 
#define W_GRN_Port  GPIOB 
#define W_GRN_Pin   GPIO_PIN_11 
/* USER CODE END PD */ 
/* Private macro -------------------------------------------------------------*/ 
/* USER CODE BEGIN PM */ 
/* USER CODE END PM */ 
/* Private variables ---------------------------------------------------------*/ 
UART_HandleTypeDef huart1; // USART1: HC-12 Rx 
UART_HandleTypeDef huart2; // USART2: Debug Tx 
TIM_HandleTypeDef htim2; // Timer 2 Handle 
/* USER CODE BEGIN PV */ 
// --- Communication Variables --- 
static volatile uint8_t RxData = 0; 
static uint8_t RxPacketBuffer[PACKET_SIZE]; 
static uint8_t RxIndex = 0; 
static bool ReceivingPacket = false; 
static volatile GPIO_PinState led_state = GPIO_PIN_RESET; 
// --- Traffic FSM Variables --- 
TrafficState current_state = STATE_DEFAULT_N; 
TrafficState last_default_state = STATE_DEFAULT_N; 
uint32_t state_start_time = 0; 
volatile uint32_t g_timer_ms = 0; // Global non-blocking time tick 
// --- Emergency Queue Variables --- 
EmergencyRequest emergency_queue[4] = {0}; 
char current_emergency_lane = '0'; // The lane currently active in emergency green 
/* USER CODE END PV */ 
/* Private function prototypes -----------------------------------------------*/ 
void SystemClock_Config(void); 
static void MX_GPIO_Init(void); 
static void MX_USART1_UART_Init(void); 
static void MX_USART2_UART_Init(void); 
void MX_TIM2_Init(void); 
// Traffic Control Prototypes 
static void AllRed(void); 
static void SetLane(char lane_id, GPIO_PinState R, GPIO_PinState Y, GPIO_PinState G); 
static void TrafficFSM(void); 
static void DebugTransmit(const char *msg); 
                    
 
// HC-12 Prototypes 
void Process_Emergency_Packet(uint8_t *packet); 
uint8_t HexToByte(uint8_t hex_char); 
uint8_t Calculate_Checksum(uint8_t type, uint8_t priority, uint8_t lane); 
static bool CheckAndEnqueueRequest(char lane_id, char priority); 
static char FindHighestPriorityRequest(void); 
 
/* USER CODE BEGIN PFP */ 
/* USER CODE END PFP */ 
 
/* Private user code ---------------------------------------------------------*/ 
/* USER CODE BEGIN 0 */ 
 
// --- UTILITY FUNCTIONS (INCLUDING TIMER CALLBACK) --- 
 
static bool TimeElapsed(uint32_t duration) 
{ 
    return (g_timer_ms - state_start_time) >= duration; 
} 
 
static void DebugTransmit(const char *msg) 
{ 
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100); 
} 
 
// TIM2 Interrupt Handler: The 1ms system tick 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{ 
    if (htim->Instance == TIM2) { 
        g_timer_ms++; 
    } 
} 
 
// --- CHECKSUM & PACKET HELPERS --- 
 
uint8_t HexToByte(uint8_t hex_char) 
{ 
    if (hex_char >= '0' && hex_char <= '9') return hex_char - '0'; 
    if (hex_char >= 'A' && hex_char <= 'F') return hex_char - 'A' + 10; 
    if (hex_char >= 'a' && hex_char <= 'f') return hex_char - 'a' + 10; 
    return 0; 
} 
 
uint8_t Calculate_Checksum(uint8_t type, uint8_t priority, uint8_t lane) 
{ 
    return (uint8_t)(type + priority + lane); 
} 
                    
 
static int GetQueueIndex(char lane_id) { 
    // Lanes '1' (N) through '4' (W) map to array indices 0 through 3 
    if (lane_id >= '1' && lane_id <= '4') return lane_id - '1'; 
    return -1; 
} 
 
static bool CheckAndEnqueueRequest(char lane_id, char priority) 
{ 
    int index = GetQueueIndex(lane_id); 
    if (index == -1) return false; 
 
    if (!emergency_queue[index].is_active) { 
        emergency_queue[index].lane_id = lane_id; 
        emergency_queue[index].priority = priority; 
        emergency_queue[index].arrival_time = g_timer_ms; 
        emergency_queue[index].is_active = true; 
 
        char dbg[50]; 
        snprintf(dbg, sizeof(dbg), "[QUEUE] NEW Request: Lane %c, Prio %c.\r\n", lane_id, priority); 
        DebugTransmit(dbg); 
        return true; 
    } 
 
    // Heartbeat: update arrival time (to keep the priority active) 
    emergency_queue[index].arrival_time = g_timer_ms; 
    return false; 
} 
 
/** 
 * @brief Finds the highest priority vehicle request using the packet's priority character. 
 * Priority is set by the ASCII value of the character: '3' (Ambulance) > '2' (Firetruck). 
 * Ties are broken by FCFS (oldest arrival_time). 
 */ 
static char FindHighestPriorityRequest(void) 
{ 
    char highest_lane = '0'; 
    char highest_priority = '0'; 
    uint32_t oldest_time = ULONG_MAX; 
 
    for (int i = 0; i < 4; i++) { 
        if (emergency_queue[i].is_active) { 
 
            // 1. Priority Check: If current request has higher ASCII priority character 
            if (emergency_queue[i].priority > highest_priority) { 
                highest_priority = emergency_queue[i].priority; 
                highest_lane = emergency_queue[i].lane_id; 
                oldest_time = emergency_queue[i].arrival_time; 
                    
            } 
 
            // 2. Tiebreaker (Same Priority Level): Use First Come First Serve (FCFS) 
            else if (emergency_queue[i].priority == highest_priority) { 
                if (emergency_queue[i].arrival_time < oldest_time) { 
                    oldest_time = emergency_queue[i].arrival_time; 
                    highest_lane = emergency_queue[i].lane_id; 
                } 
            } 
        } 
    } 
    return highest_lane; 
} 
 
 
// --- HC-12 PACKET PROCESSING --- 
 
void Process_Emergency_Packet(uint8_t *packet) 
{ 
    char debug_msg[100]; 
 
    // Packet indices: [0:$][1:Type][2:Prio][3:Lane][4:C1][5:C2][6:\r][7:\n] 
    uint8_t priority_char = packet[2]; // PRIORITY (e.g., '3' or '2') 
    uint8_t lane_id = packet[3];       // LANE ID (e.g., '1' through '4') 
 
    uint8_t received_checksum_char1 = packet[4]; 
    uint8_t received_checksum_char2 = packet[5]; 
    uint8_t reconstructed_checksum = (HexToByte(received_checksum_char1) << 4) | 
HexToByte(received_checksum_char2); 
    uint8_t expected_checksum = Calculate_Checksum(packet[1], priority_char, lane_id); 
 
    if (reconstructed_checksum != expected_checksum) { 
        snprintf(debug_msg, sizeof(debug_msg), "[ERROR] Checksum Mismatch! Exp: %02X, Got: %02X.\r\n", 
expected_checksum, reconstructed_checksum); 
        DebugTransmit(debug_msg); 
        return; 
    } 
 
    DebugTransmit("[VALID] Packet Received. Checking Preemption...\r\n"); 
 
    // Add the request to the queue using the priority character 
    CheckAndEnqueueRequest(lane_id, priority_char); 
 
    // Check if we need to switch to emergency mode 
    if (current_state < STATE_EMERGENCY_CLEAR_TO_RED) { 
        DebugTransmit("[STATE] Default sequence interrupted. Switching to EMERGENCY.\r\n"); 
        // Store the current state (which will be a default state, N, E, S, W, CLEAR, or PREPARE) 
        // If it was CLEAR or PREPARE, it means last_default_state already holds the last green lane. 
                    
        if (current_state == STATE_DEFAULT_N || current_state == STATE_DEFAULT_E || 
            current_state == STATE_DEFAULT_S || current_state == STATE_DEFAULT_W) { 
            last_default_state = current_state; 
        } 
 
        current_state = STATE_EMERGENCY_CLEAR_TO_RED; 
        state_start_time = g_timer_ms; 
    } else if (current_state == STATE_EMERGENCY_GREEN) { 
        // Check for preemption if a higher priority vehicle arrived 
        char winning_lane = FindHighestPriorityRequest(); 
        if (winning_lane != current_emergency_lane) { 
            DebugTransmit("[STATE] Higher Prio or New FCFS Winner arrived. Clearing current lane.\r\n"); 
            // Switch directly to ALL_RED to handle the new request immediately 
            current_state = STATE_EMERGENCY_CLEAR_TO_RED; 
            state_start_time = g_timer_ms; 
        } 
    } 
 
    // Toggle the PA5 LED (Success confirmation) 
    led_state = (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET; 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state); 
} 
 
// --- TRAFFIC CONTROL & FSM FUNCTIONS --- 
 
static void AllRed(void) 
{ 
    HAL_GPIO_WritePin(N_RED_Port,N_RED_Pin,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(N_YEL_Port,N_YEL_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(N_GRN_Port,N_GRN_Pin,GPIO_PIN_RESET); 
 
    HAL_GPIO_WritePin(E_RED_Port,E_RED_Pin,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(E_YEL_Port,E_YEL_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(E_GRN_Port,E_GRN_Pin,GPIO_PIN_RESET); 
 
    HAL_GPIO_WritePin(S_RED_Port,S_RED_Pin,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(S_YEL_Port,S_YEL_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(S_GRN_Port,S_GRN_Pin,GPIO_PIN_RESET); 
 
    HAL_GPIO_WritePin(W_RED_Port,W_RED_Pin,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(W_YEL_Port,W_YEL_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(W_GRN_Port,W_GRN_Pin,GPIO_PIN_RESET); 
} 
 
static void SetLane(char lane_id, GPIO_PinState R, GPIO_PinState Y, GPIO_PinState G) 
{ 
    // Sets all lanes to RED first, then sets the target lane state. 
    AllRed(); 
                    
 
    GPIO_TypeDef* port; 
    uint16_t r_pin, y_pin, g_pin; 
 
    switch (lane_id) { 
        case '1': port = N_RED_Port; r_pin = N_RED_Pin; y_pin = N_YEL_Pin; g_pin = N_GRN_Pin; break; 
        case '2': port = E_RED_Port; r_pin = E_RED_Pin; y_pin = E_YEL_Pin; g_pin = E_GRN_Pin; break; 
        case '3': port = S_RED_Port; r_pin = S_RED_Pin; y_pin = S_YEL_Pin; g_pin = S_GRN_Pin; break; 
        case '4': port = W_RED_Port; r_pin = W_RED_Pin; y_pin = W_YEL_Pin; g_pin = W_GRN_Pin; break; 
        default: return; 
    } 
 
    HAL_GPIO_WritePin(port, r_pin, R); 
    HAL_GPIO_WritePin(port, y_pin, Y); 
    HAL_GPIO_WritePin(port, g_pin, G); 
 
    char dbg[30]; 
    snprintf(dbg, sizeof(dbg), "[TRAF] Lane %c -> R%d Y%d G%d\r\n", lane_id, R, Y, G); 
    DebugTransmit(dbg); 
} 
 
// The core non-blocking logic loop 
static void TrafficFSM(void) 
{ 
    switch (current_state) { 
        // ------------------ DEFAULT SEQUENCE ------------------ 
        case STATE_DEFAULT_N: 
            SetLane('1', GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); // N Green 
            if (TimeElapsed(DEFAULT_GREEN_TIME)) { 
                current_state = STATE_DEFAULT_CLEAR; 
                last_default_state = STATE_DEFAULT_N; 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        case STATE_DEFAULT_E: 
            SetLane('2', GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); // E Green 
            if (TimeElapsed(DEFAULT_GREEN_TIME)) { 
                current_state = STATE_DEFAULT_CLEAR; 
                last_default_state = STATE_DEFAULT_E; 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        case STATE_DEFAULT_S: 
            SetLane('3', GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); // S Green 
            if (TimeElapsed(DEFAULT_GREEN_TIME)) { 
                current_state = STATE_DEFAULT_CLEAR; 
                    
                last_default_state = STATE_DEFAULT_S; 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        case STATE_DEFAULT_W: 
            SetLane('4', GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); // W Green 
            if (TimeElapsed(DEFAULT_GREEN_TIME)) { 
                current_state = STATE_DEFAULT_CLEAR; 
                last_default_state = STATE_DEFAULT_W; 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        case STATE_DEFAULT_CLEAR: 
            // 1. Set clearing lane to YELLOW (e.g., N Yellow) 
            if (last_default_state == STATE_DEFAULT_N) { 
                 SetLane('1', GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); 
            } else if (last_default_state == STATE_DEFAULT_E) { 
                 SetLane('2', GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); 
            } else if (last_default_state == STATE_DEFAULT_S) { 
                 SetLane('3', GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); 
            } else if (last_default_state == STATE_DEFAULT_W) { 
                 SetLane('4', GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); 
            } 
 
            // 2. Wait for Yellow time, then transition to the PREPARE state (R+Y) 
            if (TimeElapsed(DEFAULT_YELLOW_TIME)) { 
                current_state = STATE_DEFAULT_PREPARE; 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        case STATE_DEFAULT_PREPARE: // Red + Yellow warning state 
            // 1. Determine the NEXT default state (N->E->S->W->N) 
            TrafficState next_state = (last_default_state == STATE_DEFAULT_W) ? STATE_DEFAULT_N : 
(TrafficState)(last_default_state + 1); 
            char next_lane_id = '0'; 
 
            // Convert the next state enum back to a lane ID character 
            if (next_state == STATE_DEFAULT_N) next_lane_id = '1'; 
            else if (next_state == STATE_DEFAULT_E) next_lane_id = '2'; 
            else if (next_state == STATE_DEFAULT_S) next_lane_id = '3'; 
            else if (next_state == STATE_DEFAULT_W) next_lane_id = '4'; 
 
            // 2. Set the Next Lane to RED+YELLOW (Red ON, Yellow ON, Green OFF) 
            // Note: SetLane() calls AllRed() first, ensuring all other lanes are safe Red. 
            if (next_lane_id != '0') { 
                    
                 SetLane(next_lane_id, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET); 
                 DebugTransmit("[STATE] Preparing Next Lane (R+Y).\r\n"); 
            } else { 
                 AllRed(); // Safety fallback 
            } 
 
            // 3. Wait for the All Red/Preparation Time 
            if (TimeElapsed(DEFAULT_ALL_RED_TIME)) { 
                current_state = next_state; // Transition to the next lane's Green state 
                state_start_time = g_timer_ms; 
            } 
            break; 
 
        // ------------------ EMERGENCY PREEMPTION ------------------ 
        case STATE_EMERGENCY_CLEAR_TO_RED: 
            AllRed(); // Safety step: force all to RED 
            char winning_lane_check = FindHighestPriorityRequest(); 
 
            if (winning_lane_check == '0') { 
                // Queue is empty -> Restore default cycle 
                if (TimeElapsed(DEFAULT_ALL_RED_TIME)) { 
                    current_state = STATE_EMERGENCY_RESTORE; 
                    state_start_time = g_timer_ms; 
                    DebugTransmit("[STATE] Emergency queue empty. Restoring default cycle.\r\n"); 
                } 
            } 
            // Wait for safe ALL_RED time, then proceed to emergency green for the new winner 
            else if (TimeElapsed(DEFAULT_ALL_RED_TIME)) { 
                current_emergency_lane = winning_lane_check; // Set the new winner 
                current_state = STATE_EMERGENCY_GREEN; 
                state_start_time = g_timer_ms; 
                DebugTransmit("[STATE] Next Emergency vehicle found. Proceeding to Green.\r\n"); 
            } 
            break; 
 
 
        case STATE_EMERGENCY_GREEN: 
            // Set the winning lane green 
            SetLane(current_emergency_lane, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET); 
 
            // Check if the pass time has elapsed 
            if (TimeElapsed(EMERGENCY_PASS_TIME)) { 
                // Vehicle pass time complete. Transition to YELLOW. 
                current_state = STATE_EMERGENCY_YELLOW; 
                state_start_time = g_timer_ms; 
                DebugTransmit("[STATE] Emergency pass time complete. Switching to YELLOW.\r\n"); 
            } 
            break; 
                    
 
        case STATE_EMERGENCY_YELLOW: 
            // The lane that was just emergency green turns YELLOW 
            SetLane(current_emergency_lane, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET); 
 
            if (TimeElapsed(DEFAULT_YELLOW_TIME)) { 
                // Wait for Yellow time, then transition to All-Red, and remove the completed request 
                int index = GetQueueIndex(current_emergency_lane); 
                if (index != -1) { 
                    emergency_queue[index].is_active = false; // Deactivate the request 
                } 
 
                current_state = STATE_EMERGENCY_CLEAR_TO_RED; 
                state_start_time = g_timer_ms; 
                DebugTransmit("[STATE] Emergency Yellow complete. Checking queue status.\r\n"); 
            } 
            break; 
 
 
        case STATE_EMERGENCY_RESTORE: 
            AllRed(); 
            if (TimeElapsed(DEFAULT_ALL_RED_TIME)) { 
                // Resume where we left off, going to the PREPARE state to set up the R+Y for the next expected default 
lane 
                current_state = STATE_DEFAULT_PREPARE; 
                state_start_time = g_timer_ms; 
                DebugTransmit("[STATE] Resuming default sequence via PREPARE state.\r\n"); 
            } 
            break; 
    } 
} 
 
 
/* USER CODE END 0 */ 
 
/** 
  * @brief  The application entry point. 
  * @retval int 
  */ 
int main(void) 
{ 
    HAL_Init(); 
    SystemClock_Config(); 
 
    MX_GPIO_Init(); 
    MX_USART1_UART_Init(); 
    MX_USART2_UART_Init(); 
    MX_TIM2_Init(); 
                    
 
    HAL_TIM_Base_Start_IT(&htim2); 
 
    /* USER CODE BEGIN 2 */ 
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&RxData, 1); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
    DebugTransmit("STM32 System Init: Ready. Starting Default Traffic.\r\n"); 
    state_start_time = g_timer_ms; 
    /* USER CODE END 2 */ 
 
    /* Infinite loop: Runs the Non-Blocking State Machine */ 
    while (1) 
    { 
        TrafficFSM(); 
    } 
} 
 
 
// --- BOILERPLATE FUNCTIONS (Required for compilation) --- 
 
void SystemClock_Config(void) 
{ 
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; 
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; 
 
    // Assumed standard 72MHz clock setup 
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; 
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; 
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1; 
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; 
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; 
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; 
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; 
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) { Error_Handler(); } 
 
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK 
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; 
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; 
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; 
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; 
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK) { Error_Handler(); } 
} 
 
/** 
  * @brief TIM2 Initialization: Configured for 1ms interrupt 
  */ 
void MX_TIM2_Init(void) 
                    
{ 
  TIM_ClockConfigTypeDef sClockSourceConfig = {0}; 
  TIM_MasterConfigTypeDef sMasterConfig = {0}; 
 
  htim2.Instance = TIM2; 
  // Setting Prescaler for 72MHz clock to achieve 1MHz (72,000,000 / (71+1)) 
  htim2.Init.Prescaler = 71; 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP; 
  // Setting Period for 1MHz clock to achieve 1ms (1,000,000 / (999+1)) 
  htim2.Init.Period = 999; 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; 
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) 
  { 
    Error_Handler(); 
  } 
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; 
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) 
  { 
    Error_Handler(); 
  } 
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; 
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; 
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) 
  { 
    Error_Handler(); 
  } 
} 
 
/** 
  * @brief USART1 Initialization Function (HC-12 Data Link @ 9600 Baud) 
  */ 
static void MX_USART1_UART_Init(void) 
{ 
    huart1.Instance = USART1; 
    huart1.Init.BaudRate = 9600; 
    huart1.Init.WordLength = UART_WORDLENGTH_8B; 
    huart1.Init.StopBits = UART_STOPBITS_1; 
    huart1.Init.Parity = UART_PARITY_NONE; 
    huart1.Init.Mode = UART_MODE_TX_RX; 
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE; 
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; 
    if (HAL_UART_Init(&huart1)!= HAL_OK) { Error_Handler(); } 
} 
 
/** 
  * @brief USART2 Initialization Function (Debug/Putty @ 115200 Baud) 
  */ 
                    
static void MX_USART2_UART_Init(void) 
{ 
    huart2.Instance = USART2; 
    huart2.Init.BaudRate = 115200; 
    huart2.Init.WordLength = UART_WORDLENGTH_8B; 
    huart2.Init.StopBits = UART_STOPBITS_1; 
    huart2.Init.Parity = UART_PARITY_NONE; 
    huart2.Init.Mode = UART_MODE_TX_RX; 
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; 
    huart2.Init.OverSampling = UART_OVERSAMPLING_16; 
    if (HAL_UART_Init(&huart2)!= HAL_OK) { Error_Handler(); } 
} 
 
/** 
  * @brief GPIO Initialization Function 
  */ 
static void MX_GPIO_Init(void) 
{ 
    GPIO_InitTypeDef GPIO_InitStruct = {0}; 
 
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
    __HAL_RCC_GPIOA_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE(); 
 
    // 1. LED Pin Initialization (PA5) 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
    GPIO_InitStruct.Pin = GPIO_PIN_5; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
 
    // 2. USART Pins 
    // PA2 (TX) and PA3 (RX) for USART2 Debug 
    // PA9 (TX) and PA10 (RX) for USART1 HC-12 
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
 
    // 3. Traffic Light Pins (PB0-PB11) 
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                        |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
                    
 
    // Start all lights in the RED state for safety 
    AllRed(); 
} 
 
/* USER CODE BEGIN 4 */ 
 
/* UART Rx complete callback: called by HAL from IRQ context */ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{ 
    // Confirm the interrupt originated from the HC-12 UART instance (USART1) 
    if (huart->Instance == USART1) 
    { 
        uint8_t received_byte = RxData; 
 
        // --- State Machine for Packet Assembly --- 
        if (received_byte == START_DELIMITER) { 
            RxIndex = 0; 
            ReceivingPacket = true; 
            RxPacketBuffer[RxIndex++] = received_byte; 
        } 
        else if (ReceivingPacket) { 
 
            if (RxIndex < PACKET_SIZE) { 
                RxPacketBuffer[RxIndex++] = received_byte; 
 
                if (received_byte == STOP_DELIMITER) { 
                    // Packet complete! Attempt processing. 
                    Process_Emergency_Packet(RxPacketBuffer); 
 
                    RxIndex = 0; 
                    ReceivingPacket = false; 
                } 
            } 
            else { 
                // Buffer overflow (packet too long). Abort. 
                ReceivingPacket = false; 
                RxIndex = 0; 
                HAL_UART_Transmit(&huart2, (uint8_t*)"[ERROR] Buffer Overflow. Resetting.\r\n", 36, 10); 
            } 
        } 
 
        // CRITICAL STEP: Re-arm the receiver for the next byte on USART1 
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&RxData, 1); 
    } 
} 
 
/* Optional: UART error callback to recover on errors */ 
                    
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{ 
    if (huart->Instance == USART1) 
    { 
        if (huart->ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE)) 
        { 
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE); 
            huart->RxState = HAL_UART_STATE_READY; 
            HAL_UART_Receive_IT(&huart1, (uint8_t*)&RxData, 1); 
        } 
    } 
} 
/* USER CODE END 4 */ 
 
/** 
  * @brief  This function is executed in case of error occurrence. 
  * @retval None 
  */ 
void Error_Handler(void) 
{ 
    /* USER CODE BEGIN Error_Handler_Debug */ 
    __disable_irq(); 
    while (1) 
    { 
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
        HAL_Delay(50); 
    } 
    /* USER CODE END Error_Handler_Debug */ 
} 
 
#ifdef  USE_FULL_ASSERT 
void assert_failed(uint8_t *file, uint32_t line) 
{ 
} 
#endif /* USE_FULL_ASSERT */ 