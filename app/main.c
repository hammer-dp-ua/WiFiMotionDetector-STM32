/**
 * unsigned char  uint8_t
 * unsigned short uint16_t
 * unsigned int   uint32_t
 */
#include "stm32f0xx.h"
#include "arm_math.h"
#include "stdlib.h"

#define CLOCK_SPEED 16000000
#define USART_BAUD_RATE 115200

#define USART1_TX_DMA_CHANNEL DMA1_Channel2
#define USART1_TDR_ADDRESS (unsigned int)(&(USART1->TDR))
#define NETWORK_STATUS_LED_PIN GPIO_Pin_1
#define NETWORK_STATUS_LED_PORT GPIOA
#define SERVER_AVAILABILITI_LED_PIN GPIO_Pin_2
#define SERVER_AVAILABILITI_LED_PORT GPIOA
#define MOTION_SENSOR_LED_PIN GPIO_Pin_4
#define MOTION_SENSOR_LED_PORT GPIOA
#define MOTION_SENSOR_INPUT_PIN GPIO_Pin_3
#define MOTION_SENSOR_EXTI_PIN_SOURCE EXTI_PinSource3
#define MOTION_SENSOR_INPUT_PORT GPIOA
#define ESP8266_CONTROL_PIN GPIO_Pin_12

#define USART_DATA_RECEIVED_FLAG 1
#define SERVER_IS_AVAILABLE_FLAG 2
#define SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG 4
#define SENDING_USART_ERRORS_OVERFLOW_FLAG 8

#define GET_VISIBLE_NETWORK_LIST_FLAG 1
#define DISABLE_ECHO_FLAG 2
#define CONNECT_TO_NETWORK_FLAG 4
#define CONNECTION_STATUS_FLAG 8
#define GET_CONNECTION_STATUS_AND_CONNECT_FLAG 16
#define GET_OWN_IP_ADDRESS_FLAG 32
#define SET_OWN_IP_ADDRESS_FLAG 64
#define CONNECT_TO_SERVER_FLAG 128
#define BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG 256
#define GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG 512
#define POST_REQUEST_SENT_FLAG 1024
#define GET_CURRENT_DEFAULT_WIFI_MODE_FLAG 2048
#define SET_DEFAULT_STATION_WIFI_MODE_FLAG 4096
#define CLOSE_CONNECTION_FLAG 8192
#define GET_CONNECTION_STATUS_FLAG 16384
#define SERVER_AVAILABILITY_RESPONSE_FLAG 32768
#define GET_SERVER_AVAILABILITY_FLAG 65536
#define ALARM_RESPONSE_FLAG 131072
#define SEND_ALARM_FLAG 262144

#define USART_DATA_RECEIVED_BUFFER_SIZE 500
#define PIPED_REQUEST_COMMANDS_TO_SEND_SIZE 3
#define PIPED_REQUEST_CIPSTART_COMMAND_INDEX 0
#define PIPED_REQUEST_CIPSEND_COMMAND_INDEX 1
#define PIPED_REQUEST_INDEX 2

#define PIPED_TASKS_TO_SEND_SIZE 10
#define PIPED_TASKS_HISTORY_SIZE 10
#define DEFAULT_ACCESS_POINT_GAIN_SIZE 4

#define TIMER3_PERIOD_MS 0.13f
#define TIMER3_PERIOD_SEC (TIMER3_PERIOD_MS / 1000)
#define TIMER3_100MS (unsigned short)(100 / TIMER3_PERIOD_MS)
#define TIMER6_1S 10
#define TIMER6_2S 20
#define TIMER6_5S 50
#define TIMER6_10S 100
#define TIMER6_30S 300
#define TIMER6_60S 600
#define TIMER6_10MIN 6000

#define EXECUTE_FUNCTION 1
#define DO_NOT_EXECUTE_FUNCTION 0

unsigned int piped_tasks_to_send_g[PIPED_TASKS_TO_SEND_SIZE];
unsigned int piped_tasks_history_g[PIPED_TASKS_HISTORY_SIZE];
char *piped_request_commands_to_send_g[PIPED_REQUEST_COMMANDS_TO_SEND_SIZE]; // AT+CIPSTART="TCP","address",port; AT+CIPSEND=bytes_to_send; request
unsigned int sent_flag_g;
unsigned int successfully_received_flags_g;
unsigned int general_flags_g;

char USART_OK[] __attribute__ ((section(".text.const"))) = "OK";
char USART_ERROR[] __attribute__ ((section(".text.const"))) = "ERROR";
char DEFAULT_ACCESS_POINT_NAME[] __attribute__ ((section(".text.const"))) = "Asus";
char DEFAULT_ACCESS_POINT_PASSWORD[] __attribute__ ((section(".text.const"))) = "";
char ESP8226_REQUEST_DISABLE_ECHO[] __attribute__ ((section(".text.const"))) = "ATE0\r\n";
char ESP8226_RESPONSE_BUSY[] __attribute__ ((section(".text.const"))) = "busy";
char ESP8226_REQUEST_GET_VISIBLE_NETWORK_LIST[] __attribute__ ((section(".text.const"))) = "AT+CWLAP\r\n";
char ESP8226_RESPONSE_VISIBLE_NETWORK_LIST_PREFIX[] __attribute__ ((section(".text.const"))) = "+CWLAP:";
char ESP8226_REQUEST_GET_CONNECTION_STATUS[] __attribute__ ((section(".text.const"))) = "AT+CWJAP?\r\n";
char ESP8226_RESPONSE_NOT_CONNECTED_STATUS[] __attribute__ ((section(".text.const"))) = "No AP";
char ESP8226_REQUEST_CONNECT_TO_NETWORK_AND_SAVE[] __attribute__ ((section(".text.const"))) = "AT+CWJAP_DEF=\"<1>\",\"<2>\"\r\n";
char ESP8226_REQUEST_GET_VERSION_ID[] __attribute__ ((section(".text.const"))) = "AT+GMR\r\n";
char ESP8226_RESPONSE_CONNECTED[] __attribute__ ((section(".text.const"))) = "CONNECT";
char ESP8226_CONNECTION_CLOSED[] __attribute__ ((section(".text.const"))) = "CLOSED";
char ESP8226_REQUEST_CONNECT_TO_SERVER[] __attribute__ ((section(".text.const"))) = "AT+CIPSTART=\"TCP\",\"<1>\",<2>\r\n";
char ESP8226_REQUEST_DISCONNECT_FROM_SERVER[] __attribute__ ((section(".text.const"))) = "AT+CIPCLOSE\r\n";
char ESP8226_REQUEST_SERVER_PING[] __attribute__ ((section(".text.const"))) = "AT+PING=\"<1>\"\r\n";
char ESP8226_REQUEST_START_SENDING[] __attribute__ ((section(".text.const"))) = "AT+CIPSEND=<1>\r\n";
char ESP8226_RESPONSE_START_SENDING_READY[] __attribute__ ((section(".text.const"))) = ">";
char ESP8226_RESPONSE_SENDING[] __attribute__ ((section(".text.const"))) = "busy s...";
char ESP8226_RESPONSE_SUCCSESSFULLY_SENT[] __attribute__ ((section(".text.const"))) = "SEND OK";
char ESP8226_RESPONSE_ALREADY_CONNECTED[] __attribute__ ((section(".text.const"))) = "ALREADY CONNECTED";
char ESP8226_RESPONSE_PREFIX[] __attribute__ ((section(".text.const"))) = "+IPD";
char ESP8226_OWN_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "192.168.0.20";
char ESP8226_SERVER_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "192.168.0.2";
char ESP8226_SERVER_PORT[] __attribute__ ((section(".text.const"))) = "80";
char ESP8226_REQUEST_GET_CURRENT_DEFAULT_WIFI_MODE[] __attribute__ ((section(".text.const"))) = "AT+CWMODE_DEF?\r\n";
char ESP8226_RESPONSE_WIFI_MODE_PREFIX[] __attribute__ ((section(".text.const"))) = "+CWMODE_DEF:";
char ESP8226_RESPONSE_WIFI_STATION_MODE[] __attribute__ ((section(".text.const"))) = "1";
char ESP8226_REQUEST_SET_DEFAULT_STATION_WIFI_MODE[] __attribute__ ((section(".text.const"))) = "AT+CWMODE_DEF=1\r\n";
char ESP8226_REQUEST_GET_OWN_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "AT+CIPSTA_DEF?\r\n";
char ESP8226_RESPONSE_CURRENT_OWN_IP_ADDRESS_PREFIX[] __attribute__ ((section(".text.const"))) = "+CIPSTA_DEF:ip:";
char ESP8226_REQUEST_SET_OWN_IP_ADDRESS[] __attribute__ ((section(".text.const"))) = "AT+CIPSTA_DEF=\"<1>\"\r\n";
char ESP8226_REQUEST_GET_SERVER_AVAILABILITY[] __attribute__ ((section(".text.const"))) =
      "GET /server/esp8266/test HTTP/1.1\r\nHost: <1>\r\nUser-Agent: ESP8266\r\nAccept: application/json\r\nConnection: close\r\n\r\n";
char ESP8226_REQUEST_SEND_STATUS_INFO_AND_GET_SERVER_AVAILABILITY[] __attribute__ ((section(".text.const"))) =
      "POST /server/esp8266/statusInfo HTTP/1.1\r\nContent-Length: <1>\r\nHost: <2>\r\nUser-Agent: ESP8266\r\nContent-Type: application/json\r\nAccept: application/json\r\nConnection: close\r\n\r\n<3>\r\n";
char GAIN_JSON[] __attribute__ ((section(".text.const"))) = "{\"gain\":\"<1>\"}";
char ESP8226_RESPONSE_OK_STATUS_CODE[] __attribute__ ((section(".text.const"))) = "{\"statusCode\":\"OK\"}";
char ESP8226_REQUEST_SEND_ALARM[] __attribute__ ((section(".text.const"))) =
      "GET /server/esp8266/alarm HTTP/1.1\r\nHost: <1>\r\nUser-Agent: ESP8266\r\nAccept: application/json\r\nConnection: close\r\n\r\n";

char *usart_data_to_be_transmitted_buffer_g = NULL;
char usart_data_received_buffer_g[USART_DATA_RECEIVED_BUFFER_SIZE];
char default_access_point_gain_g[DEFAULT_ACCESS_POINT_GAIN_SIZE] = {' ', ' ', ' ', ' '};
volatile unsigned short usart_received_bytes_g;
volatile unsigned int final_task_for_request_resending_g;

void (*send_usart_data_function_g)() = NULL;
void (*on_response_g)() = NULL;
volatile unsigned int send_usart_data_timer_counter_g;
volatile unsigned short send_usart_data_timout_sec_g = 0xFFFF;
volatile unsigned short send_usart_data_errors_counter_g;
volatile unsigned short network_searching_status_led_counter_g;
volatile unsigned short response_timeout_timer_g;
volatile unsigned char esp8266_disabled_counter_g;
volatile unsigned char esp8266_disabled_timer_g = TIMER6_5S;
volatile unsigned char resending_requests_counter_g;
volatile unsigned short checking_connection_status_and_server_availability_counter_g;
volatile unsigned short visible_network_list_counter_g;

volatile unsigned short usart_overrun_errors_counter_g;
volatile unsigned short usart_idle_line_detection_counter_g;
volatile unsigned short usart_noise_detection_counter_g;
volatile unsigned short usart_framing_errors_counter_g;

void Clock_Config();
void Pins_Config();
void TIMER3_Confing();
void TIMER6_Confing();
void EXTERNAL_Interrupt_Config();
void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
unsigned char read_flag_state(unsigned int *flags, unsigned int flag_value);
void DMA_Config();
void USART_Config();
void set_appropriate_successfully_recieved_flag();
void disable_echo();
void get_network_list();
void connect_to_network();
void get_connection_status();
void execute_usart_data_sending(void (*function_to_execute)(), unsigned char timeout, unsigned char execute_function);
void send_usard_data(char string[]);
unsigned char is_usart_response_contains_elements(char *data_to_be_contained[], unsigned char elements_count);
unsigned char is_usart_response_contains_element(char string_to_be_contained[]);
unsigned char contains_string(char being_compared_string[], char string_to_be_contained[]);
void clear_usart_data_received_buffer();
void *set_string_parameters(char string[], char *parameters[]);
unsigned short get_string_length(char string[]);
unsigned int get_current_piped_task_to_send();
void delete_current_piped_task();
void add_piped_task_to_send_into_tail(unsigned int task);
void add_piped_task_to_send_into_head(unsigned int task);
void delete_piped_task(unsigned int task);
void on_successfully_receive_general_actions(unsigned short successfully_received_flag);
void send_http_request(char address[], char port[], char request[], void (*on_response)(), unsigned int final_task);
void resend_usart_get_request(unsigned int final_task);
void resend_usart_get_request_using_global_final_task();
void *short_to_string(unsigned short number);
void *array_to_string(char array[], unsigned char array_length);
void connect_to_server();
void set_bytes_amount_to_send();
void send_request(unsigned int sent_flag_to_set);
void action_on_response();
void get_current_default_wifi_mode();
void set_default_wifi_mode();
void enable_esp8266();
void disable_esp8266();
unsigned char is_esp8266_enabled(unsigned char include_timer);
void clear_piped_request_commands_to_send();
void delete_all_piped_tasks();
void execute_function_for_current_piped_task(unsigned int current_piped_task_to_send, unsigned char send_usart_data_passed_time_sec);
void get_server_avalability();
void get_own_ip_address();
void set_own_ip_address();
void close_connection();
void set_appropriate_successfully_recieved_flag_general_action(unsigned int flag_value, char to_be_contained_in_response[]);
void check_connection_status_and_server_availability(unsigned short current_piped_task_to_send);
void add_piped_task_into_history(unsigned int task);
unsigned int get_last_piped_task_in_history();
void save_default_access_point_gain();

void DMA1_Channel2_3_IRQHandler() {
   DMA_ClearITPendingBit(DMA1_IT_TC2);
}

/**
 * TIM6 and DAC
 */
void TIM6_DAC_IRQHandler() {
   TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

   response_timeout_timer_g++;
   visible_network_list_counter_g++;
   checking_connection_status_and_server_availability_counter_g++;
   if (!is_esp8266_enabled(0)) {
      esp8266_disabled_counter_g++;
   }
   if (esp8266_disabled_timer_g > 0) {
      esp8266_disabled_timer_g--;
   }
}

void TIM3_IRQHandler() {
   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

   // Some error eventually occurs when only the first symbol is exists
   if (usart_received_bytes_g > 1) {
      set_flag(&general_flags_g, USART_DATA_RECEIVED_FLAG);
   }
   usart_received_bytes_g = 0;
   if (send_usart_data_function_g != NULL || resending_requests_counter_g) {
      send_usart_data_timer_counter_g++;
   }
   network_searching_status_led_counter_g++;
}

void EXTI2_3_IRQHandler() {
   if (EXTI_GetITStatus(EXTI_Line3)) {
      EXTI_ClearITPendingBit(EXTI_Line3);
      GPIO_ReadInputDataBit(MOTION_SENSOR_INPUT_PORT, MOTION_SENSOR_INPUT_PIN);
   }
}

void USART1_IRQHandler() {
   if (USART_GetFlagStatus(USART1, USART_FLAG_ORE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_ORE);
      usart_overrun_errors_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_IDLE);
      usart_idle_line_detection_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_NE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_NE);
      usart_noise_detection_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_FE)) {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      USART_ClearFlag(USART1, USART_FLAG_FE);
      usart_framing_errors_counter_g++;
   } else if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
      TIM_SetCounter(TIM3, 0);
      usart_data_received_buffer_g[usart_received_bytes_g] = USART_ReceiveData(USART1);
      usart_received_bytes_g++;

      if (usart_received_bytes_g >= USART_DATA_RECEIVED_BUFFER_SIZE) {
         usart_received_bytes_g = 0;
      }
   }
}

int main() {
   Clock_Config();
   Pins_Config();
   disable_esp8266();
   DMA_Config();
   USART_Config();
   TIMER3_Confing();
   TIMER6_Confing();
   EXTERNAL_Interrupt_Config();

   add_piped_task_to_send_into_tail(DISABLE_ECHO_FLAG);
   add_piped_task_to_send_into_tail(GET_CURRENT_DEFAULT_WIFI_MODE_FLAG);
   add_piped_task_to_send_into_tail(GET_OWN_IP_ADDRESS_FLAG);
   add_piped_task_to_send_into_tail(GET_VISIBLE_NETWORK_LIST_FLAG);
   add_piped_task_to_send_into_tail(GET_CONNECTION_STATUS_AND_CONNECT_FLAG);
   add_piped_task_to_send_into_tail(GET_SERVER_AVAILABILITY_FLAG);

   while (1) {
      if (is_esp8266_enabled(1)) {
         // Seconds
         unsigned char send_usart_data_passed_time_sec = (unsigned char)(TIMER3_PERIOD_SEC * send_usart_data_timer_counter_g);

         if (read_flag_state(&general_flags_g, USART_DATA_RECEIVED_FLAG)) {
            reset_flag(&general_flags_g, USART_DATA_RECEIVED_FLAG);
            send_usart_data_timer_counter_g = 0;

            if (usart_data_to_be_transmitted_buffer_g != NULL) {
               free(usart_data_to_be_transmitted_buffer_g);
               usart_data_to_be_transmitted_buffer_g = NULL;
            }

            set_appropriate_successfully_recieved_flag();
         } else if (send_usart_data_function_g != NULL && send_usart_data_passed_time_sec >= send_usart_data_timout_sec_g) {
            send_usart_data_timer_counter_g = 0;
            if (usart_data_to_be_transmitted_buffer_g != NULL) {
               free(usart_data_to_be_transmitted_buffer_g);
               usart_data_to_be_transmitted_buffer_g = NULL;
            }

            send_usart_data_function_g();
         }

         if (successfully_received_flags_g) {
            if (read_flag_state(&successfully_received_flags_g, DISABLE_ECHO_FLAG)) {
               on_successfully_receive_general_actions(DISABLE_ECHO_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG)) {
               on_successfully_receive_general_actions(GET_CONNECTION_STATUS_AND_CONNECT_FLAG);

               if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
                  // Has already been connected
                  set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
                  GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
               } else if (is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
                  reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
                  // Connect
                  add_piped_task_to_send_into_head(CONNECT_TO_NETWORK_FLAG);
               }
            }
            if (read_flag_state(&successfully_received_flags_g, GET_CONNECTION_STATUS_FLAG)) {
               on_successfully_receive_general_actions(GET_CONNECTION_STATUS_FLAG);

               if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
                  // Has already been connected
                  set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
                  GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
               } else if (is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
                  reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
               }
            }
            if (read_flag_state(&successfully_received_flags_g, GET_VISIBLE_NETWORK_LIST_FLAG)) {
               on_successfully_receive_general_actions(GET_VISIBLE_NETWORK_LIST_FLAG);

               save_default_access_point_gain();
            }
            if (read_flag_state(&successfully_received_flags_g, CONNECT_TO_NETWORK_FLAG)) {
               on_successfully_receive_general_actions(CONNECT_TO_NETWORK_FLAG);

               set_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
               GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
            }
            if (read_flag_state(&successfully_received_flags_g, CONNECT_TO_SERVER_FLAG)) {
               on_successfully_receive_general_actions(CONNECT_TO_SERVER_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG)) {
               on_successfully_receive_general_actions(BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG)) {
               on_successfully_receive_general_actions(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);

               resending_requests_counter_g = 0;
               if (on_response_g != NULL) {
                  on_response_g();
                  on_response_g = NULL;
               }
            }
            if (read_flag_state(&successfully_received_flags_g, GET_CURRENT_DEFAULT_WIFI_MODE_FLAG)) {
               on_successfully_receive_general_actions(GET_CURRENT_DEFAULT_WIFI_MODE_FLAG);

               if (!is_usart_response_contains_element(ESP8226_RESPONSE_WIFI_STATION_MODE)) {
                  add_piped_task_to_send_into_head(SET_DEFAULT_STATION_WIFI_MODE_FLAG);
               }
            }
            if (read_flag_state(&successfully_received_flags_g, SET_DEFAULT_STATION_WIFI_MODE_FLAG)) {
               on_successfully_receive_general_actions(SET_DEFAULT_STATION_WIFI_MODE_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, GET_OWN_IP_ADDRESS_FLAG)) {
               on_successfully_receive_general_actions(GET_OWN_IP_ADDRESS_FLAG);

               unsigned char some_another_ip = !is_usart_response_contains_element(ESP8226_OWN_IP_ADDRESS);
               if (some_another_ip) {
                  add_piped_task_to_send_into_head(SET_OWN_IP_ADDRESS_FLAG);
               }
            }
            if (read_flag_state(&successfully_received_flags_g, SET_OWN_IP_ADDRESS_FLAG)) {
               on_successfully_receive_general_actions(SET_OWN_IP_ADDRESS_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, CLOSE_CONNECTION_FLAG)) {
               on_successfully_receive_general_actions(CLOSE_CONNECTION_FLAG);
            }
            if (read_flag_state(&successfully_received_flags_g, SERVER_AVAILABILITY_RESPONSE_FLAG)) {
               on_successfully_receive_general_actions(SERVER_AVAILABILITY_RESPONSE_FLAG);

               if (is_usart_response_contains_element(ESP8226_RESPONSE_OK_STATUS_CODE)) {
                  GPIO_WriteBit(SERVER_AVAILABILITI_LED_PORT, SERVER_AVAILABILITI_LED_PIN, Bit_SET);
               } else {
                  GPIO_WriteBit(SERVER_AVAILABILITI_LED_PORT, SERVER_AVAILABILITI_LED_PIN, Bit_RESET);
               }
            }
         }

         unsigned int current_piped_task_to_send = get_current_piped_task_to_send();
         execute_function_for_current_piped_task(current_piped_task_to_send, send_usart_data_passed_time_sec);

         /*if (response_timeout_timer > TIMER6_10S && !current_piped_task_to_send) {
            response_timeout_timer = 0;

            if (read_flag_state(&general_flags, SENDING_USART_ERRORS_OVERFLOW_FLAG)) {
               reset_flag(&general_flags, SENDING_USART_ERRORS_OVERFLOW_FLAG);

               char request_template[] = "GET /extjs/5errors.txt HTTP/1.1\r\nHost: {1}\r\nUser-Agent: ESP8266\r\nAccept: text/html\r\nConnection: close\r\n\r\n";
               char *parameter_for_request[] = {"192.168.0.2", NULL};
               char *request = set_string_parameters(request_template, parameter_for_request);
               send_usart_get_request("192.168.0.2", "80", request, action_on_response);
            } else {
               char request_template[] = "GET /extjs/tmp.txt HTTP/1.1\r\nHost: {1}\r\nUser-Agent: ESP8266\r\nAccept: text/html\r\nConnection: close\r\n\r\n";
               char *parameter_for_request[] = {"192.168.0.2", NULL};
               char *request = set_string_parameters(request_template, parameter_for_request);
               send_usart_get_request("192.168.0.2", "80", request, action_on_response);
            }
         }*/

         check_connection_status_and_server_availability(current_piped_task_to_send);

         if (visible_network_list_counter_g >= TIMER6_10MIN) {
            visible_network_list_counter_g = 0;
            add_piped_task_to_send_into_tail(GET_VISIBLE_NETWORK_LIST_FLAG);
         }

         // LED blinking
         if (network_searching_status_led_counter_g >= TIMER3_100MS && !read_flag_state(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG)) {
            network_searching_status_led_counter_g = 0;

            if (GPIO_ReadOutputDataBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN)) {
               GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_RESET);
            } else {
               GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_SET);
            }
         }

         if (send_usart_data_errors_counter_g > 5 || resending_requests_counter_g > 10) {
            set_flag(&general_flags_g, SENDING_USART_ERRORS_OVERFLOW_FLAG);
            send_usart_data_errors_counter_g = 0;
            resending_requests_counter_g = 0;

            delete_all_piped_tasks();
            clear_piped_request_commands_to_send();
            on_response_g = NULL;
            send_usart_data_function_g = NULL;

            successfully_received_flags_g = 0;
            sent_flag_g = 0;
         }
      } else if (esp8266_disabled_counter_g >= TIMER6_1S) {
         esp8266_disabled_counter_g = 0;
         enable_esp8266();
      }
   }
}

void check_connection_status_and_server_availability(unsigned short current_piped_task_to_send) {
   if (checking_connection_status_and_server_availability_counter_g >= TIMER6_60S && !current_piped_task_to_send) {
      checking_connection_status_and_server_availability_counter_g = 0;
      add_piped_task_to_send_into_tail(GET_CONNECTION_STATUS_FLAG);
      add_piped_task_to_send_into_tail(GET_SERVER_AVAILABILITY_FLAG);
   }
}

void execute_function_for_current_piped_task(unsigned int current_piped_task_to_send, unsigned char send_usart_data_passed_time_sec) {
   if (send_usart_data_function_g == NULL && current_piped_task_to_send) {
      if (current_piped_task_to_send == DISABLE_ECHO_FLAG) {
         execute_usart_data_sending(disable_echo, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == GET_CONNECTION_STATUS_AND_CONNECT_FLAG ||
            current_piped_task_to_send == GET_CONNECTION_STATUS_FLAG) {
         execute_usart_data_sending(get_connection_status, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == GET_VISIBLE_NETWORK_LIST_FLAG) {
         execute_usart_data_sending(get_network_list, 30, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == CONNECT_TO_NETWORK_FLAG) {
         execute_usart_data_sending(connect_to_network, 10, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == CONNECT_TO_SERVER_FLAG) {
         execute_usart_data_sending(connect_to_server, 10, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG) {
         execute_usart_data_sending(set_bytes_amount_to_send, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG) {
         final_task_for_request_resending_g = GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG;
         execute_usart_data_sending(resend_usart_get_request_using_global_final_task, 5, DO_NOT_EXECUTE_FUNCTION);
         send_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
      } else if (current_piped_task_to_send == GET_CURRENT_DEFAULT_WIFI_MODE_FLAG) {
         execute_usart_data_sending(get_current_default_wifi_mode, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == SET_DEFAULT_STATION_WIFI_MODE_FLAG) {
         execute_usart_data_sending(set_default_wifi_mode, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == GET_OWN_IP_ADDRESS_FLAG) {
         execute_usart_data_sending(get_own_ip_address, 5, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == SET_OWN_IP_ADDRESS_FLAG) {
         execute_usart_data_sending(set_own_ip_address, 2, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == CLOSE_CONNECTION_FLAG &&
            (!resending_requests_counter_g || (resending_requests_counter_g && send_usart_data_passed_time_sec >= 4))) {
         execute_usart_data_sending(close_connection, 5, EXECUTE_FUNCTION);
      } else if (current_piped_task_to_send == GET_SERVER_AVAILABILITY_FLAG) {
         delete_piped_task(GET_SERVER_AVAILABILITY_FLAG);
         get_server_avalability();
      } else if (current_piped_task_to_send == SERVER_AVAILABILITY_RESPONSE_FLAG) {
         final_task_for_request_resending_g = SERVER_AVAILABILITY_RESPONSE_FLAG;
         execute_usart_data_sending(resend_usart_get_request_using_global_final_task, 5, DO_NOT_EXECUTE_FUNCTION);
         send_request(SERVER_AVAILABILITY_RESPONSE_FLAG);
      }
   }
}

void set_appropriate_successfully_recieved_flag() {
   if (read_flag_state(&sent_flag_g, DISABLE_ECHO_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(DISABLE_ECHO_FLAG, USART_OK);
   }
   if (read_flag_state(&sent_flag_g, GET_VISIBLE_NETWORK_LIST_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(GET_VISIBLE_NETWORK_LIST_FLAG, ESP8226_RESPONSE_VISIBLE_NETWORK_LIST_PREFIX);
   }
   if (read_flag_state(&sent_flag_g, CONNECT_TO_NETWORK_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(CONNECT_TO_NETWORK_FLAG, USART_OK);
   }
   if (read_flag_state(&sent_flag_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG) || read_flag_state(&sent_flag_g, GET_CONNECTION_STATUS_FLAG)) {
      if (is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME) ||
            is_usart_response_contains_element(ESP8226_RESPONSE_NOT_CONNECTED_STATUS)) {
         if (read_flag_state(&sent_flag_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG)) {
            set_flag(&successfully_received_flags_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG);
         } else {
            set_flag(&successfully_received_flags_g, GET_CONNECTION_STATUS_FLAG);
         }
      } else {
         reset_flag(&general_flags_g, SUCCESSUFULLY_CONNECTED_TO_NETWORK_FLAG);
         send_usart_data_errors_counter_g++;
      }

      if (read_flag_state(&sent_flag_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG)) {
         reset_flag(&sent_flag_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG);
      } else {
         reset_flag(&sent_flag_g, GET_CONNECTION_STATUS_FLAG);
      }
   }
   if (read_flag_state(&sent_flag_g, CONNECT_TO_SERVER_FLAG)) {
      reset_flag(&sent_flag_g, CONNECT_TO_SERVER_FLAG);

      char *data_to_be_contained[] = {ESP8226_RESPONSE_CONNECTED, USART_OK};
      if (is_usart_response_contains_elements(data_to_be_contained, 2) ||
            is_usart_response_contains_element(ESP8226_RESPONSE_ALREADY_CONNECTED)) {
         set_flag(&successfully_received_flags_g, CONNECT_TO_SERVER_FLAG);
      } else {
         send_usart_data_errors_counter_g++;
      }
   }
   if (read_flag_state(&sent_flag_g, BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG)) {
      reset_flag(&sent_flag_g, BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);

      if (is_usart_response_contains_element(ESP8226_RESPONSE_START_SENDING_READY)) {
         set_flag(&successfully_received_flags_g, BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
      } else {
         //resend_usart_get_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
         send_usart_data_errors_counter_g++;
      }
   }
   if (read_flag_state(&sent_flag_g, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG)) {
      reset_flag(&sent_flag_g, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);

      char *data_to_be_contained[] = {ESP8226_RESPONSE_SUCCSESSFULLY_SENT, ESP8226_RESPONSE_PREFIX};
      if (is_usart_response_contains_elements(data_to_be_contained, 2)) {
         set_flag(&successfully_received_flags_g, GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
      } else {
         //resend_usart_get_request(GET_REQUEST_SENT_AND_RESPONSE_RECEIVED_FLAG);
         send_usart_data_errors_counter_g++;
      }
   }
   if (read_flag_state(&sent_flag_g, GET_CURRENT_DEFAULT_WIFI_MODE_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(GET_CURRENT_DEFAULT_WIFI_MODE_FLAG, ESP8226_RESPONSE_WIFI_MODE_PREFIX);
   }
   if (read_flag_state(&sent_flag_g, SET_DEFAULT_STATION_WIFI_MODE_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(SET_DEFAULT_STATION_WIFI_MODE_FLAG, USART_OK);
   }
   if (read_flag_state(&sent_flag_g, GET_OWN_IP_ADDRESS_FLAG)) {
      reset_flag(&sent_flag_g, GET_OWN_IP_ADDRESS_FLAG);

      if (is_usart_response_contains_element(ESP8226_OWN_IP_ADDRESS) ||
            is_usart_response_contains_element(ESP8226_RESPONSE_CURRENT_OWN_IP_ADDRESS_PREFIX)) {
         set_flag(&successfully_received_flags_g, GET_OWN_IP_ADDRESS_FLAG);
      } else {
         send_usart_data_errors_counter_g++;
      }
   }
   if (read_flag_state(&sent_flag_g, SET_OWN_IP_ADDRESS_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(SET_OWN_IP_ADDRESS_FLAG, USART_OK);
   }
   if (read_flag_state(&sent_flag_g, CLOSE_CONNECTION_FLAG)) {
      reset_flag(&sent_flag_g, CLOSE_CONNECTION_FLAG);
      set_flag(&successfully_received_flags_g, CLOSE_CONNECTION_FLAG);
   }
   if (read_flag_state(&sent_flag_g, SERVER_AVAILABILITY_RESPONSE_FLAG)) {
      set_appropriate_successfully_recieved_flag_general_action(SERVER_AVAILABILITY_RESPONSE_FLAG, ESP8226_RESPONSE_SUCCSESSFULLY_SENT);
   }
}

void set_appropriate_successfully_recieved_flag_general_action(unsigned int flag_value, char to_be_contained_in_response[]) {
   reset_flag(&sent_flag_g, flag_value);

   if (is_usart_response_contains_element(to_be_contained_in_response)) {
      set_flag(&successfully_received_flags_g, flag_value);
   } else {
      send_usart_data_errors_counter_g++;
   }
}

void get_server_avalability() {
   char *gain = array_to_string(default_access_point_gain_g, DEFAULT_ACCESS_POINT_GAIN_SIZE);
   char *parameter_for_gain[] = {gain, NULL};
   char *gain_json = set_string_parameters(GAIN_JSON, parameter_for_gain);
   unsigned char gain_string_length = (unsigned char) get_string_length(gain_json);
   char *gain_string_length_string = short_to_string(gain_string_length);
   char *parameters_for_request[] = {gain_string_length_string, ESP8226_SERVER_IP_ADDRESS, gain_json, NULL};
   char *request = set_string_parameters(ESP8226_REQUEST_SEND_STATUS_INFO_AND_GET_SERVER_AVAILABILITY, parameters_for_request);

   free(gain);
   free(gain_json);
   free(gain_string_length_string);

   send_http_request(ESP8226_SERVER_IP_ADDRESS, ESP8226_SERVER_PORT, request, NULL, SERVER_AVAILABILITY_RESPONSE_FLAG);
}

void get_own_ip_address() {
   send_usard_data(ESP8226_REQUEST_GET_OWN_IP_ADDRESS);
   set_flag(&sent_flag_g, GET_OWN_IP_ADDRESS_FLAG);
}

void set_own_ip_address() {
   char *parameters[] = {ESP8226_OWN_IP_ADDRESS, NULL};
   usart_data_to_be_transmitted_buffer_g = set_string_parameters(ESP8226_REQUEST_SET_OWN_IP_ADDRESS, parameters);
   send_usard_data(usart_data_to_be_transmitted_buffer_g);
   set_flag(&sent_flag_g, SET_OWN_IP_ADDRESS_FLAG);
}

void close_connection() {
   send_usard_data(ESP8226_REQUEST_DISCONNECT_FROM_SERVER);
   set_flag(&sent_flag_g, CLOSE_CONNECTION_FLAG);
}

void get_current_default_wifi_mode() {
   send_usard_data(ESP8226_REQUEST_GET_CURRENT_DEFAULT_WIFI_MODE);
   set_flag(&sent_flag_g, GET_CURRENT_DEFAULT_WIFI_MODE_FLAG);
}

void set_default_wifi_mode() {
   send_usard_data(ESP8226_REQUEST_SET_DEFAULT_STATION_WIFI_MODE);
   set_flag(&sent_flag_g, SET_DEFAULT_STATION_WIFI_MODE_FLAG);
}

void action_on_response() {
   clear_piped_request_commands_to_send();
}

/**
 * address, port and request shall be allocated with malloc. Later they will be removed with free function
 */
void send_http_request(char address[], char port[], char request[], void (*execute_on_response)(), unsigned int final_task) {
   clear_piped_request_commands_to_send();
   send_usart_data_function_g = NULL;

   char *parameters[] = {address, port, NULL};
   piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX] = set_string_parameters(ESP8226_REQUEST_CONNECT_TO_SERVER, parameters);

   unsigned short request_length = get_string_length(request);
   char *request_length_string = short_to_string(request_length);
   char *start_sending_parameters[] = {request_length_string, NULL};
   piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX] = set_string_parameters(ESP8226_REQUEST_START_SENDING, start_sending_parameters);
   free(request_length_string);

   piped_request_commands_to_send_g[PIPED_REQUEST_INDEX] = request;

   on_response_g = execute_on_response;

   add_piped_task_to_send_into_tail(CONNECT_TO_SERVER_FLAG);
   add_piped_task_to_send_into_tail(BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
   add_piped_task_to_send_into_tail(final_task);
   add_piped_task_to_send_into_tail(CLOSE_CONNECTION_FLAG);
}

void resend_usart_get_request_using_global_final_task() {
   resend_usart_get_request(final_task_for_request_resending_g);
}

void resend_usart_get_request(unsigned int final_task) {
   send_usart_data_timer_counter_g = 0;
   resending_requests_counter_g++;
   send_usart_data_function_g = NULL;
   delete_piped_task(BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
   delete_piped_task(final_task);

   add_piped_task_to_send_into_tail(CLOSE_CONNECTION_FLAG);
   add_piped_task_to_send_into_tail(CONNECT_TO_SERVER_FLAG);
   add_piped_task_to_send_into_tail(BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
   add_piped_task_to_send_into_tail(final_task);
}

void clear_piped_request_commands_to_send() {
   for (unsigned char i = 0; i < PIPED_REQUEST_COMMANDS_TO_SEND_SIZE; i++) {
      char *command = piped_request_commands_to_send_g[i];
      if (command != NULL) {
         free(command);
         piped_request_commands_to_send_g[i] = NULL;
      }
   }
}

void connect_to_server() {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_CIPSTART_COMMAND_INDEX]);
   set_flag(&sent_flag_g, CONNECT_TO_SERVER_FLAG);
}

void set_bytes_amount_to_send() {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_CIPSEND_COMMAND_INDEX]);
   set_flag(&sent_flag_g, BYTES_TO_SEND_IN_REQUEST_IS_SET_FLAG);
}

void send_request(unsigned int sent_flag_to_set) {
   if (piped_request_commands_to_send_g[PIPED_REQUEST_INDEX] == NULL) {
      return;
   }

   send_usard_data(piped_request_commands_to_send_g[PIPED_REQUEST_INDEX]);
   set_flag(&sent_flag_g, sent_flag_to_set);
}

void on_successfully_receive_general_actions(unsigned short successfully_received_flag) {
   if (successfully_received_flag) {
      reset_flag(&successfully_received_flags_g, successfully_received_flag);
   }
   send_usart_data_function_g = NULL;
   send_usart_data_errors_counter_g = 0;
   add_piped_task_into_history(get_current_piped_task_to_send());
   delete_current_piped_task();
}

// +CWLAP:("Asus",-74,...)
void save_default_access_point_gain() {
   if (!is_usart_response_contains_element(DEFAULT_ACCESS_POINT_NAME)) {
      return;
   }

   unsigned char first_comma_is_found = 0;
   char *access_point_starting_position = strstr(usart_data_received_buffer_g, DEFAULT_ACCESS_POINT_NAME);

   if (access_point_starting_position == NULL) {
      for (unsigned char i = 0; i < DEFAULT_ACCESS_POINT_GAIN_SIZE; i++) {
         default_access_point_gain_g[i] = ' ';
      }
   }

   while (*access_point_starting_position != '\0') {
      if (first_comma_is_found && *access_point_starting_position == ',') {
         access_point_starting_position--;
         break;
      }

      if (*access_point_starting_position == ',') {
         first_comma_is_found = 1;
      }
      access_point_starting_position++;
   }

   for (unsigned char i = DEFAULT_ACCESS_POINT_GAIN_SIZE - 1; i != 0xFF; i--) {
      if (*access_point_starting_position == ',') {
         for (unsigned char i2 = i; i2 != 0xFF; i2--) {
            default_access_point_gain_g[i2] = ' ';
         }
         break;
      }

      default_access_point_gain_g[i] = *access_point_starting_position;
      access_point_starting_position--;
   }
}

unsigned int get_current_piped_task_to_send() {
   return piped_tasks_to_send_g[0];
}

void delete_current_piped_task() {
   for (unsigned char i = 0; piped_tasks_to_send_g[i] != 0; i++) {
      unsigned int next_task = piped_tasks_to_send_g[i + 1];
      piped_tasks_to_send_g[i] = next_task;
   }
}

void add_piped_task_to_send_into_tail(unsigned int task) {
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
      if (piped_tasks_to_send_g[i] == 0) {
         piped_tasks_to_send_g[i] = task;
         break;
      }
   }
}

void add_piped_task_to_send_into_head(unsigned int task) {
   for (unsigned char i = PIPED_TASKS_TO_SEND_SIZE - 1; i != 0; i--) {
      piped_tasks_to_send_g[i] = piped_tasks_to_send_g[i - 1];
   }
   piped_tasks_to_send_g[0] = task;
}

void delete_piped_task(unsigned int task) {
   unsigned char task_is_found = 0;
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
      if (piped_tasks_to_send_g[i] == task || task_is_found) {
         piped_tasks_to_send_g[i] = piped_tasks_to_send_g[i + 1];
         task_is_found = 1;
      }

      if (piped_tasks_to_send_g[i] == 0) {
         break;
      }
   }
}

void delete_all_piped_tasks() {
   for (unsigned char i = 0; i < PIPED_TASKS_TO_SEND_SIZE; i++) {
     piped_tasks_to_send_g[i] = 0;
   }
}

void add_piped_task_into_history(unsigned int task) {
   if (task == 0) {
      return;
   }

   unsigned char i;
   unsigned int last_task = piped_tasks_history_g[PIPED_TASKS_HISTORY_SIZE - 1];
   for (i = 0; i < PIPED_TASKS_HISTORY_SIZE - 1; i++) {
      if (piped_tasks_history_g[i] == 0) {
         break;
      }

      if (last_task) {
         // The last task already exists
         unsigned int next_task = piped_tasks_history_g[i + 1];
         piped_tasks_history_g[i] = next_task;
      }
   }
   piped_tasks_history_g[i] = task;
}

unsigned int get_last_piped_task_in_history() {
   for (unsigned char i = PIPED_TASKS_HISTORY_SIZE - 1; i != 0xFF; i--) {
      if (piped_tasks_history_g[i] != 0) {
         return piped_tasks_history_g[i];
      }
   }
   return piped_tasks_history_g[0];
}

unsigned char is_usart_response_contains_element(char string_to_be_contained[]) {
   if (contains_string(usart_data_received_buffer_g, string_to_be_contained)) {
      return 1;
   } else {
      return 0;
   }
}

//char *data_to_be_contained[] = {ESP8226_REQUEST_DISABLE_ECHO, USART_OK};
unsigned char is_usart_response_contains_elements(char *data_to_be_contained[], unsigned char elements_count) {
   for (unsigned char elements_index = 0; elements_index < elements_count; elements_index++) {
      if (!contains_string(usart_data_received_buffer_g, data_to_be_contained[elements_index])) {
         return 0;
      }
   }
   return 1;
}

unsigned char contains_string(char being_compared_string[], char string_to_be_contained[]) {
   unsigned char found = 0;

   if (*being_compared_string == '\0' || *string_to_be_contained == '\0') {
      return found;
   }

   for (; *being_compared_string != '\0'; being_compared_string++) {
      unsigned char all_chars_are_equal = 1;

      for (char *char_address = string_to_be_contained; *char_address != '\0';
            char_address++, being_compared_string++) {
         if (*being_compared_string == '\0') {
            return found;
         }

         all_chars_are_equal = *being_compared_string == *char_address ? 1 : 0;

         if (!all_chars_are_equal) {
            break;
         }
      }

      if (all_chars_are_equal) {
         found = 1;
         break;
      }
   }
   return found;
}

void disable_echo() {
   send_usard_data(ESP8226_REQUEST_DISABLE_ECHO);
   set_flag(&sent_flag_g, DISABLE_ECHO_FLAG);
}

void get_network_list() {
   send_usard_data(ESP8226_REQUEST_GET_VISIBLE_NETWORK_LIST);
   set_flag(&sent_flag_g, GET_VISIBLE_NETWORK_LIST_FLAG);
}

void get_connection_status() {
   send_usard_data(ESP8226_REQUEST_GET_CONNECTION_STATUS);
   set_flag(&sent_flag_g, GET_CONNECTION_STATUS_AND_CONNECT_FLAG);
}

void connect_to_network() {
   char *parameters[] = {DEFAULT_ACCESS_POINT_NAME, DEFAULT_ACCESS_POINT_PASSWORD, NULL};
   usart_data_to_be_transmitted_buffer_g = set_string_parameters(ESP8226_REQUEST_CONNECT_TO_NETWORK_AND_SAVE, parameters);
   send_usard_data(usart_data_to_be_transmitted_buffer_g);
   set_flag(&sent_flag_g, CONNECT_TO_NETWORK_FLAG);
}

void execute_usart_data_sending(void (*function_to_execute)(), unsigned char timeout, unsigned char execute_function) {
   send_usart_data_timout_sec_g = timeout;
   send_usart_data_function_g = function_to_execute;
   if (execute_function == EXECUTE_FUNCTION) {
      function_to_execute();
   }
}

void Clock_Config() {
   RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
   RCC_PLLCmd(DISABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET);
   RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_4); // 8MHz / 2 * 4
   RCC_PCLKConfig(RCC_HCLK_Div1);
   RCC_PLLCmd(ENABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void Pins_Config() {
   // Connect BOOT0 to ground

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

   GPIO_InitTypeDef gpioInitType;
   gpioInitType.GPIO_Pin = 0x89E1; // PA13, PA14 - Debugger pins
   gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_2; // 10 MHz
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &gpioInitType);

   // Motion sensor input
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Pin = MOTION_SENSOR_INPUT_PIN;
   GPIO_Init(MOTION_SENSOR_INPUT_PORT, &gpioInitType);

   // For USART1
   gpioInitType.GPIO_Pin = (1<<GPIO_PinSource9) | (1<<GPIO_PinSource10);
   gpioInitType.GPIO_PuPd = GPIO_PuPd_NOPULL;
   gpioInitType.GPIO_Mode = GPIO_Mode_AF;
   gpioInitType.GPIO_OType = GPIO_OType_PP;
   GPIO_Init(GPIOA, &gpioInitType);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

   gpioInitType.GPIO_Pin = GPIO_Pin_All;
   gpioInitType.GPIO_Mode = GPIO_Mode_IN;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOB, &gpioInitType);

   // PA1 LED
   gpioInitType.GPIO_Pin = NETWORK_STATUS_LED_PIN;
   gpioInitType.GPIO_Mode = GPIO_Mode_OUT;
   gpioInitType.GPIO_Speed = GPIO_Speed_Level_1;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_DOWN;
   gpioInitType.GPIO_OType = GPIO_OType_PP;
   GPIO_Init(NETWORK_STATUS_LED_PORT, &gpioInitType);

   // PA2 LED
   gpioInitType.GPIO_Pin = SERVER_AVAILABILITI_LED_PIN;
   GPIO_Init(SERVER_AVAILABILITI_LED_PORT, &gpioInitType);

   // MOTION SENSOR LED
   gpioInitType.GPIO_Pin = MOTION_SENSOR_LED_PIN;
   GPIO_Init(MOTION_SENSOR_LED_PORT, &gpioInitType);

   // ESP8266 enable/disable
   gpioInitType.GPIO_Pin = ESP8266_CONTROL_PIN;
   gpioInitType.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &gpioInitType);
}

/**
 * USART frame time Tfr = (1 / USART_BAUD_RATE) * 10bits
 * Timer time to be sure the frame is ended Tt = Tfr + 0.5 * Tfr
 * Frequency = 16Mhz, USART_BAUD_RATE = 115200. Tt = 0.13ms
 */
void TIMER3_Confing() {
   DBGMCU_APB1PeriphConfig(DBGMCU_TIM3_STOP, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = CLOCK_SPEED * 15 / USART_BAUD_RATE; // CLOCK_SPEED * 15 / USART_BAUD_RATE;
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

   NVIC_EnableIRQ(TIM3_IRQn);
   TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM3, ENABLE);
}

/**
 * 0.1s with 16MHz clock
 */
void TIMER6_Confing() {
   DBGMCU_APB1PeriphConfig(DBGMCU_TIM6_STOP, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = 24;
   TIM_TimeBaseStructure.TIM_Prescaler = 0xFFFF;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

   TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
   NVIC_EnableIRQ(TIM6_IRQn);
   TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM6, ENABLE);
}

void DMA_Config() {
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

   DMA_InitTypeDef dmaInitType;
   dmaInitType.DMA_PeripheralBaseAddr = USART1_TDR_ADDRESS;
   //dmaInitType.DMA_MemoryBaseAddr = (uint32_t)(&usartDataToBeTransmitted);
   dmaInitType.DMA_DIR = DMA_DIR_PeripheralDST; // Specifies if the peripheral is the source or destination
   dmaInitType.DMA_BufferSize = 0;
   dmaInitType.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   dmaInitType.DMA_MemoryInc = DMA_MemoryInc_Enable; // DMA_MemoryInc_Enable if DMA_InitTypeDef.DMA_BufferSize > 1
   dmaInitType.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   dmaInitType.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
   dmaInitType.DMA_Mode = DMA_Mode_Normal;
   dmaInitType.DMA_Priority = DMA_Priority_High;
   dmaInitType.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(USART1_TX_DMA_CHANNEL, &dmaInitType);

   DMA_ITConfig(USART1_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
   NVIC_EnableIRQ(USART1_IRQn);

   DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
}

void USART_Config() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   USART_OverSampling8Cmd(USART1, DISABLE);

   USART_InitTypeDef USART_InitStructure;
   USART_InitStructure.USART_BaudRate = USART_BAUD_RATE;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART1, &USART_InitStructure);

   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

   USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

   USART_Cmd(USART1, ENABLE);
}

void EXTERNAL_Interrupt_Config() {
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, MOTION_SENSOR_EXTI_PIN_SOURCE);

   EXTI_InitTypeDef EXTI_InitStructure;
   EXTI_InitStructure.EXTI_Line = EXTI_Line3;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   NVIC_InitTypeDef NVIC_InitTypeInitStructure;
   NVIC_InitTypeInitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
   NVIC_InitTypeInitStructure.NVIC_IRQChannelPriority = 3;
   NVIC_InitTypeInitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitTypeInitStructure);

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

void set_flag(unsigned int *flags, unsigned int flag_value) {
   *flags |= flag_value;
}

void reset_flag(unsigned int *flags, unsigned int flag_value) {
   *flags &= ~(*flags & flag_value);
}

unsigned char read_flag_state(unsigned int *flags, unsigned int flag_value) {
   return (*flags & flag_value) > 0 ? 1 : 0;
}

void send_usard_data(char *string) {
   clear_usart_data_received_buffer();
   DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);
   unsigned short bytes_to_send = get_string_length(string);

   if (bytes_to_send == 0) {
      return;
   }

   DMA_SetCurrDataCounter(USART1_TX_DMA_CHANNEL, bytes_to_send);
   USART1_TX_DMA_CHANNEL->CMAR = (unsigned int) string;
   USART_ClearFlag(USART1, USART_FLAG_TC);
   DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
}

/**
 * Supports only 9 parameters (1 - 9). Do not forget to call free() function on returned pointer when it's no longer needed
 *
 * *parameters - array of pointers to strings. The last parameter has to be NULL
 */
void *set_string_parameters(char string[], char *parameters[]) {
   unsigned char open_brace_found = 0;
   unsigned char parameters_amount = 0;
   unsigned short result_string_length = 0;

   for (; parameters[parameters_amount] != NULL; parameters_amount++) {
   }

   // Calculate the length without symbols to be replaced ('<x>')
   for (char *string_pointer = string; *string_pointer != '\0'; string_pointer++) {
      if (*string_pointer == '<') {
         if (open_brace_found) {
            return NULL;
         }
         open_brace_found = 1;
         continue;
      }
      if (*string_pointer == '>') {
         if (!open_brace_found) {
            return NULL;
         }
         open_brace_found = 0;
         continue;
      }
      if (open_brace_found) {
         continue;
      }

      result_string_length++;
   }

   if (open_brace_found) {
      return NULL;
   }

   for (unsigned char i = 0; parameters[i] != NULL; i++) {
      result_string_length += get_string_length(parameters[i]);
   }
   // 1 is for the last \0 character
   result_string_length++;

   char *allocated_result = malloc(result_string_length); // (string_length + 1) * sizeof(char)

   if (allocated_result == NULL) {
      return NULL;
   }

   unsigned char result_string_index = 0, input_string_index = 0;
   for (; result_string_index < result_string_length - 1; result_string_index++) {
      char input_string_symbol = string[input_string_index];

      if (input_string_symbol == '<') {
         input_string_index++;
         input_string_symbol = string[input_string_index] ;

         if (input_string_symbol < '1' || input_string_symbol > '9') {
            return NULL;
         }

         input_string_symbol -= 48; // Now it's not allocated_result char character, but allocated_result number
         if (input_string_symbol > parameters_amount) {
            return NULL;
         }
         input_string_index += 2;

         // Parameters are starting with 1
         char *parameter = parameters[input_string_symbol - 1];

         for (; *parameter != '\0'; parameter++, result_string_index++) {
            *(allocated_result + result_string_index) = *parameter;
         }
         result_string_index--;
      } else {
         *(allocated_result + result_string_index) = string[input_string_index];
         input_string_index++;
      }
   }
   *(allocated_result + result_string_length - 1) = '\0';
   return allocated_result;
}

unsigned short get_string_length(char string[]) {
   unsigned short length = 0;

   for (char *string_pointer = string; *string_pointer != '\0'; string_pointer++, length++) {
   }
   return length;
}

/**
 * Do not forget to call free() function on returned pointer when it's no longer needed
 */
void *short_to_string(unsigned short number) {
   if (number == 0) {
      return "0";
   }

   unsigned short remaining = number;
   unsigned short divider = 10000;
   unsigned char string_length = 0;
   char *result_string_pointer = NULL;

   for (unsigned char string_index = 5; string_index > 0; string_index--, divider /= 10) {
      char result_character = (char) (remaining / divider);

      if (result_string_pointer == NULL && result_character) {
         result_string_pointer = malloc(string_index + 1);
         string_length = string_index;
      }
      if (result_string_pointer != NULL) {
         unsigned char index = string_length - string_index;
         *(result_string_pointer + index) = result_character + 48;
      }

      remaining -= result_character * divider;
   }
   result_string_pointer[string_length] = '\0';
   return result_string_pointer;
}

/**
 * Do not forget to call free() function on returned pointer when it's no longer needed
 */
void *array_to_string(char array[], unsigned char array_length) {
   char *result = malloc(array_length + 1);

   for (unsigned char i = 0; i < array_length; i++) {
      result[i] = array[i];
   }
   result[array_length] = '\0';
   return result;
}

void clear_usart_data_received_buffer() {
   for (int i = 0; i < USART_DATA_RECEIVED_BUFFER_SIZE; i++) {
      if (usart_data_received_buffer_g[i] == '\0') {
         break;
      }

      usart_data_received_buffer_g[i] = '\0';
   }
}

void enable_esp8266() {
   GPIO_WriteBit(GPIOA, ESP8266_CONTROL_PIN, Bit_RESET);
   esp8266_disabled_timer_g = TIMER6_5S;
}

void disable_esp8266() {
   GPIO_WriteBit(GPIOA, ESP8266_CONTROL_PIN, Bit_SET);
   GPIO_WriteBit(NETWORK_STATUS_LED_PORT, NETWORK_STATUS_LED_PIN, Bit_RESET);
}

unsigned char is_esp8266_enabled(unsigned char include_timer) {
   return include_timer ? (!GPIO_ReadOutputDataBit(GPIOA, ESP8266_CONTROL_PIN) && esp8266_disabled_timer_g == 0) :
         !GPIO_ReadOutputDataBit(GPIOA, ESP8266_CONTROL_PIN);
}
