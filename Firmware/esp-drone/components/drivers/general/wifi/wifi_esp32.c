#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "crtp_commander.h"
#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"
//below added by dks
#include "ledseq.h"
#include "system.h"
#include <string.h>
#include "pm_esplane.h" //dks

#include "ms5611.h"
#include "position_controller.h"
#include "position_estimator.h"
#include "zranger2.h"



#define UDP_SERVER_PORT         2390
#define UDP_SERVER_BUFSIZE      128
float distanceDown = 0.0f; //dks
int motor_rpm = 0;
static struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6

//#define WIFI_SSID      "Udp Server"
static char WIFI_SSID[32] = "ARIS_01";
static char WIFI_PWD[64] = "12345678" ;
static uint8_t WIFI_CH = 1;
#define MAX_STA_CONN (3)

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

bool takeoff_battery = false;
int counter = 0;
bool armMode = false;
bool altHoldMode = true;
bool disarm_clicked = false;
bool isarmMode = false;
bool takeoff_completed = false;
bool land_completed = false;
bool less_voltage = false;
bool f = false;
//bool landModde = false;
//bool takeOffMode = false;
bool landMode = false;
bool landCompleatedOnce = false;
bool isTakeOff = false;
bool land = true;
bool landed = false;
bool isArmsuccess = false;
static char rx_buffer[UDP_SERVER_BUFSIZE];
static char tx_buffer[UDP_SERVER_BUFSIZE];
const int addr_family = (int)AF_INET;
const int ip_protocol = IPPROTO_IP;
static struct sockaddr_in dest_addr;
static int sock;

static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;
static UDPPacket inPacket;
static UDPPacket outPacket;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

static bool isAltHoldEnabled = false; // Track the state of AltHold dks
static bool isArmed = false;
// bool altHoldMode = false;

static bool isOnground = false; // status of the drone dks
static bool motorinit = false;
static bool ismotorinit = false;
static esp_err_t udp_server_create(void *arg);
int start_time_us = 0;
static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }

    return cksum;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "join, AID=%d", MAC2STR(event->mac), event->aid);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "leave, AID=%d", MAC2STR(event->mac), event->aid);
    } 
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(1);
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    static UDPPacket outStage;
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static esp_err_t udp_server_create(void *arg)
{ 
    if (isUDPInit){
        return ESP_OK;
    }
    
    struct sockaddr_in *pdest_addr = &dest_addr;
    pdest_addr->sin_addr.s_addr = htonl(INADDR_ANY);
    pdest_addr->sin_family = AF_INET;
    pdest_addr->sin_port = htons(UDP_SERVER_PORT);

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    uint8_t cksum = 0;
    socklen_t socklen = sizeof(source_addr);
    
    while (true) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0) {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            break;
        } else if(len > WIFI_RX_TX_PACKET_SIZE - 4) {
            DEBUG_PRINT_LOCAL("Received data length = %d > 64", len);
        } else {
            //copy part of the UDP packet
            rx_buffer[len] = 0;// Null-terminate whatever we received and treat like a string...
             // Log the incoming packet
            
           // printf("Received packet of size: %d bytes\n", len);
            //printf("Received packet data: ");
            for (int i = 0; i < len; i++) {
                if (rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13){
                printf(" %02X", rx_buffer[i]);
                }
            }
           // printf("\n");
            
            if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x33 && rx_buffer[3] == 0x01){ // ARM message
                isArmed = false;  
                printf("arm button pressed on\n");
                if (!isArmed)
                {
                    isArmsuccess = true;
                    armMode = true;
                    disarm_clicked = false;
                    isArmed = true;
                    uint8_t packet[4] = {0xCD, 0xCC, 0xAC, 0x41};  // Hardcoded float 21.4 (little-endian)
                    for (int i = 0; i < 10; i++) {
                        wifiSendData(sizeof(packet), packet);
                    }
                    printf("ARM Packet bytes: ");
                    for (size_t i = 0; i < sizeof(packet); i++) {
                        printf("%02X ", packet[i]);
                    }
                }
                 else {
                     armMode = false;
                     printf("arm mode is false \n");
                     isArmed = false;
                 }

            }
            
            else if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x33 && rx_buffer[3] == 0x00) // disarm message
            {   
                printf("Disarm button pressed \n");
                if (isArmed)
                {
                    isArmsuccess = false;
                    armMode = false;
                    disarm_clicked = true;
                    printf("Disarm mode is activated \n");
                    uint8_t packet[4] = {0xCD, 0xCC, 0xAC, 0x42};  // Hardcoded float 21.4 (little-endian)
                    for (int i = 0; i < 10; i++) {
                        wifiSendData(sizeof(packet), packet);
                    }
                    printf("Packet bytes: ");
                    for (size_t i = 0; i < sizeof(packet); i++) {
                        printf("%02X ", packet[i]);
                    }
                    isArmed = false;
                }
            }
            if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x11 && rx_buffer[3] == 0x01){ // takeoff message
                counter = 4;
                landed=false;
                takeoff_battery = true;
                printf("takeoff  mode is pressed on\n");
                //isTakeOff = false;
                if (!less_voltage && !isTakeOff && altHoldMode && distanceDown>0 && isArmsuccess)
                {
                    //altHoldMode = true;
                    targetAltitude = 0.50f;// 0.50f; //distanceDown;
                    printf("Takeoff mode is actvated with TOF  target altitude is %f \n", targetAltitude);
                    isTakeOff = true;
                    //land_completed = false;
                    //landMode = false;
                }
                 else {
                     //altHoldMode = false;
                     printf("island mode is false \n");
                    isTakeOff = false;
                 }

            }
            else if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x11 && rx_buffer[3] == 0x00) //LAND message
            {
                takeoff_battery = false;
                printf("land mode is pressed \n");
                if (isTakeOff && altHoldMode && isArmsuccess){
                    if(takeoff_completed){
                        landMode = true;
                        //altHoldMode = false;
                        targetAltitude = 0.05f;
                        isTakeOff = false;
                        printf("Land mode is actvated with TOF  target altitude is %f \n", targetAltitude);

                    }else{
                        printf("Drone is on ground!!! %f \n", distanceDown);
                    }
                     
                }
            }
            if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x22 && rx_buffer[3] == 0x00) // for enabling sports mode
            {
                printf("sports mode is pressed on\n");
                altHoldMode = false;   
            }

            else if(rx_buffer[0] == 0x71 && rx_buffer[1] == 0x13 && rx_buffer[2] == 0x22 && rx_buffer[3] == 0x01) // for disabling sports mode and enabling alt hold mode
            {
                printf("sports mode is pressed off\n");
                altHoldMode = true; 
            }
            memcpy(inPacket.data, rx_buffer, len);
            cksum = inPacket.data[len - 1];
            //remove cksum, do not belong to CRTP
            inPacket.size = len - 1;
            //check packet
            if (cksum == calculate_cksum(inPacket.data, len - 1) && inPacket.size < 64){
                xQueueSend(udpDataRx, &inPacket, M2T(2));
                if(!isUDPConnected) isUDPConnected = true;
            }else{
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }

#ifdef DEBUG_UDP
            DEBUG_PRINT_LOCAL("1.Received data size = %d  %02X \n cksum = %02X", len, inPacket.data[0], cksum);
            for (size_t i = 0; i < len; i++) {
                DEBUG_PRINT_LOCAL(" data[%d] = %02X ", i, inPacket.data[i]);
            }
#endif
        }
         if(distanceDown > 0.10f && altHoldMode ){ // 
            takeoff_completed = true;
            takeoff_battery = true;
            f = true;
            // printf("takeoff_completed %f \n",distanceDown);
            if(land){
            uint8_t packet[4] = {0xCD, 0xCC, 0xAC, 0x43};  // Hardcoded float 21.4 (little-endian)
           
            for (int i = 0; i < 10; i++) {
                wifiSendData(sizeof(packet), packet);
                printf("kaushik it is in takeoff mode\n");
            }
            //landed=false;
            land = false;
        }
            
            landCompleatedOnce = false;
            landMode = true;
            }
        if(takeoff_completed && landMode && distanceDown <= 0.065f){
            //printf("distance down is in landing\n");
            landCompleatedOnce = false;
            takeoff_battery = false;
            if(!landCompleatedOnce){
                land_completed = true;
                landCompleatedOnce = true;
                // printf("land_completed %f \n",distanceDown);     
                uint8_t packet[4] = {0xCD, 0xCC, 0xAC, 0x44};  // Hardcoded float 21.4 (little-endian)
               // printf("Land Packet bytes: ");
                for (size_t i = 0; i < sizeof(packet); i++) {
                    wifiSendData(sizeof(packet), packet);
                    printf("kaushik it is in landoff mode\n");

                }
                landed = true;
                land = true;
                
                }else{
                land_completed = false;
            
                }
       // printf("Tof data %f \n",distanceDown);
        //printf("Tof data %f \n",tofMeasurement->distance);
    }

}
}

static void udp_server_tx_task(void *pvParameters)
{
    while (true) {
        // Wait for UDP to be initialized
        if (!isUDPInit) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            continue;
        }

        // Wait for data to transmit
        if ((xQueueReceive(udpDataTx, &outPacket, 5 / portTICK_PERIOD_MS) == pdTRUE) && isUDPConnected) {
            
            // Copy data into tx buffer
            memcpy(tx_buffer, outPacket.data, outPacket.size);

            // Calculate and add checksum at the end
            uint8_t checksum = calculate_cksum(tx_buffer, outPacket.size);
            tx_buffer[outPacket.size] = checksum;
            tx_buffer[outPacket.size + 1] = 0;  // Null-terminator (optional if not needed)

            ESP_LOGI("UDP_TX", "Sending %d bytes + 1 byte checksum", outPacket.size);

            // Print data before sending
            // printf("TX Packet: ");
            // for (size_t i = 0; i < outPacket.size + 1; i++) {
            //     printf("%02X ", tx_buffer[i]);
            // }
            // printf("\n");

            ESP_LOGI("HEAP", "Before send: %d", esp_get_free_heap_size());

            // Send the data
            int err = sendto(sock, tx_buffer, outPacket.size + 1, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            ESP_LOGI("HEAP", "After send: %d", esp_get_free_heap_size());

            if (err < 0) {
                ESP_LOGE("UDP_TX", "Error occurred during sending: errno %d", errno);
                continue;
            }

            ESP_LOGI("UDP_TX", "UDP packet sent successfully (%d bytes)", err);
        }
    }
}



void wifiInit(void)
{
    if (isInit) {
        return;
    }

    esp_netif_t *ap_netif = NULL;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ap_netif = esp_netif_create_default_wifi_ap();
    uint8_t mac[6];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "ARIS_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_CH,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1) ;
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1) ;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr      = ipaddr_addr("192.168.43.42"),
    };
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);

    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);
    if (udp_server_create(NULL) == ESP_FAIL) {
        DEBUG_PRINT_LOCAL("UDP server create socket failed!!!");
    } else {
        DEBUG_PRINT_LOCAL("UDP server create socket succeed!!!");
    } 
    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    xTaskCreate(sendBatteryVoltageTask,"BatteryVoltageTask",2048,NULL,UDP_TX_TASK_PRI + 1,NULL);

    isInit = true;
}

static void sendBatteryVoltageTask(void)
{
    float voltage;
    const uint8_t header = 0xBE; // Define a header byte (e.g., 0xBE for "Battery")
    uint8_t packet[1 + sizeof(float)]; // 1 byte for header + 4 bytes for float
 
    while (1)
    {  
        voltage = pmGetBatteryVoltage(); // Retrieve battery voltage
       // printf("Battery voltage: %f\n", voltage); // Print battery voltage to console
 if(isthrust || takeoff_battery){
     voltage += 0.37;
 }
         printf("Battery voltage  before: %f\n", voltage);

 if(f==true){
 voltage = 3.6;
 }
 if(voltage<=3.7){
    uint8_t packets[4] = {0xCD, 0xCC, 0xAC, 0x45};  // Hardcoded float 21.4 (little-endian)
    wifiSendData(sizeof(packets), packets);            
 }

        printf("Battery voltage: %f\n", voltage);
            packet[0] = header; // Add header at the beginning
            memcpy(&packet[1], &voltage, sizeof(float)); // Copy voltage after the header
 
            wifiSendData(sizeof(packet), packet); // Send the packet over Wi-Fi
        
            
             if (voltage <= 3.6f ) {
                //  isTakeOff = true;
                //  altHoldMode = true;
                //  isArmed = true;
                //  landMode = false;
                //  less_voltage = true;
                //  printf("Battery voltage low: %.2f V. Initiating auto-landing...\n", voltage);

                //  takeoff_battery = false;
                //  landMode = true;
                 targetAltitude = 0.05f;
                //  isTakeOff = false;
                //  land = true;

                 // Optional: alert over UDP
                //  uint8_t lowBatteryPacket[4] = {0xCD, 0xCC, 0xAC, 0x45}; // or another signal
                //  for (int i = 0; i < 10; i++)
                //  {
                //      wifiSendData(sizeof(lowBatteryPacket), lowBatteryPacket);
                //      vTaskDelay(pdMS_TO_TICKS(100));
                //  }
        }
        else{
            less_voltage = false;
        }
        // Debug print of packet contents
        // printf("Packet bytes: ");
        // for (size_t i = 0; i < sizeof(packet); i++)
        // {
        //     printf("%02X ", packet[i]);
        // }
        // printf("\n");
 
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}