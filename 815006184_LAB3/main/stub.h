#include <stdio.h>
#include <assert.h>
#include <string.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"


//static const char *TAG = "main";

void unitTestOne();
void unitTestTwo();
void unitTestThree();
void unitTestFour();

void unitTestOne(char *string){

	printf("%s\n",string);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",chip_info.cores);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	// set the test conditions, string must more than 8 characters...
	assert(strlen(string) > 8);
			
}

void unitTestTwo(char *string){
    printf("%s\n",string);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",chip_info.cores);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    // string cannot be NULL
	assert(string != NULL);
   
   
}

void unitTestThree(char *string){
printf("%s\n",string);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",chip_info.cores);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
     //string cannot be empty, test3 should fail here and program abort...
    assert(*string != '\0');
}

void unitTestFour(char *string){
    printf("%s\n",string);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",chip_info.cores);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	 //string must output Hello World...
    int result = strcmp(string, "Hello world!");
    assert(result == 0);
    
}

//void unitTestTwo(int n){
//	
//	for (int i = n; i >= 0; i--) {
//        ESP_LOGI(TAG, "Restarting in %d seconds...\n", i);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//    ESP_LOGI(TAG, "Restarting now.\n");
//	
//}
//assert(strlen(string) > 8);
    // string cannot be NULL
//	assert(string != NULL);
    // string cannot be empty, test3 should fail here and program abort...
  //  assert(*string != '\0');