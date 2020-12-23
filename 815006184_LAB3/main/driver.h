#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "stub.h"

void printDriverOne();
void printDriverTwo();
void printDriverThree();
void printDriverFour();

void printDriverOne(){
// first test array of char, 10 characters, should be OK for the 3 test conditions...
char test1[] = "abcdefghij";
printf("Testing the string #1 \"%s\"\n", test1);
unitTestOne(test1);
}

void printDriverTwo(){
// second test pointer to string, 9 characters, should be OK for the 3 test conditions...
char *test2 = "123456789";
printf("Testing the string #2 \"%s\"\n", test2);
unitTestTwo(test2);
}

void printDriverThree(){
// third test array char, empty, should fail on the 3rd condition, cannot be empty...
char test3[] = "";
printf("Testing the string #3, Expected to Fail \"%s\"\n", test3);
unitTestThree(test3);
}

void printDriverFour(){
// third test array char, empty, should fail on the 3rd condition, cannot be empty...
char test4[] = "World Hello!";
printf("Testing the string #4, Expected to Fail \"%s\"\n", test4);
unitTestFour(test4);
}

//printf("Testing the string #1 \"%s\"\n", test1);
//unitTestOne(test1);
//printf("Testing the string #2 \"%s\"\n", test2);
//unitTestOne(test2);
//printf("Testing the string #3, Expected to Fail \"%s\"\n", test3);
//unitTestOne(test3);