#include <stdio.h>
#include <string.h>
#include "stubf3.h" 

void integrationdriver1();
void integrationdriver2();
void integrationdriver3();
void integrationdriver4();
void integrationdriver5();

void integrationdriver1(){
    //test case 1
    char test1[] = "";
    printf("The output of test is: \"%s\"\n", test1);
    tryTest(test1);
    //test case 2
    char test2[] = "My name is ";
    printf("The output of test2 is: \"%s\"\n", test2);
    tryTest(test2);
}

void integrationdriver2(){
    //test case 3
    char test3[] = "Hello World";
    printf("The output of test3 is: \"%s\"\n", test3);
    tryTest(test3);
    //test case 4
    char test4[] = "";
    printf("The output of test4 is: \"%s\"\n", test4);
    tryTest(test4);
}

void integrationdriver3(){
    //test case 1 for integration testing 
    char test1[] = "";
    tryTest(test1);
    printf("The output of test is: \"%s\"\n", test1);
    
    int itest1 = 8;
    tryTest2(itest1);
    printf("The value of i: %i\n", itest1);
    for (int i = itest1 ; i >= 0; i--) {

    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

void integrationdriver4(){
    //test case 2 for integration testing 
    char test1[] = "";
    tryTest(test1);
    printf("The output of test is: \"%s\"\n", test1);
    
    int itest2 = 15;
    tryTest2(itest2);
    printf("The value of i: %i\n", itest2);
    for (int i = itest2 ; i >= 0; i--) {

    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

void integrationdriver5(){
    //test case 3 for integration testing 
    char test1[] = "";
    tryTest(test1);
    printf("The output of test is: \"%s\"\n", test1);
    
    int itest3 = -6;
    printf("%i\n", itest3);
    tryTest2(itest3);
    printf("The value of i: %i\n", itest3);
    for (int i = itest3 ; i >= 0; i--) {

    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

