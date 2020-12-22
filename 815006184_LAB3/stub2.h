#include <stdio.h>
#include <string.h>
#include <assert.h>



//prototype
void integrationOne(char *string);
void integrationTwo(int i); 

//stub function for name
void integrationOne(char *string) {
    //the string cannot be longer than 8 characters
    assert(strlen(string)>8);

    //the string cannot be NULL
    assert(string != NULL);

    //the string cannot be empty 
    assert(*string!='\0');

    //show test case passed the unit test
    printf("The unit test was passed\n");
}


//stub function for ID number
void integrationTwo(int i) {
    //i cannot be more than ten 
    assert(i<10);

    //i cannot be less than zero
    assert(i>0);

    //show test case passed the unit test
    printf("The unit test was passed\n");
}
 