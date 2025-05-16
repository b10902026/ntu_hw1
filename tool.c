#include<stdio.h>
#include<stdbool.h>
#include<assert.h>
#include<string.h>
#include<stdlib.h>
/*
 Order of state expansion:\\$S \rightarrow B \rightarrow E \rightarrow G$
 \\final path :\\$S \rightarrow B \rightarrow E \rightarrow G$
*/
int main(){
    char c[100];
    printf("Order of state expansion:\n");
    scanf("%s",c);
    char d[100];
    printf("final path :\n");
    scanf("%s",d);
    printf("Order of state expansion:\\\\$");
    printf("");
    for(int i=0;i<strlen(c);i++){
        printf("%c",c[i]);
        if(i!=strlen(c)-1)printf(" \\rightarrow ");
    }
    printf("$\\\\final path :\\\\$");
    for(int i=0;i<strlen(d);i++){
        printf("%c",d[i]);
        if(i!=strlen(d)-1)printf(" \\rightarrow ");
    }
    printf("$");
    return 0;
 }