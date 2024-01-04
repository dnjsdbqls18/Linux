#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

int var = 0;

void *func1(void* arg) //1번 스레드 실행 함수
{
    while(var < 10)
    {
        printf("thread1 : %d\n", ++var);
        sleep(1);
    }
    pthread_exit(NULL); //1번 스레드 종료
}

void *func2(void *arg) //2번 스레드 실행 함수
{
    while(var < 10)
    {
        printf("thread2 : %d\n", ++var);
        sleep(1);
    }
    pthread_exit(NULL); //2번 스레드 종료
}

int main()
{
    pthread_t tid1,tid2;
    
    if(pthread_create(&tid1, NULL, func1, NULL) != 0) //1번 thread 생성
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }
    
    if(pthread_create(&tid2, NULL, func2, NULL) != 0) //2번 thread 생성
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }
    
    pthread_join(tid1, NULL); //1번 스레드 자원 회수
    pthread_join(tid2, NULL); //2번 스레드 자원 회수 
    
    return 0;
}
