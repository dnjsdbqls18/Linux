#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

// 쓰레드 동작시 실행될 함수
void *firstThreadRun(void* arg)
{
    while(1)
    {
        sleep(1);
        printf("start First Thread\n");
    }
}
 
void *secondThreadRun(void* arg)
{
    while(1)
    {
        sleep(3);
        printf("start Second Thread\n");
    }
}
 
 
 
int main()
{
    pthread_t firstThread, seconThread;
    int threadErr;
    
    
    // 쓰레드를 만들고 쓰레드 함수 실행
    if(threadErr = pthread_create(&firstThread,NULL,firstThreadRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    if(threadErr = pthread_create(&seconThread,NULL,secondThreadRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    //while(1);
    //sleep(10);
    
    pthread_join(firstThread, NULL);
    pthread_join(seconThread, NULL);
}
