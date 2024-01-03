#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

// 쓰레드 동작시 실행될 함수
void *firstThreadRun(void* arg)
{
        for(int i = 1; i<=9;i++)
        {
          printf("1 * %d = %d\n",i,1*i);
          sleep(1);
        }
       printf("\n");
       pthread_exit(NULL);
}
 
void *secondThreadRun(void* arg)
{
        sleep(10);
    
        for(int i = 1; i<=9;i++)
        {
          printf("2 * %d = %d\n",i,2*i);
          sleep(1);
        }
        printf("\n");
        pthread_exit(NULL);
}

void *thirdThreadRun(void* arg)
{
        sleep(20);
    
        for(int i = 1; i<=9;i++)
        {
          printf("3 * %d = %d\n",i,3*i);
          sleep(1);
        }
        
        pthread_exit(NULL);
}
 
 
 
int main()
{
    pthread_t firstThread, seconThread,thirdThread;
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
    
    if(threadErr = pthread_create(&thirdThread,NULL,thirdThreadRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    //while(1);
    //sleep(10);
    
    pthread_join(firstThread, NULL);
    pthread_join(seconThread, NULL);
    pthread_join(thirdThread, NULL);
}
