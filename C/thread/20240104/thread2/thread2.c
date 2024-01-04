#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int value;

void *func1(void *arg) //1번 스레드가 실행할 함수
{
    for(int i = 0; i < 5; i++)
    {
        pthread_mutex_lock(&mutex); //mutex 변수 lock(디른 변수에서 접근 불가능)
        printf("thread1 : %d\n", value); //변수 출력
        value++; //변수 증가
        pthread_mutex_unlock(&mutex); //mutex 변수 unlock(다른 변수에서도 접근 가능)
        sleep(1);
    }
    
    pthread_exit(NULL);
}

void *func2(void *arg) //2번 스레드 실행할 함수
{
    for(int i = 0; i < 5; i++)
    {
        pthread_mutex_lock(&mutex); //mutex 변수 lock(다른 변수에서 접근 불가능)
        printf("thread2 : %d\n", value); //변수 출력
        value++; //변수 증가
        pthread_mutex_unlock(&mutex); //mutex 변수 unlock(다른 변수에서 접근 가능)
        sleep(1);
    }
    
    pthread_exit(NULL);
}

int main()
{
    pthread_t tid1, tid2; //스레드 변수 선언
    
    value = 0; //공유 변수 초기화
    
    if(pthread_create(&tid1, NULL, func1, NULL) != 0) //1번 스레드 생성
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }
     
    if(pthread_create(&tid2, NULL, func2, NULL) != 0) //2번 스레드 ㅅㅇ성
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }
    
    if(pthread_join(tid1, NULL) != 0) //1번 스레드 종료 후 리소스 회수
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }
    
    if(pthread_join(tid2, NULL) != 0) //2번 스레드 종료 후 리소스 회수
    {
        fprintf(stderr, "prthread join error\n");
        exit(1);
    }
    
    pthread_mutex_destroy(&mutex); //mutex 변수 해제
    
    exit(0);
}
