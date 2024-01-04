#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int value;

int sumRange(int start, int end)
{
    int sum = 0;
    for (int i = start; i <= end; i++)
    {
        sum += i;
    }
    return sum;
}

void *sum0To100(void *arg)
{
    pthread_mutex_lock(&mutex);
    int result = sumRange(0, 100);
    printf("0~100의 합 : %d\n", result);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

void *sum101To200(void *arg)
{
    pthread_mutex_lock(&mutex);
    int result = sumRange(101, 200);
    printf("101~200의 합 : %d\n", result);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

void *sum201To300(void *arg)
{
    pthread_mutex_lock(&mutex);
    int result = sumRange(201, 300);
    printf("201~300의 합 : %d\n", result);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

void *sum0To300(void *arg)
{
    pthread_mutex_lock(&mutex);
    int result = sumRange(0, 300);
    printf("0~300의 합: %d\n", result);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

int main()
{
    pthread_t tid1, tid2, tid3, tid4; 

    value = 0; 

    if (pthread_create(&tid1, NULL, sum0To100, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, sum101To200, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, sum201To300, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid4, NULL, sum0To300, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_join(tid1, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid2, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid3, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid4, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    pthread_mutex_destroy(&mutex); 

    exit(0);
}
