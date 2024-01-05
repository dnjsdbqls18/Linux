#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int sum = 0;

void *sum0_to_100(void *arg)
{
    pthread_mutex_lock(&mutex);
    for (int i = 0; i <= 100; i++)
    {
        sum += i;
    }
    printf("0~100의 합 : %d\n", sum);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

void *sum101_to_200(void *arg)
{
    pthread_mutex_lock(&mutex);
    for (int i = 101; i <= 200; i++)
    {
        sum += i;
    }
    printf("101~200의 합 : %d\n", sum);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

void *sum201_to_300(void *arg)
{
    pthread_mutex_lock(&mutex);
    for (int i = 201; i <= 300; i++)
    {
        sum += i;
    }
    printf("201~300의 합 : %d\n", sum);
    pthread_mutex_unlock(&mutex);
    pthread_exit(NULL);
}

int main()
{
    pthread_t tid1, tid2, tid3;

    if (pthread_create(&tid1, NULL, sum0_to_100, NULL) != 0)
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, sum101_to_200, NULL) != 0)
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, sum201_to_300, NULL) != 0)
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

    pthread_mutex_lock(&mutex);
    printf("0~300 : %d\n", sum);
    pthread_mutex_unlock(&mutex);

    pthread_mutex_destroy(&mutex);

    exit(0);
}
