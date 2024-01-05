#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

void *sum0_to_100(void* arg)
{
    int sum1 = 0;
    for (int i = 0; i <= 100; i++)
    {
        sum1 += i;
    }
    printf("0~100의 합 : %d\n", sum1);
    pthread_exit((void *)sum1); 
}

void *sum101_to_200(void *arg)
{
    int sum2 = 0;
    for (int i = 101; i <= 200; i++)
    {
        sum2 += i;
    }
    printf("101~200의 합 : %d\n", sum2);
    pthread_exit((void *)sum2); 
}

void *sum201_to_300(void *arg)
{
    int sum3 = 0;
    for (int i = 201; i <= 300; i++)
    {
        sum3 += i;
    }
    printf("201~300의 합 : %d\n", sum3);
    pthread_exit((void *)sum3); 
}

int main()
{
    pthread_t tid1, tid2, tid3;
    int sum1_result, sum2_result, sum3_result;

    if (pthread_create(&tid1, NULL, sum0_to_100, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, sum101_to_200, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, sum201_to_300, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    pthread_join(tid1, (void **)&sum1_result); 
    pthread_join(tid2, (void **)&sum2_result); 
    pthread_join(tid3, (void **)&sum3_result); 

    int sum0_to_300 = sum1_result + sum2_result + sum3_result;
    printf("0~300의 합 : %d\n", sum0_to_300);

    return 0;
}
