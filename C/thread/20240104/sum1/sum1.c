#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

int var = 0;

void *sum1(void* arg) // 1번 스레드 실행 함수
{
    int sum = 0;
    for (int i = 0; i <= 100; i++)
    {
        sum += i;
    }
    printf("0~100의 합 : %d\n", sum);
    pthread_exit((void *)sum); // 1번 스레드 종료, 합을 반환
}

void *sum2(void *arg) // 2번 스레드 실행 함수
{
    int sum = 0;
    for (int i = 101; i <= 200; i++)
    {
        sum += i;
    }
    printf("101~200의 합 : %d\n", sum);
    pthread_exit((void *)sum); // 2번 스레드 종료, 합을 반환
}

void *sum3(void *arg) // 3번 스레드 실행 함수
{
    int sum = 0;
    for (int i = 201; i <= 300; i++)
    {
        sum += i;
    }
    printf("201~300의 합 : %d\n", sum);
    pthread_exit((void *)sum); // 3번 스레드 종료, 합을 반환
}

int calculateTotalSum(int sum1_result, int sum2_result, int sum3_result)
{
    return sum1_result + sum2_result + sum3_result;
}

int main()
{
    pthread_t tid1, tid2, tid3;
    int sum1_result, sum2_result, sum3_result;

    if (pthread_create(&tid1, NULL, sum1, (void *)&sum1_result) != 0) // 1번 thread 생성
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, sum2, (void *)&sum2_result) != 0) // 2번 thread 생성
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, sum3, (void *)&sum3_result) != 0) // 3번 thread 생성
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    pthread_join(tid1, (void **)&sum1_result); // 1번 스레드 자원 회수 및 결과 저장
    pthread_join(tid2, (void **)&sum2_result); // 2번 스레드 자원 회수 및 결과 저장
    pthread_join(tid3, (void **)&sum3_result); // 3번 스레드 자원 회수 및 결과 저장

    int totalSum = calculateTotalSum(sum1_result, sum2_result, sum3_result);
    printf("총합 : %d\n", totalSum);

    return 0;
}
