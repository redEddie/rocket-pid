#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <math.h>

// 0과 1 사이의 white noise 생성 함수
double whiteNoise()
{
    return (double)rand() / RAND_MAX;
}

int main()
{
    int a = 1;
    int b = -2;
    printf("%lf, %lf \n", asin(a), asin(abs(b)));

    // srand((unsigned int)time(NULL)); // 시드 설정

    // int num_samples = 10; // 생성할 샘플 수
    // printf("Generated White Noise Samples:\n");

    // for (int i = 0; i < num_samples; ++i)
    // {
    //     double noise = whiteNoise();
    //     printf("%f\n", noise);
    // }

    system("pause");

    return 0;
}
