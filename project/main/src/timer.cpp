#include"timer.hpp"
// 信号处理函数
void timer_handler(int signum) 
{
    if (signum == SIGALRM) 
    {
        // 在这里进行周期性操作，例如打印信息或调用其他函数
       
        //Final_Motor_Control(70,0.3,20);
        //dynamic_pid_value_set();
        //printf("Periodic interrupt occurred\n");


        // 重新设置 setitimer 定时器，实现周期性
        struct itimerval timer;
        timer.it_value.tv_sec = 0;
        timer.it_value.tv_usec = 50000;  // 0.05 秒后再次触发
        timer.it_interval.tv_sec = 0;
        timer.it_interval.tv_usec = 50000;  // 周期性触发间隔 0.05 秒
        if (setitimer(ITIMER_REAL, &timer, NULL) == -1) 
        {
            perror("setitimer");
            exit(EXIT_FAILURE);
        }
    }
}

void time_init(void)
{
    // 注册信号处理函数
    signal(SIGALRM, timer_handler);


    // 设置初始 setitimer 定时器
    struct itimerval timer;
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 50000;  // 初始延迟 0.05 秒
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 50000;  // 周期性触发间隔 0.05 秒
    if (setitimer(ITIMER_REAL, &timer, NULL) == -1) 
    {
        perror("setitimer");
        exit(EXIT_FAILURE);
    }
}