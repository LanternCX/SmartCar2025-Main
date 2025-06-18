#include"thread.hpp"


void set_thread_priority(pthread_t thread_id, int policy, int priority) //设置线程优先级
{
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(thread_id, policy, &param)) 
    {
        perror("pthread_setschedparam");
    }
}