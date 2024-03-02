# bsp_dwt

**为什么要使用dwt？**
1. 首先使用dwt可以节省一个定时器资源。在某些MCU中或者是某种应用场景中，定时器资源是十分稀缺的，如果使用定时器定时来计算代码执行时间，代价是非常昂贵的。，
2. HAL库的delay函数使用的是滴答定时器，并且直接对滴答定时器的寄存器进行了赋值的操作，由于FreeRTOS的系统调度也是使用的滴答定时器，直接使用HAL_Delay会倒是任务调度无法进行。

[STM32隐藏定时器dwt(zhihu.com)](https://zhuanlan.zhihu.com/p/405212820)
[FreeRTOS信号量](https://www.jianshu.com/p/ca0ff9c9adae)