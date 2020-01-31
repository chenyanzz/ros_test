# ros_test

## 这个项目是什么

为dji rm2020冬令营提前写的一部分ros代码 (造轮子)

## 都有什么

代码在```src/```下面

- ```dashboard``` [TEST HALF PASSED]

    一个Qt做的兼容```bool, string, int```类型的可读写仪表盘

    可以做库也可以做ros节点都行

- ```gyro``` [TEST PASSED]

    一个为维特智能WT61C串口陀螺仪制作的陀螺仪数据桥接ros话题

- ```motor``` [NO TESTED]

    一个与RM电调进行交互的节点

    可以做库也可以做ros节点都行

- ```uart``` [TEST PASSED]
    通过UART进行数据打包发送的节点

- ```test```

    一些节点测试程序

- ```point_n, point_m, rotator```

    RM2020面试题里的位姿转换

- ```turtle_ctrl```
    Rm2020网上任务的小乌龟