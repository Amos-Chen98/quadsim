# quadsim
 北京航空航天大学《四旋翼飞行器建模仿真综合实验》

## 实验内容

1.  四旋翼飞行器建模：在Simulink中建立四旋翼飞行模型，能正确对飞行器在力和力矩作用下的线运动、角运动进行 6DOF仿真；
2.  定点悬停控制实验：基于四旋翼飞行器模型，实现对指定3D空间点的悬停控制；建立GUI界面，能为四旋翼飞行器输入参数并观察仿真飞行轨迹；分析控制误差；在可能的情况下改进控制算法，提高控制精度；
3. 航路跟踪控制实验：基于四旋翼飞行器模型，实现对指定任意指定的 3D空间航路进行跟踪；实现GUI界面，能交互式输入一组航路点，并绘制 3D的飞行轨迹分析控制误差；在可能的情况下改进控制算法，提高控制精度
4. 编队跟踪控制实验：基于四旋翼飞行器模型，建立三机线性与圆形编队，沿指定路径编队飞行；要求实现GUI界面，绘制编队飞行轨迹，分析控制误差；在可能的情况下改进控制算法，提高控制精度。

## 如何运行

有两种运行仿真的方式：

1. APP运行模式：运行QuadSim.mlapp可打开GUI界面，随后可以在GUI界面中输入仿真参数并获得输出。GUI界面比较精美，但受限于界面画幅，输出的信息有限。

2. 脚本运行模式：有四个平行的.m脚本文件，分别对应四个实验，功能是将参数加载进工作空间，运行Simulink仿真，并输出详尽的仿真结果。

![image-20220424233225479](https://raw.githubusercontent.com/Amos-Chen98/Image_bed/main/2022/20220424233225.png)

3Dplot_fcns文件夹中是绘制3D动画（如下）的函数，由指导教师提供。该文件夹在APP运行时（在实验一与实验二中）被调用。由于该函数运行缓慢，因此实验三与实验四的动态效果图由本人编写的动态绘图代码实现。

![image-20220424233237792](https://raw.githubusercontent.com/Amos-Chen98/Image_bed/main/2022/20220424233237.png)

由于控制器参数数目较多，在实验报告中不列写全部参数，所有参数的具体取值可在脚本文件中查看。若无特殊说明，所有位置坐标均在北-东-地坐标系下。

## 原理与建模过程

请见实验报告。
