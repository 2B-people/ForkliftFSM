## 测试调用接口示例

> 这里把接口分为两块
> 调用接口指的是其他app调用FSM的API
> 动作接口指的是FSM调用机器人动作的API

测试流程示例：

* 将该包放入catkin工作空间
* `catkin_make`编译
* `source devel/setup.bash`加载环境


* 运行模拟action，fsm主程序，smach_viewer可视化的测试脚本

```bash
    roslaunch ForkliftFSM test.launch
```
>这里如果在远程终端启动,会无法启动smach_viewer节点,报错,这是正常的,因为smach_viewer是图形界面,需要在本地启动,如果在本地启动,则不会报错

* 运行调用API示例脚本

```bash
    rosrun ForkliftFSM TestAPICall.py
```


