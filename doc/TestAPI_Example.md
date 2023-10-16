## 测试调用接口示例

> 这里把接口分为两块
> 调用接口指的是其他app调用FSM的API
> 动作接口指的是FSM调用机器人动作的API

测试流程示例：

* 将该包放入catkin工作空间
* `catkin_make`编译
* `source devel/setup.bash`加载环境
* 运行roscore

```bash
 roscore
```

* 运行模拟FSM，service&&param_server接口的测试脚本

```bash
 rosrun ForkliftFSM TestAPIServer.py
```
* 运行调用API示例脚本

```bash
 rosrun ForkliftFSM examples\APIClient_v1.py
```

>!注：这里的API不是最后FSM的API，只是为了测试通信链路是否完整
>完整API请参考FSM的API文档（TODO）
