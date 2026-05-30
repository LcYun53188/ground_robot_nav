# nav_safety

安全监视与急停发布包。

- 订阅: `/oakd/points`
- 发布: `/nav/emergency`

当前实现检查点云、TF 和里程计健康状态，并面向地面机器人发布应急状态。
