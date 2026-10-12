## 文件说明

- SDK 文件夹存放供应商提供的 Linux SDK 和 SDK 插件的压缩文件
- ENI_export ENI 文件制作所需要的软件和xml文件以及制作流程说明文档

# 主站运行环境初始化

主站运行时需要root权限

只做一次

```
mkdir ~/.config/lejuconfig
cp config/offset.csv ~/.config/lejuconfig/
```

每次开机都需要初始化，这里提供一个自动初始化脚本

```shell
sudo su
chmod +x setupEnv.sh
./setupEnv.sh
```

自动初始化不成功,可以尝试手动初始化

```
cd LinkOsLayer/Linux/atemsys
make
sudo insmod atemsys.ko
make clean
```

```
echo "<Instance-ID>" > /sys/bus/pci/drivers/<driver-name>/unbind
# echo "0000:58:00.0" > /sys/bus/pci/drivers/igc/unbind
```

## 动态连接库 分支 编译与调用说明

需要先生成 主站进程和控制接口 .so文件
```shell
cd EC_Master
mkdir build
cd build
cmake ..
make
```

其他文件调用 .so文件时可以使用

```cmake
target_link_libraries(
${PROJECT_SOURCE_DIR}/EC_Master/build/src/libec_app.so
${PROJECT_SOURCE_DIR}/EC_Master/build/src/libec_master_main.so
)
```
编译可以参考 https://www.lejuhub.com/highlydynamic/ec_master_joint_actuator

### API 接口

电机控制以及 EC_Master 其他内容可以参考

EC_Master/src/Examples/EcMasterDemo/EcDemoApp.h 下的 FUNCTION DECLARATIONS 部分

## Master 分支 测试demo 编译与运行

```shell
mkdir build
cd build
cmake ..
make -j`nproc`
```

## ENI选项与命令严格对齐
:warning: -auxclk 和 dcmmode 在导出 ENI 时进行设置

EC-EngineerWeb 中
Append Slave 直接添加从站
点击左边栏主站: 
- Master页面， Cycle Time : 是 auxclk 的值 
- Distributed Clocks:Clock Adjustment 是 dcmmode 模型

```shell
./src/ec_master -i8254x 1 1 -f ../config/elmo_1_eni.xml -auxclk 500 -dcmmode mastershift -t 0 # -perf -v 3 -sp 6000
```

常用参数说明：
- -i8253x 1 1 i8254x 表示使用的通讯网卡为 Intel 千兆网卡，第一个 1 为 instance，从 1 开始 1,2,3,..
 第二个 0 为网卡工作模式，0 为中断模式，1 为轮询模式，同步时钟必须使用 1
- -v 2 log 信息的级别，数字越大，信息越详细；0 为关闭，0,1,2,3,...
- -perf 主站性能统计信息，统计各个任务执行时间
- -f eni.xml 为使用的网络描述文件
- -auxclk 设置的主站循环周期，单位为微秒（us）;如果操作系统不支持高精度时钟，则功能
和-b 相同
- -dcmmode 适用于 EcMasterDemoDc 这个 Demo，模式可选为 busshift(默认)，mastershift 和 off，
off 表示不使用同步时钟，适用于网络中不存在支持 DC 的从站时

