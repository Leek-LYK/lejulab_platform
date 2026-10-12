# LEJU_CLAW_DRIVER



## Getting started
## 乐聚自研二指夹爪  
* 夹爪初始化时会有寻零操作（正负方向各运动一次来确定运行量程），此时夹爪不能存在遮挡，否则会影响行程
* 夹爪左右id分别为 ``0X0F,0X10``
* 请将`config.yaml`复制或替换原来`/home/lab/.config/lejuconfig/config.yaml`下的配置文件
* 
## 如何测试？  
```bash
git clone https://www.lejuhub.com/zhanglongbo/leju_claw_driver.git  
cd leju_claw_driver 
sudo rm -rf ~/.config/lejuconfig/config.yaml
cp ./config.yaml ~/.config/lejuconfig
mkdir build && cd build 
cmake .. && make   
sudo ./lejuclaw_test 

```
测试过程，观察夹爪是否正常运动
### 控制函数接口  

`` LeJuClaw::PawMoveState LeJuClaw::move_paw(const std::vector<double> &positions, const std::vector<double> &velocity, const std::vector<double> &torque);``  

```bash
/**
 * @brief 控制夹爪按照指定位置、速度和力矩进行运动，并返回夹爪最后的夹取结果。
 *
 * @param positions 目标位置，单位为百分比（0 ~ 100）。表示夹爪的开合程度（100为完全闭合）。
 * @param velocity 目标速度，每个夹爪的夹取速度。
 * @param torque 目标力矩，电机不会输出大于该值的电流（如果给的过小，可能运动效果受限，推荐 1A~2A）。
 *
 * @return LeJuClaw::PawMoveState
 *         - LEFT_REACHED_RIGHT_REACHED: 左、右夹爪均到达目标位置，没有夹取物品。
 *         - LEFT_REACHED_RIGHT_GRABBED: 左夹爪到达目标位置，右夹爪夹取到物品。
 *         - LEFT_GRABBED_RIGHT_REACHED: 左夹爪夹取到物品，右夹爪到达目标位置。
 *         - LEFT_GRABBED_RIGHT_GRABBED: 左、右夹爪均夹取到物品。
 *         - ERROR: 发生错误，如线程未运行。
 *
 * @note 
 * - 确保函数调用前夹爪系统已正确初始化且线程已运行。
 * - 运动过程中可能受速度或力矩限制影响，目标可能无法完全达到。
 */
```

``std::vector<double> LeJuClaw::get_positions();``

```bash
/**
 * @brief 获取当前各夹爪位置。
 *
 * @return std::vector<double>
 *         当前个夹爪的位置，以百分比（0 ~ 100）表示的列表。每个值对应一个夹爪的开合程度：
 *         - 0: 表示夹爪完全打开。
 *         - 100: 表示夹爪完全关闭。
 *
 * @note 
 * - 。
 */
```

### 测试用例  
参考`lejuclaw_test.cpp`  

### 遇到问题/想要新功能，请在该仓库提交QA