# 2026/10/12
使用同济sp战队开源的电控中间件的底盘代码

## 如何编译（协作者必读）

### 1. 装好三样东西，并全部加入 `PATH`

| 工具 | 版本 | 下载 |
|---|---|---|
| CMake | ≥ 3.22 | https://cmake.org/download/ |
| Ninja | 任意 | https://github.com/ninja-build/ninja/releases |
| arm-none-eabi-gcc | 12 及以上 | https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads |

验证：三条命令都能打印版本号即可。

```bat
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

### 2. 一条命令编译（**不需要手动配置 CMake**）

```bat
build.bat            :: 编译 Debug
build.bat Release    :: 编译 Release
build.bat Debug clean :: 清空重编
```

产物：`build\Debug\Qust_version.elf`

**原理**：仓库里已经带了 `CMakePresets.json`（预设了 Ninja 生成器、`build/<预设>` 输出目录、`cmake/gcc-arm-none-eabi.cmake` 交叉工具链），所以 `cmake --preset Debug` 就是完整配置，不需要任何 `-D` 参数。

### 3. 在 VS Code 里（推荐）

打开工程 → 自动提示安装推荐插件（CMake Tools / C/C++ / Cortex-Debug）→

- **Ctrl+Shift+B**：编译 + 烧录（任务已配好）
- **F5**：启动调试（OpenOCD + Live Watch）

> `build/` 是每台机器本地生成的，**已被 `.gitignore` 排除**。如果编译报错，先 `build.bat Debug clean`。

## 云台c板代码

云台控制思路：
由于电机反馈的rpm并不能精确反应角速度，需要使用imu加速度计（+delta t）进行角速度值的反馈

速控模式：将遥控器摇杆通道/鼠标通道*k（灵敏度）
    操作手模式时比较跟手
位控模式：外环位置环，内环速度环。调参麻烦。
    自瞄模式时启用位置控制

小陀螺：角速度越大越难直线行驶，当需要小陀螺模式行进时减小转速以及增加前馈；小陀螺模式时位控可以只依赖云台陀螺仪进行自稳。速度环时需要底盘角速度值反馈调节

底盘跟随：当上坡时需要正对坡道不然会有轮子悬空，因此需要底盘跟随，而RMUL 步兵对抗赛地图没有坡，所以底盘跟随是可选项


