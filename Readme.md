# Smart Car 2025

## 写在前面

浙江省智能车竞赛，C1 组 - 极速光电龙芯赛道完赛代码

第一次作为 A 类赛的主程，作为一个小白学习到了很多。

有点遗憾的是最终因为种种原因只跑完了 2.3 m/s 的低速稳定版，拿到了省三等。

视觉部分完成了大津，基于补线的全元素，基于色块的圆环状态识别，基于陀螺仪的坡道识别。

控制部分完成了模糊 PID 控制，阿克曼差速，速度环开环下的速度决策与速度控制，无负压，可以参考。

dev 分支里面有速度环闭环的方案，[LanternCX/SmartCar2025-Vision](https://github.com/LanternCX/SmartCar2025-Vision) 里面有基于逆透视的视觉方案，但是因为种种原因最后没有采用。

## 项目结构

```bash
.
├── libraries 				# 存放外部库 主要是逐飞库
└── project 				# 存放项目代码和构建输出
    ├── main 				# 存放项目主要代码
    │   ├── build.sh 		# 交叉编译构建脚本
    │   ├── CMakeLists.txt 	# CMakeLists.txt
    │   ├── cross.cmake 	# 交叉编译 cross.cmake
    │   ├── control 		# 控制相关代码
    │	├── vision			# 视觉相关代码
    │   ├── utils 			# 工具
    │   │   ├── control		# 控制相关工具
    │   │   ├── display		# 屏幕显示相关工具
    │   │   ├── debug		# 调试相关工具
    │   │   ├── time		# 时间相关工具
    │   │   └── math		# 数学相关工具
    │   ├── Main.cpp 		# 主函数文件
    │   └── Main.h 			# 主头文件
    └── out 				# 构建输出，需要自己创建
```

## 手动联网

```bash
wpa_supplicant -B -i wlan0 -c <(wpa_passphrase "Unknow" "buzhidao")
sleep 3
wpa_supplicant -B -i wlan0 -c <(wpa_passphrase "Unknow" "buzhidao")
sleep 3
udhcpc -i wlan0
```

然后如果是通过网线连接的话`ssh`会话会断开，直接在无线连接就好了。

## 注意

使用代码请遵守 [GPL v3.0 协议](https://www.runoob.com/w3cnote/open-source-license.html)
