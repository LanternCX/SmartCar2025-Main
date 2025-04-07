# Smart Car 2025

## 项目结构

```bash
.
├── libraries 				# 存放外部库 主要是逐飞库
└── project 				# 存放项目代码和构建输出
    ├── main 				# 存放项目主要代码
    │   ├── build.sh 		# 交叉编译构建脚本
    │   ├── CMakeLists.txt 	# CMakeLists.txt
    │   ├── cross.cmake 	# 交叉编译 cross.cmake
    │   ├── control 		# 方向以及速度控制相关代码
    │	├── vision			# 视觉相关代码
    │   ├── utils 			# 工具
    │   │   ├── control		# 控制相关工具
    │   │   ├── display		# 屏幕显示相关工具
    │   │   └── math		# 数学相关工具
    │   ├── Main.cpp 		# 主函数文件
    │   └── Main.h 			# 主头文件
    └── out 				# 构建输出
```

## 视觉部分

### Done

1. 大津法二值化

2. 八邻域扫线

### Todo

1. 逆透视
2. 扫线优化
3. 形态学滤波
4. 十字判断
5. 圆环判断
6. 障碍判断
7. 坡道判断

## 控制部分

### Done

1. 速度环
2. 方向环
3. 后轮差速

### Todo

1. 优化差速策略
2. 陀螺仪
3. 速度策略

## 其他部分

### Todo

1. 调参菜单
2. 逐飞助手接入
3. 适配自研主板引角
4. 自研上位机