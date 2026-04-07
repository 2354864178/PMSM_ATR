# ATR_C++ 文档导航

本项目文档已按用途拆分为三份，请按场景阅读。

## 快速开始
1. 构建

```bash
make
```

2. 运行

```bash
make run
```

3. 清理

```bash
make clean
```

说明：当前 make run 会自动先 clean 再全量编译运行。

## 文档入口
1. 架构说明：[docs/架构说明.md](docs/架构说明.md)
2. 使用说明：[docs/使用说明.md](docs/使用说明.md)
3. 原理说明：[docs/原理说明.md](docs/原理说明.md)

## 关键文件
1. 主程序入口：[src/main.cpp](src/main.cpp)
2. 运行配置：[include/config/sim_config.h](include/config/sim_config.h)
3. 求解器接口：[include/solver/system_solver.h](include/solver/system_solver.h)
4. 控制器实现：[model/controllers/CurrentController.cpp](model/controllers/CurrentController.cpp)

## 输出文件
1. 仿真日志：[build/log.xlsx](build/log.xlsx)
2. 绘图笔记本：[notebooks/plot.ipynb](notebooks/plot.ipynb)
