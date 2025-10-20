# g2o 曲线拟合可视化工具说明

本项目包含两个可执行文件：
1. `curve_fitting` - 执行g2o优化的主程序
2. `visualization` - 专门用于可视化的Qt程序

## 文件说明

### 主程序 (curve_fitting)
执行曲线拟合优化，并生成两个输出文件：
- `data.txt` - 原始数据点
- `result.txt` - 优化后的参数结果

### 可视化程序 (visualization)
独立的Qt可视化程序，可以从文件读取数据并显示图表。

## 编译方法

```bash
mkdir build
cd build
cmake ..
make
```

## 使用方法

### 方法一：运行主程序后使用可视化程序
```bash
# 运行优化程序
./bin/curve_fitting

# 使用可视化程序显示结果
./bin/visualization ./data/data.txt 0.987 1.923 1.054  # 参数值根据实际结果替换
```

### 方法二：使用默认示例数据
```bash
./bin/visualization
```

### 方法三：从result.txt文件中读取参数
```bash
# 从result.txt中提取参数
a=$(grep "a:" ../result.txt | cut -d ' ' -f 2)
b=$(grep "b:" ../result.txt | cut -d ' ' -f 2)
c=$(grep "c:" ../result.txt | cut -d ' ' -f 2)

# 使用提取的参数运行可视化程序
./bin/visualization ./data/data.txt $a $b $c
```

## 程序参数说明

### visualization程序命令行参数：
```bash
./bin/visualization <data_file> <a> <b> <c>
```

- `data_file`: 包含数据点的文件路径（每行格式：x y）
- `a, b, c`: 指数函数 y = exp(a*x² + b*x + c) 的参数

### 示例：
```bash
./bin/visualization ./data/data.txt 1.0 2.0 1.0
```

## 数据文件格式

### 输入数据文件格式
每行包含一个数据点的x和y坐标，用空格分隔：
```
0.01 7.389
0.02 7.421
0.03 7.452
...
```

### 结果文件格式
包含优化后的参数：
```
a: 1.00023
b: 1.99874
c: 1.00156
```

## 可视化功能说明

可视化程序将显示：
1. 原始数据点（散点图）
2. 拟合曲线（线图）
3. 图表标题包含拟合函数的具体参数
4. 坐标轴标签和图例

## Qt图表组件说明

- 使用QScatterSeries显示原始数据点
- 使用QLineSeries显示拟合曲线
- 使用QValueAxis作为坐标轴
- 支持鼠标交互（缩放、平移等）