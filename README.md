# ArmorDetector
# 装甲板检测算法详细说明

## 算法概述

这是一个依托OpenCV库实现的装甲板检测系统，专门设计用于识别视频中的红色装甲板，并标记其四个角点，通过多步骤处理流程从视频帧中提取、识别和定位装甲板。

## 算法流程详细说明

### 1. 初始化和视频读取
```cpp
VideoCapture video;
video.open("/home/cw/opencv_learning/ArmorDetector/R2.mp4");
```
- 创建VideoCapture对象并打开指定路径的视频文件
- 检查视频是否成功打开，失败则输出错误信息

### 2. 逐帧处理循环
```cpp
for (;;) {
    video >> frame;
    if (frame.empty()) {
        break;
    }
    // 处理每一帧
}
```
- 循环读取视频的每一帧
- 当帧为空时退出循环，视频结束

### 3. 颜色空间转换和红色区域提取
```cpp
cvtColor(frame, hsv, COLOR_BGR2HSV);
```
**选择HSV颜色空间，而非RGB**
**发现使用RGB识别红色装甲板时，误识别了视频中明亮的白色灯光，所以改为HSV识别，HSV将颜色信息与亮度信息分离，明度通道独立于颜色信息，使颜色识别更加稳定，最后成功识别。**

```cpp
Scalar lower_red1(0, 100, 100);
Scalar upper_red1(10, 255, 255);
Scalar lower_red2(160, 100, 100);
Scalar upper_red2(179, 255, 255);
```
- 定义红色的两个HSV范围，因为红色在HSV色环上跨越0°和180°两个区域
- 第一个范围(0-10)捕捉偏橙色的红色
- 第二个范围(160-179)捕捉偏紫色的红色

```cpp
inRange(hsv, lower_red1, upper_red1, red_mask1);
inRange(hsv, lower_red2, upper_red2, red_mask2);
red_mask = red_mask1 | red_mask2;
```
- 使用`inRange`函数创建两个红色区域的二值掩码
- 通过逻辑或操作合并两个掩码，得到完整的红色区域

### 4. 图像预处理
```cpp
GaussianBlur(red_mask, Gaussian, Size(5, 5), 0);
dilate(Gaussian, dilatee, element);
```
- 高斯模糊：减少噪声和细小干扰
- 膨胀操作：连接相邻的红色区域，填补可能的中断

### 5. 轮廓检测和筛选
```cpp
findContours(dilatee, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
```
- 使用`findContours`检测二值图像中的所有轮廓
- `RETR_TREE`检索模式保留轮廓的层次结构
- `CHAIN_APPROX_NONE`存储所有轮廓点而不进行近似

```cpp
for (int i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < 5 || contours[i].size() <= 1)
        continue;
        
    RotatedRect Light_Rec = fitEllipse(contours[i]);
    if (Light_Rec.size.width / Light_Rec.size.height > 4)
        continue;
        
    lightInfos.push_back(LightDescriptor(Light_Rec));
}
```
- 筛选轮廓：去除面积过小或点数过少的轮廓（可能是噪声）
- 使用椭圆拟合轮廓，得到旋转矩形
- 筛选长宽比过大的轮廓（灯条通常是细长的）

### 6. 灯条匹配
```cpp
for (size_t i = 0; i < lightInfos.size(); i++) {
    for (size_t j = i + 1; (j < lightInfos.size()); j++) {
        // 计算各种几何特征
        // 应用匹配条件
    }
}
```
- 双重循环遍历所有灯条对
- 计算以下几何特征用于匹配：
  - 角度差(`angleGap_`)
  - 长度比例(`LenGap_ratio`, `lengap_ratio`)
  - 中心点距离(`dis`, `ratio`)
  - 垂直和水平位置差(`yGap_ratio`, `xGap_ratio`)

```cpp
// 匹配条件
if (angleGap_ > 15 ||
    LenGap_ratio > 1.0 ||
    lengap_ratio > 0.8 ||
    yGap_ratio > 1.5 ||
    xGap_ratio > 2.2 ||
    xGap_ratio < 0.8 ||
    ratio > 3 ||
    ratio < 0.8) {
    continue;
}
```
- 应用严格的几何约束条件筛选符合条件的灯条对
- 这些条件确保了匹配的灯条对具有相似的几何特性，符合装甲板上灯条的预期排列

### 7. 装甲板定位和可视化
```cpp
// 计算装甲板角点
Point center = Point((leftLight.center.x + rightLight.center.x) / 2, 
                    (leftLight.center.y + rightLight.center.y) / 2);
RotatedRect rect = RotatedRect(center, Size(dis, meanLen), 
                              (leftLight.angle + rightLight.angle) / 2);
```
- 基于匹配的灯条对计算装甲板的中心点、尺寸和角度
- 创建表示装甲板的旋转矩形

```cpp
// 对点进行排序
vector<Point2f> points(vertices, vertices + 4);
sortPoints(points);
```
- 获取旋转矩形的四个角点
- 使用自定义的`sortPoints`函数对点进行排序（左上、右上、右下、左下）

```cpp
// 绘制装甲板边界和角点
for (int k = 0; k < 4; k++) {
    line(frame, points[k], points[(k + 1) % 4], Scalar(0, 0, 255), 2);
    circle(frame, points[k], 6, Scalar(0, 255, 0), -1);
    string point_num = to_string(k + 1);
    putText(frame, point_num, points[k] + Point2f(-5, 5), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
}
```
- 绘制装甲板边界（红色线条）
- 标记角点（绿色圆点）
- 标注角点序号（蓝色数字）

### 8. 显示和结束处理
```cpp
namedWindow("Armor Detection", WINDOW_FREERATIO);
imshow("Armor Detection", frame);
```
- 创建可自由调整比例的窗口
- 显示处理后的帧

```cpp
// 视频播放完后，等待按键
waitKey(0);
```
- 视频结束后，用户按任意键关闭视频
