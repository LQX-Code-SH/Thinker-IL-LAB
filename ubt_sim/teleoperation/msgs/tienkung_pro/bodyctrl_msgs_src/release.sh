#!/bin/bash

# 获取当前包名和版本号
PKG_NAME=$(grep -oP '(?<=<name>).*?(?=</name>)' src/bodyctrl_msgs/package.xml)
VERSION=$(grep -oP '(?<=<version>).*?(?=</version>)' src/bodyctrl_msgs/package.xml)

# 如果版本号是 0.0.0，建议设置为 0.1.0 或保持原样
if [ "$VERSION" == "0.0.0" ]; then
    VERSION="0.1.0"
fi

# 按照 ROS 规范命名 deb 包 (ros-humble-xxx)
DEB_NAME="ros-humble-${PKG_NAME//_/-}"

echo "Preparing to build $DEB_NAME version $VERSION..."

# 清现有的构建目录
rm -rf build/ install/ log/

# 使用 merge-install 使得目录结构扁平化，方便打包
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查构建是否成功
if [ $? -ne 0 ]; then
    echo "Error: Colcon build failed."
    exit 1
fi

echo "Building debian package using fpm..."

# 使用 fpm 打包
# --prefix /opt/ros/humble 将文件安装到 ROS 标准路径
# -d 指定运行依赖
fpm -s dir -t deb \
    -n "$DEB_NAME" \
    -v "$VERSION" \
    --iteration 1 \
    --description "Body control messages for ROS 2 Humble" \
    --prefix /opt/ros/humble \
    -d "ros-humble-rclcpp" \
    -d "ros-humble-std-msgs" \
    -d "ros-humble-sensor-msgs" \
    -d "ros-humble-geometry-msgs" \
    -d "ros-humble-builtin-interfaces" \
    -d "ros-humble-rosidl-default-runtime" \
    -d "libyaml-cpp-dev" \
    --force \
    -C install \
    include/ \
    lib/ \
    share/ \
    local/ 

if [ $? -eq 0 ]; then
    echo "--------------------------------------------------"
    echo "成功生成 deb 包!"
    echo "安装命令: sudo dpkg -i ${DEB_NAME}_${VERSION}-1_*.deb"
    echo "之后其他人只需要执行 source /opt/ros/humble/setup.bash 即可直接使用该消息。"
    echo "--------------------------------------------------"
else
    echo "Error: fpm packaging failed."
    exit 1
fi
