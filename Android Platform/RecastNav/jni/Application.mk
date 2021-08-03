#ndk编译命令 ndk-build all -B
# 编译所有CPU的.so
# APP_ABI := all  

# 添加几乎全平台支持 ARM-x86
APP_ABI := armeabi-v7a x86 arm64-v8a x86_64  

# 最低android版本为 android-18 -> android-4.3
APP_PLATFORM := android-18

# 添加C++ STL库支持
APP_STL := c++_shared