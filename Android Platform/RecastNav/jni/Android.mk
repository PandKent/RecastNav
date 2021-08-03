# 设置本地目录
LOCAL_PATH := $(call my-dir)

# ----------------------------设置链接库导出配置--------------------------------------#

include $(CLEAR_VARS)

LOCAL_MODULE := RecastNav

LOCAL_MODULE_FILENAME := libRecastNav

# 添加源文件 使用wildcard函数获取Local目录下指定文件夹所有cpp文件
FILE_LIST := $(wildcard $(LOCAL_PATH)/Detour/Source/*.cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/Recast/Source/*.cpp)
FILE_LIST += $(wildcard $(LOCAL_PATH)/RecastNavDll/*.cpp)
# FILE_LIST += $(wildcard $(LOCAL_PATH)/Tests/*.hpp)

LOCAL_SRC_FILES := $(FILE_LIST:$(LOCAL_PATH)/%=%) 

# --------------------------- 添加头文件支持 ---------------------------------------#
HEADER_FILE_LIST := 

LOCAL_C_INCLUDES := $(LOCAL_PATH)/Detour/Include \
					$(LOCAL_PATH)/Recast/Include \
					$(LOCAL_PATH)/RecastNavDll \
					$(LOCAL_PATH)/Tests \

# --------------------------- 标准库支持 ---------------------------------------#

# 添加C++ stl 头文件支持
# LOCAL_C_INCLUDES := $(NDK_ROOT)/sources/cxx-stl/stlport/stlport

# LOCAL_STATIC_LIBRARIES := $(NDK_ROOT)/sources/cxx-stl/stlport/libs/armeabi/libstlport_static.a

LOCAL_LDLIBS += -lm
LOCAL_LDLIBS += -landroid
LOCAL_LDLIBS += -llog

# ----------------------------配置文件生成类型 ---------------------------------#

# 生成可执行文件
# include $(BUILD_EXECUTABLE) 

# 生成动态链接库
include $(BUILD_SHARED_LIBRARY)