#pragma once

// 图像处理调试显示开关。
// 只有“图像处理测试”按钮执行期间才置为 true，
// 正常摄像头/视频处理时保持 false，避免弹出大量 OpenCV 窗口。
extern bool g_showImageDebug;
