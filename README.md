# ORB-SLAM3 单目展厅视频重建与三栏可视化

本项目使用 **UZ-SLAMLab 官方 ORB-SLAM3** 从原视频重新计算相机位姿和稀疏地图，输出与老师示例相同结构的三栏演示视频。不是把以前的 COLMAP 点云改名；不会读取 COLMAP 的 `images.txt`、`points3D.txt` 或调用 COLMAP。

目标环境：**AutoDL、Ubuntu 22.04、x86_64、至少 16 GB 内存，建议 32 GB**。4090 可以使用，但官方 ORB-SLAM3 核心主要使用 CPU，不会自动使用 CUDA。请选择 CPU 核数和内存充足的实例；此项目不需要 PyTorch、ROS、桌面或 X11 窗口。

原视频已放在 `data/original.mp4`，约 18.7 MB；不需要另外上传视频。官方依赖源码由构建脚本从官方 GitHub 下载，不将巨大的依赖和词袋文件重复上传到本仓库。

## AutoDL 上直接运行

创建实例时选择 **Ubuntu 22.04** 镜像，打开终端。项目放在数据盘中：

```bash
cd /root/autodl-tmp
git clone https://github.com/NOOB-Orange/SLAM-test.git
cd SLAM-test

# 首次安装依赖并编译；默认并行 4 个任务，内存不足可改成 JOBS=2。
JOBS=4 bash scripts/bootstrap_autodl.sh

# 运行原视频：准备全部帧 → ORB-SLAM3 → 地图着色导出 → 渲染 MP4。
bash scripts/run_video.sh
```

最后的视频：

```text
/root/autodl-tmp/SLAM-test/outputs/exhibition/slam_demo.mp4
```

可在 AutoDL 文件面板下载。编译需要联网；官方源码压缩包本身约 600 MB，依赖、编译中间文件和图像序列还需数 GB 空间。首次编译耗时取决于网络、CPU 和内存。

脚本遇到错误会停止，不会把未跟踪成功的片段当作有效轨迹。**本仓库当前只完成了 Windows 下的 Python 管线测试、真实输入抽帧检查、Bash 语法检查和固定版本官方源码补丁检查；没有在本机完成 Ubuntu C++ 编译或真实 ORB-SLAM3 重建，因此不承诺视频能一次跟踪到底。** AutoDL 首次运行会给出实际的编译与跟踪结果。GitHub Actions 中的 `Build official ORB-SLAM3 on Ubuntu` 工作流也可手动触发编译。

## 输出布局

```text
+--------------------------+--------------------------+
| INPUT FRAME              | FIRST-PERSON MAP         |
| 送入 SLAM 的视频图像      | 当前位姿看到的三维地图   |
+--------------------------+--------------------------+
| GLOBAL MAPPING PROGRESS                              |
| 彩色点云 + 黄色相机轨迹 + 绿色当前相机               |
+-----------------------------------------------------+
```

默认输出 1280×720、无音频。第一视角投影使用当前帧颜色、相机畸变和深度缓冲。全局地图使用关键帧原图采样得到的固定颜色。显示的是 ORB 稀疏地图，不能保证具有示例中的稠密程度；放大绘制点不会增加几何信息。

## 与上次 COLMAP 版本的区别

| 环节 | 本次实现 |
|---|---|
| 三维点与位姿来源 | 官方 ORB-SLAM3 的跟踪、局部建图和回环优化 |
| 输入 | 默认原视频全部帧，保留真实时间戳；不再每 10 帧抽一张 |
| 相机参数 | 暂时复用以前 COLMAP 估计的内参，作为起始标定；不复用其地图/位姿 |
| 无窗口运行 | `System::MONOCULAR`，`bUseViewer=false` |
| 逐帧轨迹 | 根据帧到参考关键帧的相对位姿与优化后的参考关键帧，重新恢复 `Tcw` |
| 跟踪丢失 | 记录状态，仅接受 `OK=2`，右上角显示初始化/丢失状态，不跨越丢失段插值 |
| 多子地图 | 分开导出，不把独立坐标系强行叠在一起 |
| 可视化 | 复用之前验证过的透视投影、颜色采样、点绘制和固定第三人称思路 |

纯单目没有已知尺度，坐标单位不能直接当成米。本项目不使用 IMU，不是单目惯性模式。

## 相机标定

起始配置在 `configs/camera.json`：

```text
原标定分辨率：1280 × 720
fx = fy = 559.94053152396134
cx = 640, cy = 360
k1 = -0.0091275795262995545
k2 = p1 = p2 = k3 = 0
```

原来的 `SIMPLE_RADIAL` 一项径向畸变可对应 ORB-SLAM3/OpenCV 针孔模型的 `k1`，其余系数为 0。`prepare_video.py` 将内参写入官方 `File.version: "1.0"`、`Camera.type: "PinHole"` 配置，使用 `Camera.RGB: 0` 匹配 OpenCV BGR 输入。

这些参数来自已有 SfM 估计，**不是独立完成的物理相机标定**。如果初始化困难、转弯时经常丢失或地图扭曲，应检查标定、视频防抖/数字变焦、运动模糊和场景纹理。不能单靠换 4090 保证解决。

换视频需要提供对应标定，不应沿用这套内参。改变输入分辨率时，准备脚本会同步缩放焦距和主点；不接受与标定长宽比不一致的裁剪。

## 主要文件

```text
configs/camera.json             起始相机标定和 ORB 特征参数
data/original.mp4               本次原始视频
dependencies.json              官方源码版本及压缩包 SHA256
scripts/bootstrap_autodl.sh     Ubuntu 安装、编译、运行测试
scripts/fetch_dependencies.py   官方源码下载、校验、安全解压
scripts/patch_upstream.py       添加导出接口；幂等，不改跟踪/建图算法
scripts/run_video.sh            完整运行流程
cpp/mono_video.cc              逐帧输入、跟踪状态记录、无窗口主程序
cpp/ExportReplay.cc            停止工作线程后导出地图和优化后的逐帧位姿
prepare_video.py               按时间戳导出所有帧，生成相机 YAML
export_map.py                  关键帧着色、按子地图打包、跟踪统计
render_core.py                 相机投影、坐标变换、点精灵深度缓冲
render_video.py                三栏视频输出，丢失段不插值
tests/test_pipeline.py         合成数据的管线回归检查
```

官方接口的一个细节：`SaveTrajectoryTUM()` 在纯单目模式下会直接拒绝保存。本项目提供单独导出接口，不依赖该函数。另一个细节：锁定版本 `Shutdown()` 中等待建图线程的代码被注释，因此导出器在读取最终结果前显式等待并 join 建图/回环线程，避免边优化边导出。

地图点和 `Tcw` 使用同一个原始子地图坐标系，未单独把相机轨迹归零。否则会出现轨迹正常、地图却错位的问题。

## 结果文件说明

```text
outputs/exhibition/
  orb_slam3.log                 官方后端运行日志
  dataset/
    frames.tsv                 原视频帧编号、时间戳、图像路径
    camera.yaml                真正传给后端的配置
    camera.json
    metadata.json              视频 SHA256、尺寸、FPS 等
    rgb/*.jpg                  全帧序列，JPEG 质量 95
  raw/
    tracking.csv               每个输入帧的跟踪状态及耗时
    maps.csv                   各子地图的关键帧和有效点数
    keyframes.csv              最终关键帧位姿，世界→相机
    poses.csv                  最终恢复的有效逐帧位姿，世界→相机
    points.csv                 地图点及最早保留观测时间
    observations.csv           用于 RGB 采样的原始畸变图像像素位置
  replay/
    summary.json               所有子地图、状态统计、覆盖率、默认地图
    frame_status.csv
    map_0/
      map.npz                  坐标、颜色、点编号、显现时间
      map.ply                  可另行检查的彩色点云
      poses.csv                此子地图的有效逐帧位姿
    map_.../
  slam_demo.mp4
  slam_demo.report.json         输出帧数、FPS、有效位姿覆盖率等
  slam_demo_frame_*.jpg         首帧/中间帧/末帧预览
```

`tracking.csv` 的状态：`1=尚未初始化`、`2=正常跟踪`、`3=近期丢失`、`4=丢失`。即使后端在近期丢失时仍有预测位姿，也不会当作可靠位姿画入最终视频。

默认选择**有效跟踪帧最多**的子地图；其余地图仍分别保留。视频落在其他子地图的时间段会标为 `OUTSIDE SELECTED MAP`，不会暗中拼接坐标系。若出现许多子地图，需先检查跟踪失败原因，不能把它当成完整单地图重建成功。

下方“地图逐步增加”是最终优化结果的离线回放：按点最早保留的关键帧观测时间显现，不是在线建图历史的精确重放，也不保存已经被剔除的点。

## 分步运行与排错

完整流程拒绝覆盖已有跟踪结果。再次尝试请换输出目录：

```bash
bash scripts/run_video.sh data/original.mp4 outputs/attempt_02
```

只准备一个短片段检查输入：

```bash
.venv/bin/python prepare_video.py --video data/original.mp4 \
  --output outputs/smoke/dataset --max-frames 300
```

短片段是否有足够运动完成单目初始化取决于实际画面；没有初始化不能当作编译失败。

已经完成跟踪后，可单独重做后处理和渲染。注意选择新的导出目录与输出文件：

```bash
.venv/bin/python export_map.py --dataset outputs/exhibition/dataset \
  --raw outputs/exhibition/raw --output outputs/recolor

.venv/bin/python render_video.py --dataset outputs/exhibition/dataset \
  --replay outputs/recolor --output outputs/recolored_demo.mp4
```

指定另一个子地图进行渲染，在后一条命令中增加 `--map-id 1`。只渲染前 150 帧可加 `--max-frames 150`。

如编译出现 `Killed signal terminated program cc1plus`，通常是内存不足，使用 `JOBS=2` 或 `JOBS=1` 重试构建。源码不会上传编译好的 `.so`，以避免不同机器的 ABI/指令集问题；每台 AutoDL 实例在本机编译。

Python 与 C++ 使用两套 OpenCV：C++ 链接 apt 的 `libopencv-dev`，Python 虚拟环境使用 `opencv-python-headless`，两者无需版本完全相同。

若旧版本安装日志显示 `http://mirrors.aliyun.com/pypi/simple`，随后出现 `numpy ... (from versions: none)`，表示 pip 从当前索引没有获得可用的安装包，不能据此判断为 Python 3.10 不兼容。新版构建脚本明确使用 HTTPS 阿里云索引，失败后尝试官方 PyPI；不更改全局 pip 配置。

在服务器更新并重新执行即可，已经完成的 apt 安装不需要撤销：

```bash
git pull --ff-only
JOBS=4 bash scripts/bootstrap_autodl.sh
```

也可以优先使用官方索引：

```bash
SLAM_PIP_INDEX_URL=https://pypi.org/simple JOBS=4 bash scripts/bootstrap_autodl.sh
```

`ldconfig ... libnvidia-*.so ... is empty, not checked` 与上述 pip 索引失败属于不同环节。如果安装继续到 pip，说明这些提示没有使本次 apt 阶段停止；仅凭这些信息不能确认容器的 GPU/图形库完整性。不要为解决 NumPy 下载失败删除 NVIDIA 库或重装驱动。后续若 CMake 或动态链接明确报 GL/EGL/NVIDIA 库错误，再根据那条实际错误检查容器环境。

本地测试命令：

```bash
python -m unittest discover -s tests -v
bash -n scripts/bootstrap_autodl.sh scripts/run_video.sh
```

测试使用明确的合成后端导出数据来检查标定缩放、全帧时间戳、颜色、独立子地图、丢失帧过滤、轨迹分段、视频完整解码和补丁幂等性；测试通过不代表真实视频已经由 ORB-SLAM3 成功跟踪。

## 来源与许可证

- [ORB-SLAM3 官方仓库](https://github.com/UZ-SLAMLab/ORB_SLAM3)，固定提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。
- [Pangolin 官方仓库](https://github.com/stevenlovegrove/Pangolin)，固定 `v0.8` 压缩包及 SHA256。
- [ORB-SLAM3 官方相机配置示例](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Monocular/TUM1.yaml)。

代码采用 GPL-3.0-or-later，见 `LICENSE`。下载的第三方依赖保留各自许可证。学术使用 ORB-SLAM3 请引用 Carlos Campos 等人的 *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multi-Map SLAM*, IEEE Transactions on Robotics, 2021。
