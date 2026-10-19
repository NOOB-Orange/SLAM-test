# 离线构建依赖

为避免 AutoDL 直连 GitHub codeload 大文件时中断，仓库包含这两个校验过的源码包。

- `orb_slam3-source.tar.gz`：来自 UZ-SLAMLab/ORB_SLAM3 官方提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` 的压缩包。保留全部 `src`、`include`、`Thirdparty`、`Vocabulary`，以及 CMake 配置需要的 Examples/Examples_old 源码和配置；省略示例传感器数据、评估真值等。本项目算法源码未在此包中修改，导出补丁由单独脚本应用。保留官方 LICENSE、README、Dependencies.md 及第三方许可证。
- `pangolin-source.tar.gz`：Pangolin 官方 `v0.8` 源码压缩包原样复制。

出处、原始官方包 SHA256、离线包 SHA256 记录在仓库根目录 `dependencies.json`。

来源：

- https://github.com/UZ-SLAMLab/ORB_SLAM3
- https://github.com/stevenlovegrove/Pangolin

使用各自随包保留的许可证；ORB-SLAM3 为 GPLv3。本目录只包含源码与官方 ORB 词典，不含编译二进制。

维护者可从已下载、通过官方包哈希校验的 `third_party/orb_slam3.tar.gz` 和 `third_party/pangolin.tar.gz` 重新打包：

```bash
python scripts/package_dependencies.py
```

打包只从原始官方归档读取，不会将本地已打补丁的源码重新打包进去。
