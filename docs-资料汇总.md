# 奥比中光 Unity 骨骼姿态检测 · 示例程序与资料汇总

> 检索时间：本次会话 ｜ 目标机型：**Astra 系列（Astra Pro / Astra+ 等）**
> ⚠️ 本机出网被封（所有域名被解析到 `198.18.0.0/15` 过滤地址，curl/PowerShell TLS 全部失败，`web_fetch` 因私有 IP 校验拒绝），
> 因此以下链接**未经实际打开验证**，均来自搜索引擎索引结果，请在浏览器中自行确认。

---

## 0. 先搞清楚一件事

奥比中光**没有一个名为「骨骼姿态检测」的官方 Unity 示例工程**。它的骨骼能力分三条完全不同的技术链路，Astra 系列走的是和 Femto 系列**完全不同**的路线：

| 机型 | 骨骼方案 | Unity 支持度 |
|---|---|---|
| Femto Mega / Femto Bolt | K4A Wrapper + **Azure Kinect Body Tracking SDK** | 好（可复用微软/社区 Unity 工程） |
| **Astra / Astra Pro / Persee** | **Orbbec Body Tracking SDK**（奥比中光自研，官方称兼容 Astra、Persee 系列） | 一般（无官方 Unity 示例，需自封 C#） |
| Astra / Astra Pro / Astra Pro Plus | **Nuitrack**（第三方中间件） | 好（原生 Unity 插件 + 示例） |

**Astra 系列结论：优先 Nuitrack（有现成 Unity 示例），次选奥比中光 Body Tracking SDK + 自写 C# 封装。**

---

## 1. 路线一（推荐）：Nuitrack + Astra

Nuitrack 是 3DiVi 出的 3D 骨骼追踪中间件，官方支持奥比中光 Astra 系列，提供 Unity 原生插件、骨骼预制体和若干示例项目，是 Astra 上最省事的骨骼方案。

| 资源 | 链接 |
|---|---|
| Nuitrack 官网（下载 SDK + 申请 License；个人非商用有免费许可） | https://nuitrack.com/ |
| Nuitrack SDK 源码/镜像 | https://github.com/roger1mjh/nuitrack-sdk |
| 社区：**在 Unity 中选取指定骨骼（Orbbec Astra Pro）** ← 直接对口的实战帖 | https://community.nuitrack.com/t/select-a-specific-skeleton-unity-orbbec-astra-pro/460/4 |
| 社区：Holistic Skeleton Tracking（Unity3D 全身骨骼） | https://community.nuitrack.com/t/holistic-skeleton-tracking/3874 |
| 社区：Unity 工程识别不到相机（Astra 常见坑） | https://community.nuitrack.com/t/unity-application-does-not-recognize-the-sensor/3628/2 |
| Unity Asset Store 官方示例游戏（可当工程模板学） | https://assetstore.unity.com/packages/templates/tutorials/nuitrack-ar-football-game-126151 |

**为什么推荐：** 骨骼解算、关节映射、多人区分、Unity 预制体都现成，示例工程能直接跑起来改。

**注意：** Astra Pro Plus（较新批次）在部分 Nuitrack 版本上需要额外补丁/驱动才能被识别，入手前先在社区确认你的具体型号与固件。

---

## 2. 路线二：奥比中光自研 Body Tracking SDK（Astra / Persee）

奥比中光的 Body Tracking SDK 官方说明兼容自家 Astra、Persee 系列；但它是**原生 C/C++ SDK，没有官方 Unity 封装或示例**，需要自己写 P/Invoke 或 C++ 插件。

| 资源 | 链接 |
|---|---|
| 论坛：**Body Tracking License for Astra SDK**（授权/获取方式，必读） | https://3dclub.orbbec3d.com/t/body-tracking-license-for-astra-sdk/4534/2 |
| 论坛：Astra SDK 2.0.7 BodyFrame / 骨骼数据流用法 | https://3dclub.orbbec3d.com/t/astra-sdk-2-0-7-persee-android-bodyframe-from-streamreader-framelistener/1289 |
| 官方 OpenNI_SDK（Astra Pro 经典驱动，README 有接线说明） | https://github.com/orbbec/OpenNI_SDK |
| 老一代 Astra Pro + OpenNI2/NITE 骨骼（社区常用组合） | 检索关键词：`Astra Pro OpenNI2 NITE 骨骼 Unity` |

**Unity 集成思路：** 编译出 `BodyTracking.dll` → 在 Unity 里用 `DllImport` 调用「初始化 → 每帧取 BodyFrame → 读关节数组」→ 用 `LineRenderer`/骨骼预制体把 25 个左右关节连起来。

---

## 3. 路线三：Astra Pro 体感开发实战（中文教程）

- CSDN《奥比中光 Astra Pro 体感开发实战：从 SDK 环境搭建到骨骼追踪》
  https://blog.csdn.net/weixin_29075633/article/details/164237925
- CSDN《几种奥比中光 SDK 的功能说明以及使用方法》（先搞清楚该装哪个 SDK）
  https://blog.csdn.net/limingmin2020/article/details/125199203
- 奥比中光官方开发者文档入口
  https://www.orbbec.com.cn/tempFile/sdkak/akdk.html?id=34
- GitHub `jackfan108/astra-linux`（Astra 驱动/Linux 侧参考）
  https://github.com/jackfan108/astra-linux
- GitHub `alexwang0311/Augmented-Reality-Mirror`（用 Astra 做的 AR 镜子项目，含 Poster 论文，可参考取流与渲染）
  https://github.com/alexwang0311/Augmented-Reality-Mirror/blob/master/Poster.pdf

---

## 4. 对照参考：如果以后换成 Femto Mega / Femto Bolt

这条链路资料更全，是奥比中光「官方骨骼示例」实际存在的地方（复用微软 Azure Kinect 生态）：

| 环节 | 链接 |
|---|---|
| 奥比中光官方 Unity SDK（含示例场景） | https://gitee.com/orbbecdeveloper/OrbbecUnitySDK/ ｜ https://github.com/orbbec/OrbbecUnitySDK |
| K4A Wrapper（把相机伪装成 Azure Kinect，骨骼的关键） | https://github.com/orbbec/OrbbecSDK-K4A-Wrapper ｜ https://orbbec.github.io/OrbbecSDK-K4A-Wrapper/ |
| 官方说明 PDF《Access AKDK Application Software with Femto Bolt》 | https://orbbec.github.io/OrbbecSDK-K4A-Wrapper/src/orbbec/docs/Access_AKDK_Application_Software_with_Femto_Bolt.pdf |
| Azure Kinect Body Tracking SDK 下载 | https://learn.microsoft.com/azure/kinect-dk/body-sdk-download |
| 微软官方 Unity 骨骼示例（Azure-Kinect-Samples） | https://github.com/microsoft/Azure-Kinect-Samples |
| 社区现成 Unity 骨骼工程（C# 封装） | https://github.com/liuliyang2310/azure-kinect-dk-unity ｜ https://github.com/shawhu/azure-kinect-dk-unity |
| 付费开箱插件（Femto Bolt/Mega + Azure Kinect 骨骼） | https://assetstore.unity.com/packages/tools/integration/body-tracking-for-orbbec-femto-bolt-mega-azure-kinect-157915 |
| 官方演示视频（DigiKey，中文页） | https://www.digikey.hk/zh/videos/o/orbbec/orbbec-femto-megabolt-with-azure-kinect-unity-samples |
| 官方演示视频（B 站搬运） | https://www.bilibili.com/video/BV1Fc411b7mE/ |

---

## 5. 待你本机确认的三件事

1. **型号精确到后缀**：Astra Pro / Astra Pro Plus / Astra+ / Astra 2 —— 后缀不同，驱动与骨骼方案差别很大。
2. **Nuitrack License**：非商用免费，商用需付费；先在官网确认你的型号在支持列表内。
3. **奥比中光 Body Tracking SDK 的授权**：论坛帖标题就是 License 问题，先问清楚再投入开发。
