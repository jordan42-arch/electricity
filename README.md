# BUPT 2026 Car Vision Module

这是校内选拔小车赛道项目里已经完成并验证过的 K230/CanMV 视觉模块整理版。

## 当前完成内容

- 确认车上透明亚克力外壳的立创开发板是 K230/CanMV 视觉板。
- 确认可用摄像头为 `Sensor(id=2)`，图像格式使用 320x240 灰度图。
- 完成靶标检测程序，当前主要识别白色靶面区域，并保留矩形/圆形靶心检测的备用逻辑。
- 完成 K230 到 MSPM0G3507/TI 主控的 UART 输出协议。
- 完成安全测试脚本：只识别和打印结果，不控制云台、舵机或激光。
- 完成正式连续运行脚本：持续识别靶标，并约每 50 ms 通过 UART 输出一次数据。

注意：本仓库里的视觉模块负责“打靶/靶标定位”，不是负责小车巡线。巡线建议由 MSPM0G3507 主控读取灰度传感器完成，K230 只把靶标相对画面中心的偏差发给主控。

## 目录结构

```text
vision/k230_target/
  main.py                 # K230 正式视觉程序，持续识别并通过 UART 输出
  safe_test.py            # K230 安全测试程序，有限帧运行，不控制外设
  protocol.md             # K230 -> MSPM0G3507 串口协议说明
  tools/
    upload_canmv_file.ps1 # 从电脑上传脚本到 K230
    send_canmv_exec.ps1   # 临时发送脚本到 K230 REPL 执行
```

## 硬件分工

- `MSPM0G3507 / TI 主控`：整车主控，负责巡线、电机 PID、状态机、云台/舵机/激光控制。
- `灰度传感器`：负责巡线，输出给 MSPM0G3507。
- `K230 / CanMV`：负责打靶视觉识别，输出靶标偏差 `dx/dy`。

## UART 接线

- K230 UART2 TX pin: `11`
- K230 UART2 RX pin: `12`
- 波特率：`115200`
- 格式：`8N1`

接线：

```text
K230 TX  -> MSPM0G3507 RX
K230 RX  -> MSPM0G3507 TX，可选
K230 GND -> MSPM0G3507 GND
```

不要把电机电源或电池电压接到信号引脚。

## 输出格式

K230 约每 50 ms 输出一行：

```text
@V,valid,dx,dy,confidence,label
```

示例：

```text
@V,1,-10,10,16800,WHITE_TARGET
```

含义：

- `valid`：`1` 表示识别有效，`0` 表示当前没有可靠靶标。
- `dx`：靶标中心 x 坐标减去图像中心 x 坐标，单位像素。
- `dy`：靶标中心 y 坐标减去图像中心 y 坐标，单位像素。
- `confidence`：当前识别强度，白色靶面模式下主要是像素数量。
- `label`：识别方式，常见为 `WHITE_TARGET`。

方向约定：

- `dx > 0`：靶标在画面中心右侧。
- `dx < 0`：靶标在画面中心左侧。
- `dy > 0`：靶标在画面中心下方。
- `dy < 0`：靶标在画面中心上方。

## 使用方法

安全测试，不落盘到 K230：

```powershell
powershell -ExecutionPolicy Bypass -File .\vision\k230_target\tools\send_canmv_exec.ps1 -Port COM5 -File .\vision\k230_target\safe_test.py
```

上传正式程序到 K230：

```powershell
powershell -ExecutionPolicy Bypass -File .\vision\k230_target\tools\upload_canmv_file.ps1 -Port COM5 -LocalFile .\vision\k230_target\main.py -RemoteFile /sdcard/k230_target.py
```

在 CanMV REPL 中运行：

```python
exec(open("/sdcard/k230_target.py").read())
```

正式比赛前，确认稳定后再考虑是否覆盖 `/sdcard/main.py` 做开机自启动。

## 后续联调任务

- MSPM0G3507 端先只接收并打印 `@V` 数据，确认串口通信通了。
- 解析 `valid/dx/dy/confidence/label`。
- 小车巡线到达打靶区域后，MSPM0G3507 进入打靶状态。
- 根据 `dx/dy` 控制云台舵机修正角度。
- 当 `abs(dx)` 和 `abs(dy)` 小于设定阈值时，再执行激光打靶动作。

