# AG-95 夹爪 CAN2.0A 通信协议整理

> 整理范围：AG-95 产品手册第 12-20 页，对应文档底部页码第 5-13 页，即第 3 章 `Connect and control` 中的通信协议部分。  
> 说明：AG-95 夹爪本体使用 CAN2.0A 通信；若通过 USB、TCP/IP、RS485 等接口控制，需要先经过协议转换器，转换器再与夹爪进行 CAN2.0A 通信。

---

## 1. 通信总体结构

AG-95 夹爪通过航空插头接入 CAN 网络，支持 CAN2.0A 协议。

如果控制系统本身支持 CAN2.0A，可以直接连接夹爪，不需要协议转换器。

如果控制系统不支持 CAN，可以使用 DH 提供的协议转换器，将下列接口转换为 CAN2.0A：

- USB
- TCP/IP
- RS485
- I/O

通信结构可以理解为：

```text
PC / PLC / Robot
      |
      | USB / TCP-IP / RS485 / I-O
      v
协议转换器
      |
      | CAN2.0A
      v
AG-95 夹爪
```

直接 CAN 控制时：

```text
PC / PLC / Robot
      |
      | CAN2.0A
      v
AG-95 夹爪
```

---

## 2. 通信逻辑

### 2.1 接收成功反馈

夹爪成功接收到命令后，通常会返回相同的数据作为反馈。

例如，写入位置、力等命令成功后，返回帧内容通常与发送帧一致。

### 2.2 初始化完成自动反馈

如果开启了初始化完成反馈，夹爪初始化成功后会自动返回初始化成功标志。

### 2.3 初始化可被中断

初始化过程中，如果再次发送新的初始化命令，当前初始化过程会被中断，并重新开始新的初始化流程。

因此，程序中建议先读取初始化状态标志，避免频繁打断初始化过程。

### 2.4 位置指令可被中断

夹爪运动过程中，如果发送新的位置指令，夹爪会中断当前运动，并转向新的目标位置。

因此，程序中建议读取当前状态，避免频繁打断夹爪运动。

### 2.5 设置类命令成功后才返回

CAN ID、CAN 波特率、I/O 模式等设置类命令，夹爪不会立即返回原始帧，而是在设置成功后才返回。

### 2.6 命令发送间隔

建议相邻两条命令的发送间隔大于：

```text
20 ms
```

### 2.7 掉落检测限制

掉落检测功能要求被夹持物体直径大于：

```text
5 mm
```

---

## 3. 协议帧格式

AG-95 的通信命令核心是 8 字节数据段。  
区别在于：

- CAN2.0A 直接通信时，只发送 8 字节 CAN 数据段；CAN ID 等于夹爪 ID。
- USB、TCP/IP、RS485 通信时，需要在 8 字节数据段外加帧头、夹爪 ID 和帧尾，总长度为 14 字节。

---

### 3.1 USB / TCP/IP / RS485 帧格式

用于外部接口经过协议转换器控制夹爪时。

总长度：

```text
14 Bytes
```

帧结构如下：

| 字段 | 长度 | 固定值 / 说明 |
|---|---:|---|
| Frame Header | 4 Bytes | `FF FE FD FC` |
| Gripper ID | 1 Byte | 夹爪 CAN ID，默认 `01`，范围 `0-255` |
| Function Register | 1 Byte | 功能寄存器 |
| Sub-Function Register | 1 Byte | 子功能寄存器 |
| Read/Write | 1 Byte | `00` 读，`01` 写 |
| Reserve | 1 Byte | 保留字节，固定 `00` |
| Data | 4 Bytes | 32 位有符号整数，小端格式 |
| Frame End | 1 Byte | `FB` |

通用格式：

```text
FF FE FD FC | ID | FUNC | SUB_FUNC | RW | 00 | DATA_0 DATA_1 DATA_2 DATA_3 | FB
```

示例：初始化夹爪。

```text
FF FE FD FC 01 08 02 01 00 00 00 00 00 FB
```

---

### 3.2 CAN2.0A 帧格式

用于直接通过 CAN 网络控制夹爪时。

CAN ID：

```text
CAN ID = Gripper ID
```

默认夹爪 ID：

```text
0x01
```

CAN 数据段长度：

```text
8 Bytes
```

CAN 数据段结构：

| Byte | 字段 | 说明 |
|---:|---|---|
| 0 | Function Register | 功能寄存器 |
| 1 | Sub-Function Register | 子功能寄存器 |
| 2 | Read/Write | `00` 读，`01` 写 |
| 3 | Reserve | 保留字节，固定 `00` |
| 4-7 | Data | 32 位有符号整数，小端格式 |

通用 CAN 数据段：

```text
FUNC | SUB_FUNC | RW | 00 | DATA_0 DATA_1 DATA_2 DATA_3
```

例如，初始化夹爪时：

```text
CAN ID : 0x01
DLC    : 8
Data   : 08 02 01 00 00 00 00 00
```

---

## 4. 字段说明

### 4.1 Frame Header

仅用于 USB、TCP/IP、RS485 帧。

固定为：

```text
FF FE FD FC
```

协议转换器通过该字段判断一条命令的起始位置。

### 4.2 Gripper ID / CAN ID

夹爪 ID 是夹爪实际使用的 CAN ID。

默认值：

```text
0x01
```

范围：

```text
0-255
```

通过 CAN2.0A 直接控制时，该 ID 就是 CAN 帧的仲裁 ID。

通过 USB、TCP/IP、RS485 控制时，该 ID 是 14 字节协议帧中的第 5 个字节。

### 4.3 Function Register

用于表示主功能类别。

例如：

- `0x08`：初始化
- `0x05`：力控制
- `0x06`：位置控制
- `0x0F`：状态反馈

### 4.4 Sub-Function Register

用于表示某个主功能下的具体子功能。

例如：

- `0x08 0x01`：初始化完成反馈设置 / 读取
- `0x08 0x02`：初始化执行 / 初始化状态读取
- `0x06 0x02`：位置设置 / 读取

### 4.5 Read/Write

| 值 | 含义 |
|---|---|
| `00` | 读命令 |
| `01` | 写命令 |

只允许使用 `00` 和 `01`。

### 4.6 Reserve

保留字节，固定为：

```text
00
```

### 4.7 Data

数据字段为 4 字节，表示 32 位有符号整数。

字节序：

```text
Little Endian，小端格式
```

示例：

| 十进制值 | 十六进制小端表示 |
|---:|---|
| `1` | `01 00 00 00` |
| `60` | `3C 00 00 00` |
| `90` | `5A 00 00 00` |
| `100` | `64 00 00 00` |
| `-1` | `FF FF FF FF` |

读命令中，Data 字段不参与控制，一般填：

```text
00 00 00 00
```

### 4.8 Frame End

仅用于 USB、TCP/IP、RS485 帧。

固定为：

```text
FB
```

协议转换器通过该字段判断一条命令的结束位置。

---

## 5. 命令总览

| 功能 | Function Register | Sub-Function Register | 说明 |
|---|---|---|---|
| Initialization | `0x08` | `0x01-0x02` | 初始化相关命令 |
| Force | `0x05` | `0x02-0x03` | 读取 / 设置夹爪打开或闭合力 |
| Position | `0x06` | `0x02` | 读取 / 设置夹爪位置 |
| Feedback | `0x0F` | `0x01` | 读取当前夹爪状态 |
| I/O Mode | `0x10` | `0x01-0x0B` | 设置 / 读取 I/O 模式参数 |
| CAN ID | `0x12` | `0x01` | 读取 / 设置夹爪 CAN ID |
| Firmware Version | `0x13` | `0x01` | 读取夹爪固件版本 |
| CAN Baud Rate | `0x14` | `0x01` | 读取 / 设置夹爪 CAN 波特率 |
| Object Dropped Detection | `0x15` | `0x01/0x02` | 掉落检测相关命令 |

---

## 6. 详细命令

以下示例默认使用：

```text
Gripper ID = 0x01
```

如果是 CAN2.0A 直接通信，则只取示例中的 8 字节数据段作为 CAN 数据，CAN ID 使用夹爪 ID。

例如 USB / TCP / RS485 完整帧：

```text
FF FE FD FC 01 06 02 01 00 3C 00 00 00 FB
```

对应 CAN2.0A 帧为：

```text
CAN ID : 0x01
Data   : 06 02 01 00 3C 00 00 00
```

---

## 6.1 初始化命令

Function Register：

```text
0x08
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01` | `00/01` | Integer | 读取 / 设置初始化完成反馈 |
| `0x02` | `00/01` | Integer | 执行初始化 / 读取初始化是否完成 |

---

### 6.1.1 设置初始化完成反馈

Sub-Function：

```text
0x01
```

写入命令示例：

```text
Send    : FF FE FD FC 01 08 01 01 00 00 00 00 00 FB
Receive : FF FE FD FC 01 08 01 01 00 A5 00 00 00 FB
```

对应 CAN 数据段：

```text
08 01 01 00 00 00 00 00
```

返回数据中：

```text
A5 00 00 00
```

表示已设置或已开启初始化完成反馈。

---

### 6.1.2 读取是否开启初始化完成反馈

读命令：

```text
Send : FF FE FD FC 01 08 01 00 00 00 00 00 00 FB
```

如果已开启：

```text
Receive : FF FE FD FC 01 08 01 00 00 A5 00 00 00 FB
```

如果未开启：

```text
Receive : FF FE FD FC 01 08 01 00 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
08 01 00 00 00 00 00 00
```

---

### 6.1.3 执行初始化

Sub-Function：

```text
0x02
```

写命令：

```text
Send    : FF FE FD FC 01 08 02 01 00 00 00 00 00 FB
Receive : FF FE FD FC 01 08 02 01 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
08 02 01 00 00 00 00 00
```

如果已经设置初始化完成反馈，初始化完成后夹爪会自动返回：

```text
Receive : FF FE FD FC 01 08 02 00 00 00 00 00 00 FB
```

---

### 6.1.4 读取初始化是否完成

读命令：

```text
Send : FF FE FD FC 01 08 02 00 00 00 00 00 00 FB
```

初始化已完成：

```text
Receive : FF FE FD FC 01 08 02 00 00 01 00 00 00 FB
```

初始化未完成：

```text
Receive : FF FE FD FC 01 08 02 00 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
08 02 00 00 00 00 00 00
```

返回 Data 含义：

| Data | 含义 |
|---|---|
| `01 00 00 00` | 初始化已完成 |
| `00 00 00 00` | 初始化未完成 |

---

## 6.2 力控制命令

Function Register：

```text
0x05
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x02/0x03` | `00/01` | Integer | 读取 / 设置夹爪打开或闭合力 |

手册示例中，`0x02` 用于闭合夹持力。

力值范围：

```text
20-100
```

对应十六进制小端：

```text
14 00 00 00 - 64 00 00 00
```

---

### 6.2.1 设置 30% 闭合夹持力

十进制：

```text
30
```

十六进制：

```text
0x1E
```

完整帧：

```text
Send    : FF FE FD FC 01 05 02 01 00 1E 00 00 00 FB
Receive : FF FE FD FC 01 05 02 01 00 1E 00 00 00 FB
```

对应 CAN 数据段：

```text
05 02 01 00 1E 00 00 00
```

---

### 6.2.2 读取当前闭合夹持力

完整帧：

```text
Send    : FF FE FD FC 01 05 02 00 00 00 00 00 00 FB
Receive : FF FE FD FC 01 05 02 00 00 1E 00 00 00 FB
```

对应 CAN 数据段：

```text
05 02 00 00 00 00 00 00
```

返回 Data：

```text
1E 00 00 00 = 30
```

---

## 6.3 位置控制命令

Function Register：

```text
0x06
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x02` | `00/01` | Integer | 读取 / 设置夹爪位置 |

位置范围：

```text
0-100
```

对应十六进制小端：

```text
00 00 00 00 - 64 00 00 00
```

---

### 6.3.1 设置 60% 位置

十进制：

```text
60
```

十六进制：

```text
0x3C
```

完整帧：

```text
Send    : FF FE FD FC 01 06 02 01 00 3C 00 00 00 FB
Receive : FF FE FD FC 01 06 02 01 00 3C 00 00 00 FB
```

对应 CAN 数据段：

```text
06 02 01 00 3C 00 00 00
```

---

### 6.3.2 读取当前位置

完整帧：

```text
Send    : FF FE FD FC 01 06 02 00 00 00 00 00 00 FB
Receive : FF FE FD FC 01 06 02 00 00 3C 00 00 00 FB
```

对应 CAN 数据段：

```text
06 02 00 00 00 00 00 00
```

返回 Data：

```text
3C 00 00 00 = 60
```

---

## 6.4 状态反馈命令

Function Register：

```text
0x0F
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01` | `00` | Integer | 读取当前夹爪状态 |

---

### 6.4.1 读取当前状态

完整帧：

```text
Send : FF FE FD FC 01 0F 01 00 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
0F 01 00 00 00 00 00 00
```

返回 Data 含义：

| Data | 状态含义 |
|---|---|
| `00 00 00 00` | 默认状态或正在运动 |
| `02 00 00 00` | 已到达目标位置 / 角度，但未夹到物体 |
| `03 00 00 00` | 已夹到物体，但未到达目标位置 / 角度 |

示例返回：

默认或运动中：

```text
Receive : FF FE FD FC 01 0F 01 00 00 00 00 00 00 FB
```

已到达目标位置，但未夹到物体：

```text
Receive : FF FE FD FC 01 0F 01 00 00 02 00 00 00 FB
```

已夹到物体，但未到达目标位置：

```text
Receive : FF FE FD FC 01 0F 01 00 00 03 00 00 00 FB
```

> 注意：协议格式中 `Read/Write` 后还有 1 字节 `Reserve = 00`，状态值位于 4 字节 Data 字段中。因此这里按照协议结构写为 `... 00 00 02 00 00 00 FB`，即 `RW=00, Reserve=00, Data=02 00 00 00`。

---

## 6.5 I/O 模式相关命令

Function Register：

```text
0x10
```

I/O 模式是一种简单控制模式。协议转换器检测 I/O 输入引脚状态，并根据引脚状态向夹爪发送相应命令。

协议转换器有两个输入引脚，每个输入引脚有两种状态，因此总共可对应四种动作状态。

---

### 6.5.1 I/O 模式子功能表

| Sub-Function | R/W | 功能 |
|---|---|---|
| `0x01` | `00/01` | 读取 / 设置 Group 1 Position 1 |
| `0x02` | `00/01` | 读取 / 设置 Group 1 Position 2 |
| `0x03` | `00/01` | 读取 / 设置 Group 1 Force 1 |
| `0x04` | `01` | 控制 Group 1 grip |
| `0x05` | `00/01` | 读取 / 设置 Group 2 Position 1 |
| `0x06` | `00/01` | 读取 / 设置 Group 2 Position 2 |
| `0x07` | `00/01` | 读取 / 设置 Group 2 Force 1 |
| `0x08` | `01` | 控制 Group 2 grip |
| `0x09` | `00/01` | 读取 / 设置是否启用 I/O 模式 |
| `0x0A` | `00/01` | 读取 / 设置 Group 1 Force 2 |
| `0x0B` | `00/01` | 读取 / 设置 Group 2 Force 2 |

其中：

```text
Position 1 绑定 Force 1
Position 2 绑定 Force 2
```

`0x01-0x03`、`0x05-0x07`、`0x0A-0x0B` 用于写入目标位置和力值。  
位置和力值范围分别参考位置命令和力命令。

`0x04` 和 `0x08` 是 I/O 模式下协议转换器发送给夹爪的控制命令。

`0x09` 用于读写 I/O 模式开关状态。

---

### 6.5.2 启用 I/O 模式

完整帧：

```text
Send    : FF FE FD FC 01 10 09 01 00 01 00 00 00 FB
Receive : FF FE FD FC 01 10 09 01 00 01 00 00 00 FB
```

对应 CAN 数据段：

```text
10 09 01 00 01 00 00 00
```

Data：

```text
01 00 00 00 = 启用 I/O 模式
```

---

### 6.5.3 设置 I/O 模式参数示例

设置 Group 1 Position 1 = 0：

```text
Send    : FF FE FD FC 01 10 01 01 00 00 00 00 00 FB
Receive : FF FE FD FC 01 10 01 01 00 00 00 00 00 FB
```

设置 Group 1 Position 2 = 90：

```text
Send    : FF FE FD FC 01 10 02 01 00 5A 00 00 00 FB
Receive : FF FE FD FC 01 10 02 01 00 5A 00 00 00 FB
```

设置 Group 1 Force 1 = 90：

```text
Send    : FF FE FD FC 01 10 03 01 00 5A 00 00 00 FB
Receive : FF FE FD FC 01 10 03 01 00 5A 00 00 00 FB
```

设置 Group 1 Force 2 = 60：

```text
Send    : FF FE FD FC 01 10 0A 01 00 3C 00 00 00 FB
Receive : FF FE FD FC 01 10 0A 01 00 3C 00 00 00 FB
```

设置 Group 2 Position 1 = 30：

```text
Send    : FF FE FD FC 01 10 05 01 00 1E 00 00 00 FB
Receive : FF FE FD FC 01 10 05 01 00 1E 00 00 00 FB
```

设置 Group 2 Position 2 = 60：

```text
Send    : FF FE FD FC 01 10 06 01 00 3C 00 00 00 FB
Receive : FF FE FD FC 01 10 06 01 00 3C 00 00 00 FB
```

设置 Group 2 Force 1 = 60：

```text
Send    : FF FE FD FC 01 10 07 01 00 3C 00 00 00 FB
Receive : FF FE FD FC 01 10 07 01 00 3C 00 00 00 FB
```

设置 Group 2 Force 2 = 90：

```text
Send    : FF FE FD FC 01 10 0B 01 00 5A 00 00 00 FB
Receive : FF FE FD FC 01 10 0B 01 00 5A 00 00 00 FB
```

---

### 6.5.4 I/O 控制示例

控制夹爪执行 Group 1 Position 1：

```text
Send    : FF FE FD FC 01 10 04 01 00 00 00 00 00 FB
Receive : FF FE FD FC 01 10 04 01 00 00 00 00 00 FB
```

控制夹爪执行 Group 1 Position 2：

```text
Send    : FF FE FD FC 01 10 04 01 00 01 00 00 00 FB
Receive : FF FE FD FC 01 10 04 01 00 01 00 00 00 FB
```

控制夹爪执行 Group 2 Position 1：

```text
Send    : FF FE FD FC 01 10 08 01 00 00 00 00 00 FB
Receive : FF FE FD FC 01 10 08 01 00 00 00 00 00 FB
```

控制夹爪执行 Group 2 Position 2：

```text
Send    : FF FE FD FC 01 10 08 01 00 01 00 00 00 FB
Receive : FF FE FD FC 01 10 08 01 00 01 00 00 00 FB
```

---

## 6.6 CAN ID 设置命令

Function Register：

```text
0x12
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01` | `00/01` | Integer | 读取 / 设置夹爪 CAN ID |

默认 ID：

```text
1
```

可设置范围：

```text
1-255
```

小端表示：

```text
01 00 00 00 - FF 00 00 00
```

重要说明：

```text
设置 CAN ID 成功后，必须重启夹爪才能生效。
```

如果不知道或忘记当前夹爪 CAN ID，可以使用 ID `0` 来读取或设置夹爪 CAN ID。

---

### 6.6.1 当前 ID = 1，设置 CAN ID 为 2

完整帧：

```text
Send    : FF FE FD FC 01 12 01 01 00 02 00 00 00 FB
Receive : FF FE FD FC 01 12 01 01 00 02 00 00 00 FB
```

对应 CAN 数据段：

```text
12 01 01 00 02 00 00 00
```

---

### 6.6.2 不知道当前 ID 时，使用 ID = 0 设置 CAN ID 为 2

完整帧：

```text
Send    : FF FE FD FC 00 12 01 01 00 02 00 00 00 FB
Receive : FF FE FD FC 00 12 01 01 00 02 00 00 00 FB
```

通过 CAN2.0A 直接通信时，可理解为：

```text
CAN ID : 0x00
Data   : 12 01 01 00 02 00 00 00
```

---

### 6.6.3 当前 ID = 2，读取 CAN ID

完整帧：

```text
Send    : FF FE FD FC 02 12 01 00 00 00 00 00 00 FB
Receive : FF FE FD FC 02 12 01 00 00 02 00 00 00 FB
```

对应 CAN 数据段：

```text
12 01 00 00 00 00 00 00
```

返回 Data：

```text
02 00 00 00 = CAN ID 2
```

---

### 6.6.4 不知道当前 ID 时，使用 ID = 0 读取 CAN ID

完整帧：

```text
Send    : FF FE FD FC 00 12 01 01 00 00 00 00 00 FB
Receive : FF FE FD FC 00 12 01 01 00 02 00 00 00 FB
```

> 注意：手册示例中该读取命令的 `Read/Write` 字段写为 `01`，即写命令形式，但说明为使用 ID=0 读取 ID。实际程序中建议按设备手册示例实现，并结合实机返回验证。

---

## 6.7 固件版本读取命令

Function Register：

```text
0x13
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01` | `00` | `00 00 00 00` | 读取夹爪固件版本 |

完整帧：

```text
Send    : FF FE FD FC 01 13 01 00 00 00 00 00 00 FB
Receive : FF FE FD FC 01 13 01 00 00 00 02 01 04 FB
```

对应 CAN 数据段：

```text
13 01 00 00 00 00 00 00
```

返回示例中的 Data：

```text
00 02 01 04
```

---

## 6.8 CAN 波特率设置命令

Function Register：

```text
0x14
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01` | `00/01` | Integer | 读取 / 设置夹爪 CAN 波特率 |

重要说明：

```text
设置 CAN 波特率成功后，必须重启夹爪才能生效。
```

波特率索引范围：

```text
0-5
```

### 6.8.1 波特率索引表

| Index | Baud Rate |
|---:|---|
| `0` | `500 Kbps` |
| `1` | `400 Kbps` |
| `2` | `250 Kbps` |
| `3` | `200 Kbps` |
| `4` | `125 Kbps` |
| `5` | `100 Kbps` |

---

### 6.8.2 设置 CAN 波特率为 250 Kbps

250 Kbps 对应索引：

```text
2
```

完整帧：

```text
Send    : FF FE FD FC 01 14 01 01 00 02 00 00 00 FB
Receive : FF FE FD FC 01 14 01 01 00 02 00 00 00 FB
```

对应 CAN 数据段：

```text
14 01 01 00 02 00 00 00
```

---

### 6.8.3 读取当前 CAN 波特率

完整帧：

```text
Send    : FF FE FD FC 01 14 01 00 00 00 00 00 00 FB
Receive : FF FE FD FC 01 14 01 00 00 02 00 00 00 FB
```

对应 CAN 数据段：

```text
14 01 00 00 00 00 00 00
```

返回 Data：

```text
02 00 00 00 = Index 2 = 250 Kbps
```

---

## 6.9 物体掉落检测命令

Function Register：

```text
0x15
```

| Sub-Function | R/W | Data | 功能 |
|---|---|---|---|
| `0x01/0x02` | `00/01` | Integer | 物体掉落反馈相关命令 |

---

### 6.9.1 开启物体掉落反馈

完整帧：

```text
FF FE FD FC 01 15 01 01 00 01 00 00 00 FB
```

对应 CAN 数据段：

```text
15 01 01 00 01 00 00 00
```

---

### 6.9.2 关闭物体掉落反馈

完整帧：

```text
FF FE FD FC 01 15 01 01 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
15 01 01 00 00 00 00 00
```

---

### 6.9.3 物体掉落时自动反馈

当检测到夹持物体掉落时，夹爪自动发送：

```text
FF FE FD FC 01 15 02 00 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
15 02 00 00 00 00 00 00
```

---

### 6.9.4 停止掉落反馈

完整帧：

```text
FF FE FD FC 01 15 02 01 00 00 00 00 00 FB
```

对应 CAN 数据段：

```text
15 02 01 00 00 00 00 00
```

---

## 7. CAN2.0A 直接控制常用命令速查

以下假设：

```text
CAN ID = 0x01
DLC    = 8
```

| 功能 | CAN 数据段 |
|---|---|
| 执行初始化 | `08 02 01 00 00 00 00 00` |
| 读取初始化是否完成 | `08 02 00 00 00 00 00 00` |
| 设置闭合力为 30 | `05 02 01 00 1E 00 00 00` |
| 读取闭合力 | `05 02 00 00 00 00 00 00` |
| 设置位置为 60 | `06 02 01 00 3C 00 00 00` |
| 读取当前位置 | `06 02 00 00 00 00 00 00` |
| 读取当前状态 | `0F 01 00 00 00 00 00 00` |
| 启用 I/O 模式 | `10 09 01 00 01 00 00 00` |
| 读取 CAN ID | `12 01 00 00 00 00 00 00` |
| 读取固件版本 | `13 01 00 00 00 00 00 00` |
| 设置 CAN 波特率为 250 Kbps | `14 01 01 00 02 00 00 00` |
| 读取 CAN 波特率 | `14 01 00 00 00 00 00 00` |
| 开启物体掉落反馈 | `15 01 01 00 01 00 00 00` |
| 关闭物体掉落反馈 | `15 01 01 00 00 00 00 00` |

---

## 8. 编程实现建议

### 8.1 构造 CAN 数据段

CAN 数据段统一为：

```text
[FUNC, SUB_FUNC, RW, 0x00, DATA_L, DATA_H, DATA_2, DATA_3]
```

其中 `DATA` 为 32 位整数小端序。

例如，写位置 60：

```text
FUNC     = 0x06
SUB_FUNC = 0x02
RW       = 0x01
Reserve  = 0x00
Data     = 60 = 0x0000003C = 3C 00 00 00
```

CAN 数据段：

```text
06 02 01 00 3C 00 00 00
```

### 8.2 构造 USB / TCP / RS485 完整帧

完整帧统一为：

```text
FF FE FD FC | ID | FUNC | SUB_FUNC | RW | 00 | DATA_LE_4B | FB
```

例如，写位置 60，ID 为 1：

```text
FF FE FD FC 01 06 02 01 00 3C 00 00 00 FB
```

### 8.3 发送后读取反馈

多数普通写命令成功后，夹爪会返回相同数据。程序中可以用如下逻辑判断：

```text
发送命令
等待返回帧
如果返回帧与发送帧一致，则认为命令被成功接收
```

读取命令则需要解析返回帧中的 `Data` 字段。

### 8.4 控制流程建议

典型控制流程：

```text
1. 上电
2. 发送初始化命令
3. 周期读取初始化状态，直到初始化完成
4. 设置夹持力
5. 设置目标位置
6. 周期读取状态反馈
7. 根据状态判断是否到位或是否夹到物体
```

对应 CAN 命令示例：

```text
初始化：
08 02 01 00 00 00 00 00

读取初始化状态：
08 02 00 00 00 00 00 00

设置闭合力 30：
05 02 01 00 1E 00 00 00

设置位置 60：
06 02 01 00 3C 00 00 00

读取当前状态：
0F 01 00 00 00 00 00 00
```

### 8.5 注意事项

- 相邻命令发送间隔建议大于 `20 ms`。
- 设置 CAN ID 后必须重启夹爪。
- 设置 CAN 波特率后必须重启夹爪。
- 读命令中 Data 字段一般填 `00 00 00 00`。
- Data 字段为小端格式，不能按大端解析。
- 通过 CAN2.0A 直接通信时，不要添加 `FF FE FD FC` 帧头和 `FB` 帧尾。
- 通过 USB、TCP/IP、RS485 通信时，必须添加完整 14 字节协议帧。
- 状态反馈中 `02 00 00 00` 表示到位但未夹住，`03 00 00 00` 表示夹住但未到位。
- 掉落检测要求物体直径大于 `5 mm`。

