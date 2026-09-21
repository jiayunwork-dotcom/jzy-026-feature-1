# cam-follower — 凸轮从动件运动学小服务

给定运动规律与参数（升程 h、升程角 β、角速度 ω），沿凸轮转角给出从动件
**位移 s、速度 v、加速度 a、跃度 j**，并自动核对各段接头处各阶量的连续性；
还能把 **升程 → 远休止 → 回程 → 近休止** 拼成 360° 循环曲线。

- 语言：Go 1.22，仅经 HTTP 提供服务
- 角度单位全程钉死为 **度（°）**，ω 单位 **度/秒**，结果 JSON 中写明单位
- s/v/a/j 全部由同一套闭式解析式对转角逐次求导再乘 ω 的 0/1/2/3 次方得到，
  **不使用有限差分**

## 无因次规律与量纲还原

无因次时间 T = θ/β（升程段 0→1）。返回量纲：

| 量 | 公式 |
|---|---|
| s | h·y(T) |
| v | h·ω/β·y'(T) |
| a | h·ω²/β²·y''(T) |
| j | h·ω³/β³·y'''(T) |

三条规律（`type` 取值）：

- `cosine` 余弦（简谐）：y=(1−cos πT)/2 —— 两端 v=0，a≠0
- `cycloid` 摆线：y=T−sin(2πT)/(2π) —— 两端 v=0、a=0
- `parabolic` 等加速等减速：前半 y=2T²，T=0.5 必须切换到后半 y=1−2(1−T)²；
  两端 v=0 但 **a≠0**（±4hω²/β²，已知性质），中点 a 变号、s 与 v 连续

回程把升程规律对位移镜像：s=h−h·y(T)，v/a/j 同时取负，端点连续性与升程相同。
停歇段 s 保持 h（远休止）或 0（近休止），v/a/j 全为 0；停歇角为 0 时该段省略。

闭式峰值（摆线）：v_max=2hω/β，a_max=2π·hω²/β²，j_max=4π²·hω³/β³。

## 源码结构（按职责分文件）

```
main.go                      启动、载入初始配方
handlers.go                  HTTP 路由与请求/响应
internal/law/law.go          无因次规律闭式（含单侧取值与无因次峰值）
internal/kinematics/kinematics.go  乘 ω/β/h 还原量纲、升程曲线、单接头连续性核对
internal/cycle/cycle.go      四段循环拼接、整周网格、全部接头连续性检查
internal/store/store.go      配方/循环档校验与目录读写
internal/store/jsonfile.go   JSON 原子落盘
data/                        档文件（recipe_*.json / cycle_*.json）
```

## 运行

### 直接运行
```bash
go run . -addr :8080 -data data
```

### Docker（golang:1.22-alpine，单容器）
```bash
docker build -t cam-follower .
docker run --rm -p 8080:8080 -v $PWD/data:/data cam-follower
```

### 测试
```bash
go test ./... -count=1
```

启动时自动登记一条摆线升程配方 `cycloid-lift`（type=cycloid, h=10, β=60°）。

## HTTP 接口

### 配方（名字、类型、h、β）
- `GET  /recipes`                       列出配方（含类型与参数）
- `POST /recipes`                       登记/覆盖配方
- `GET  /recipes/{name}`                取一条配方

```json
POST /recipes
{"name":"harmonic-1","type":"cosine","h":5,"beta_deg":75}
```

### 循环档（四段角度与各段类型；ω 求曲线时再给）
- `GET  /cycles` / `POST /cycles` / `GET /cycles/{name}`

```json
POST /cycles
{"name":"week1","h":10,
 "rise_law":"cycloid","return_law":"cosine",
 "rise_angle_deg":60,"outer_dwell_deg":30,
 "return_angle_deg":90,"inner_dwell_deg":180}
```

### 升程曲线 s/v/a/j
- `GET  /lift?recipe=cycloid-lift&omega_deg_s=360&points=361`
- `POST /lift`（可用配方，或直接给 type/h/beta_deg 临时计算）

```json
POST /lift
{"type":"parabolic","h":10,"beta_deg":90,"omega_deg_s":180,"points":181}
```

响应含 `points[]`（每点 theta_deg,t_s,s,v,a,j）、`closed_form_peaks`（v/a/j 闭式峰值）、
`internal_seams`（如等加速中点的变号核对）。

### 整周循环曲线
- `GET  /cycle-curve?cycle=week1&omega_deg_s=360&points=361`
- `POST /cycle-curve`（可用循环档，或直接给四段参数临时计算）

响应含：
- `segments[]`：实际拼接出的段（停歇角 0 的段已省略）及段界角度
- `points[]`：0°..360° 等距网格
- `joints[]`：每个段界接头 / 等加速内部中点 / 360°↔0° 闭合点，逐量给出
  `s_continuous / v_continuous / a_continuous / j_continuous` 及左右闭式值
- `displacement_jump`：是否存在任何位移跳变（任何合法循环都应为 false）

## 拒绝规则（400，并在 error 中说明原因）

- 未知规律名（在求曲线前直接拒绝）
- h ≤ 0、ω ≤ 0、β ≤ 0 或 β ≥ 360°
- 四段角度之和不等于 360°、停歇角为负
- 缺项（POST 时既无 recipe/cycle 又未给全参数）
