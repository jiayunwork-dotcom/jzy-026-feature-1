# cam-follower — 凸轮从动件运动学小服务

给定运动规律与参数（升程 h、升程角 β、角速度 ω），沿凸轮转角给出从动件
**位移 s、速度 v、加速度 a、跃度 j**，并自动核对各段接头处各阶量的连续性；
还能把 **升程 → 远休止 → 回程 → 近休止** 拼成 360° 循环曲线。

在循环档之上还有一层**凸轮组**：登记"一根轴上装了哪几片凸轮、各自错开多少
相位角"，并回答——任意基准转角下每片落在自己循环的哪一段；每片在一整周
基准转角上的**在动窗口**（升程+回程）；以及约束校验（任意时刻同时在动片数
上限 / 两片在动窗口完全不重叠），冲突时给出转角区间与牵涉的凸轮。

- 语言：Go 1.22，仅经 HTTP 提供服务
- 角度单位全程钉死为 **度（°）**，ω 单位 **度/秒**，结果 JSON 中写明单位
- s/v/a/j 全部由同一套闭式解析式对转角逐次求导再乘 ω 的 0/1/2/3 次方得到，
  **不使用有限差分**；在动片数峰值由区间扫描线推出，**不做离散转角抽样**

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
handlers.go                  HTTP 路由与请求/响应（配方、循环档、曲线）
groups.go                    凸轮组/相位/约束的 HTTP 处理
internal/law/law.go          无因次规律闭式（含单侧取值与无因次峰值）
internal/kinematics/kinematics.go  乘 ω/β/h 还原量纲、升程曲线、单接头连续性核对
internal/cycle/cycle.go      四段循环拼接、整周网格、全部接头连续性检查
internal/phasing/phasing.go  相位规约、在动窗口、圆周区间合并、在动片数扫描线
internal/store/store.go      配方/循环档校验与目录读写
internal/store/groups.go     凸轮组校验与目录读写
internal/store/jsonfile.go   JSON 原子落盘
data/                        档文件（recipe_*.json / cycle_*.json / group_*.json）
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

### 凸轮组（一根轴上的多片凸轮 + 各自相位角）

每片凸轮点名一份**已登记**的循环档，并给一个相位角 `phase_deg`
（本片循环 0° 相对轴上公共基准转角的偏移）。相位角允许任意实数
（负数、超过一周），**登记时先规约到 [0,360)** 再落盘与计算。

- `GET  /groups` / `POST /groups` / `GET /groups/{name}`

```json
POST /groups
{"name":"shaft-1","cams":[
  {"id":"c1","cycle":"week1","phase_deg":0},
  {"id":"c2","cycle":"week1","phase_deg":-210}]}
```

- `GET /groups/{name}/state?theta_deg=170`
  轴转到某基准转角时，每片各自落在自己循环的哪一段
  （`segment`：rise / outer_dwell / return / inner_dwell，`active`：是否在动）。
  `theta_deg` 同样允许任意实数，先规约到一周内。

- `GET /groups/{name}/windows`（加 `?cam=c1` 只取某一片）
  每片在一整周基准转角上的在动窗口（升程+回程合并；停歇角为 0 时
  相邻运动段并成一条，跨 0°/360° 相接的也并成一条）。

- `POST /groups/{name}/check` 提交一条约束，返回满足或冲突的判断：

```json
{"type":"max_concurrent","cams":["c1","c2","c3"],"limit":2}
{"type":"no_overlap","cams":["c1","c2"]}
```

  `max_concurrent`：点名的片任意时刻同时在动片数 ≤ `limit`；
  `no_overlap`：点名的两片在动窗口完全不重叠（等价于这两片上限 1）。
  响应含 `satisfied`、`peak_count`（在动片数峰值）、`peak_windows`
  （峰值出现的转角区间）、`conflicts`（超限/重叠的转角区间，
  每段附牵涉的凸轮 `cams` 与该段峰值）。

**窗口与重叠的约定**（全服务自洽）：

- 窗口是半开区间 `[start,end)`：一片的终点恰等于另一片的起点
  **不算重叠**（交集长度必须为正）；同一片内部相接的升程/回程合并为
  一条窗口，因此单片永远不会"和自己重叠"。
- 窗口可横跨基准轴首尾，表示为 `start_deg > end_deg` 且 `wraps:true`
  （如 `{"start_deg":350,"end_deg":20,"span_deg":190,"wraps":true}`）；
  整周在动为 `start_deg=0, end_deg=360, span_deg=360`。
- 峰值与冲突区间从区间交叠本身推出（扫描线），不靠转角抽样计数，
  因此两两比较抓不到的三片同叠（上限 ≥2 时）也能给出正确峰值与区间。

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
- 凸轮组内点名了未登记的循环档、凸轮 id 为空或重复、相位角非有限实数
- 约束点名的凸轮不在组里、重复点名、未知约束类型、
  `no_overlap` 不是恰好两片、`max_concurrent` 上限为非正整数
  （非正上限意味着任何在动瞬间都违反、数学上必然冲突，按非法输入在校验前拒绝）

组不存在时查询/校验返回 404。
