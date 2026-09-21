// Package group 在一根轴的公共基准转角上管理一组凸轮：
// 每片凸轮点名一份已登记循环档，并附一个相位角（本片循环起点相对
// 基准转角的偏移）。本层负责把各片循环里的升程/回程段整体平移到
// 基准转角轴上（跨 0°/360° 回卷），整理每片的“在动”窗口，
// 并校验两类约束：任意时刻同时在动片数不超过上限、
// 点名的两片在动窗口完全不重叠。
//
// 约定（全层一致）：
//   - 角度单位为度，一周 360°；相位角与查询转角允许任意实数，
//     一律先规约到 [0, 360) 再计算（Normalize）。
//   - 窗口为半开区间 [start, end)：两端点恰好相接不算重叠。
//     这条取舍保证单片凸轮内部相邻两段（如升程紧接远休止、
//     或停歇角为 0 时升程紧接回程）不会被误判成与自己重叠。
//   - 回卷窗口保持单段表示：End < Start 表示跨过 0°/360°。
//   - 在动片数峰值由区间扫描线推出，不做离散抽样。
package group

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"strings"

	"camfollower/internal/cycle"
	"camfollower/internal/kinematics"
)

// eps 是角度比较的全局容差，与 cycle 包校验一周角度和时用的量级一致。
const eps = 1e-9

// Cam 是组里的一片凸轮：点名一份已登记循环档，附一个相位角（度）。
type Cam struct {
	Name  string  `json:"name"`
	Cycle string  `json:"cycle"`
	Phase float64 `json:"phase_deg"`
}

// Group 是同一根轴上的一组凸轮。
type Group struct {
	Name string `json:"name"`
	Cams []Cam  `json:"cams"`
}

// Validate 校验组的结构合法性（循环档是否已登记由调用方核对）。
func (g Group) Validate() error {
	if strings.TrimSpace(g.Name) == "" {
		return errors.New("凸轮组名不能为空")
	}
	if len(g.Cams) == 0 {
		return errors.New("凸轮组至少要有一片凸轮")
	}
	seen := make(map[string]bool, len(g.Cams))
	for _, c := range g.Cams {
		if strings.TrimSpace(c.Name) == "" {
			return errors.New("组内凸轮名不能为空")
		}
		if seen[c.Name] {
			return fmt.Errorf("凸轮名 %q 在组内重复", c.Name)
		}
		seen[c.Name] = true
		if strings.TrimSpace(c.Cycle) == "" {
			return fmt.Errorf("凸轮 %q 未点名循环档", c.Name)
		}
		if math.IsNaN(c.Phase) || math.IsInf(c.Phase, 0) {
			return fmt.Errorf("凸轮 %q 的相位角必须是有限实数", c.Name)
		}
	}
	return nil
}

// Has 报告组里是否有名为 name 的凸轮。
func (g Group) Has(name string) bool {
	for _, c := range g.Cams {
		if c.Name == name {
			return true
		}
	}
	return false
}

// Normalize 把任意角度（含负数、超过一周的值）规约到 [0, 360)。
func Normalize(a float64) float64 {
	a = math.Mod(a, kinematics.FullTurn)
	if a < 0 {
		a += kinematics.FullTurn
	}
	if a >= kinematics.FullTurn {
		a = 0 // 浮点把 360-ε 舍入成一整周时归 0
	}
	if a == 0 {
		return 0 // -0 归一成 +0
	}
	return a
}

// Window 是基准转角轴上一段半开区间 [Start, End)（度）。
// End < Start 表示区间跨过 0°/360° 回卷（如 350°→20°）；
// Start==0 且 End==360 表示整周都在动。两端点相接的窗口不算重叠。
type Window struct {
	Start float64 `json:"start_deg"`
	End   float64 `json:"end_deg"`
}

// angleEq 比较两个规约后的角度是否在容差内重合（含 0°/360° 两侧）。
func angleEq(a, b float64) bool {
	d := math.Abs(a - b)
	return d < eps || d > kinematics.FullTurn-eps
}

// arcWindow 把（起点，弧长）折成窗口表示；终点恰落在 360°/0° 时写成 360，
// 避免出现 {300, 0} 这类易被误读的回卷写法。
func arcWindow(start, length float64) Window {
	s := Normalize(start)
	e := Normalize(s + length)
	if e == 0 {
		e = kinematics.FullTurn
	}
	return Window{Start: s, End: e}
}

// MovingWindows 把一片凸轮循环里的升程、回程段按相位角平移到基准转角轴，
// 整理成这一片的“在动”窗口（升程∪回程，按起点排序）：
//   - 远休止角为 0 时升程尾接回程头、近休止角为 0 时回程尾接升程头，
//     相接的在动段合并为一段，不因省略的停歇段断开或多算；
//   - 跨过一周边界的窗口保持单段回卷表示（End < Start）；
//   - 两个停歇角都为 0 时整周在动，返回单个 {0, 360} 窗口。
func MovingWindows(cfg cycle.Config, phase float64) ([]Window, error) {
	segs, err := cycle.Build(cfg) // 内含四段角度校验
	if err != nil {
		return nil, err
	}
	ph := Normalize(phase)
	type arc struct{ start, length float64 }
	arcs := make([]arc, 0, 2)
	total := 0.0
	for _, sg := range segs.All {
		if sg.Kind != "rise" && sg.Kind != "return" {
			continue
		}
		arcs = append(arcs, arc{Normalize(sg.Start + ph), sg.End - sg.Start})
		total += sg.End - sg.Start
	}
	if total >= kinematics.FullTurn-eps {
		return []Window{{Start: 0, End: kinematics.FullTurn}}, nil
	}
	if len(arcs) == 2 {
		a, b := arcs[0], arcs[1] // 本片循环里升程必先于回程
		switch {
		case angleEq(Normalize(a.start+a.length), b.start):
			// 远休止为 0：升程尾接回程头
			arcs = []arc{{a.start, a.length + b.length}}
		case angleEq(Normalize(b.start+b.length), a.start):
			// 近休止为 0：回程尾接升程头（接点跨 0°/360°）
			arcs = []arc{{b.start, b.length + a.length}}
		}
	}
	ws := make([]Window, 0, len(arcs))
	for _, a := range arcs {
		ws = append(ws, arcWindow(a.start, a.length))
	}
	sort.Slice(ws, func(i, j int) bool { return ws[i].Start < ws[j].Start })
	return ws, nil
}

// SegmentHit 是一片凸轮在某个基准转角下所处的段。
type SegmentHit struct {
	Cam        string  `json:"cam"`
	Cycle      string  `json:"cycle"`
	PhaseDeg   float64 `json:"phase_deg"`        // 规约到一周以内的相位角
	LocalAngle float64 `json:"local_angle_deg"`  // 基准转角换算到本片循环内的角度
	Segment    string  `json:"segment"`          // rise / outer_dwell / return / inner_dwell
	SegStart   float64 `json:"segment_start_deg"` // 段在本片循环内的起止转角
	SegEnd     float64 `json:"segment_end_deg"`
	Moving     bool    `json:"moving"` // 升程/回程为在动，远休/近休为闲着
}

// SegmentAt 计算一片凸轮在基准转角 refAngle 下落在本片循环的哪一段。
// 段界归属约定与单循环曲线一致：段界点有一侧是停歇段时归停歇段，
// 运动段直接相接（停歇角为 0）时归从该点开始的段。
func SegmentAt(cam Cam, cfg cycle.Config, refAngle float64) (SegmentHit, error) {
	segs, err := cycle.Build(cfg)
	if err != nil {
		return SegmentHit{}, err
	}
	ph := Normalize(cam.Phase)
	local := Normalize(Normalize(refAngle) - ph)
	sg := segs.SegmentAt(local)
	return SegmentHit{
		Cam: cam.Name, Cycle: cam.Cycle, PhaseDeg: ph,
		LocalAngle: local,
		Segment:    sg.Kind, SegStart: sg.Start, SegEnd: sg.End,
		Moving: sg.Kind == "rise" || sg.Kind == "return",
	}, nil
}

// ---------- 区间扫描 ----------

// piece 是切轴 [0, 360] 上不跨零的半开区间（回卷窗口拆成两段）。
type piece struct {
	start, end float64
	cam        string
}

func splitWindow(cam string, w Window) []piece {
	switch {
	case w.End > w.Start:
		return []piece{{w.Start, w.End, cam}}
	case w.End < w.Start: // 回卷：跨 0°/360°
		return []piece{{w.Start, kinematics.FullTurn, cam}, {0, w.End, cam}}
	case w.Start == 0: // {0,0} 防御性视为整周；正常窗口不变量下不会出现
		return []piece{{0, kinematics.FullTurn, cam}}
	}
	return nil
}

func sameCams(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// Region 是基准转角轴上一段在动凸轮集合恒定的区间。
type Region struct {
	Window
	Cams []string `json:"cams"` // 该区间内同时在动的凸轮（按名排序）
}

// Count 是该区间内同时在动的片数。
func (r Region) Count() int { return len(r.Cams) }

// Sweep 沿基准转角轴对所有在动窗口做扫描线求交，返回在动集合恒定的
// 各个区间（相邻且集合相同的已合并，跨 0°/360° 的已接回，按起点排序）。
// 峰值即各区间 Count 的最大值——从区间交叠推出，不做离散抽样。
func Sweep(perCam map[string][]Window) []Region {
	var pieces []piece
	var pts []float64
	for cam, ws := range perCam {
		for _, w := range ws {
			for _, p := range splitWindow(cam, w) {
				pieces = append(pieces, p)
				pts = append(pts, p.start, p.end)
			}
		}
	}
	if len(pieces) == 0 {
		return nil
	}
	sort.Float64s(pts)
	uniq := pts[:1]
	for _, p := range pts[1:] {
		if p-uniq[len(uniq)-1] > eps {
			uniq = append(uniq, p)
		}
	}
	var regions []Region
	for i := 0; i+1 < len(uniq); i++ {
		a, b := uniq[i], uniq[i+1]
		if b-a <= eps {
			continue
		}
		mid := (a + b) / 2 // 严格落在相邻端点之间，无边界归属歧义
		var cams []string
		for _, p := range pieces {
			if p.start <= mid && mid < p.end {
				cams = append(cams, p.cam)
			}
		}
		if len(cams) == 0 {
			continue
		}
		sort.Strings(cams)
		regions = append(regions, Region{Window: Window{Start: a, End: b}, Cams: cams})
	}
	// 相邻且集合相同的区间合并（中间只隔着被跳过的 eps 细缝时也合并）。
	merged := regions[:0]
	for _, r := range regions {
		if n := len(merged); n > 0 && sameCams(merged[n-1].Cams, r.Cams) && r.Start-merged[n-1].End <= eps {
			merged[n-1].End = r.End
		} else {
			merged = append(merged, r)
		}
	}
	// 首尾都贴着轴端且集合相同：接回成一段跨零回卷区间。
	if len(merged) >= 2 {
		first, last := merged[0], merged[len(merged)-1]
		if first.Start <= eps && last.End >= kinematics.FullTurn-eps && sameCams(first.Cams, last.Cams) {
			merged = append([]Region{{Window: Window{Start: last.Start, End: first.End}, Cams: last.Cams}},
				merged[1:len(merged)-1]...)
		}
	}
	sort.Slice(merged, func(i, j int) bool { return merged[i].Start < merged[j].Start })
	return merged
}

// OverlapWindows 求两片凸轮在动窗口之间的正长度交集（度）。
// 端点恰好相接不算重叠；跨零的交集接回成单段回卷窗口。
func OverlapWindows(a, b []Window) []Window {
	var inter []Window
	for _, wa := range a {
		for _, pa := range splitWindow("", wa) {
			for _, wb := range b {
				for _, pb := range splitWindow("", wb) {
					s := math.Max(pa.start, pb.start)
					e := math.Min(pa.end, pb.end)
					if e-s > eps {
						inter = append(inter, Window{Start: s, End: e})
					}
				}
			}
		}
	}
	return mergeWindows(inter)
}

// mergeWindows 合并切轴上相互重叠或端点相接（容差内）的窗口，并做跨零接回。
func mergeWindows(ws []Window) []Window {
	if len(ws) == 0 {
		return nil
	}
	sort.Slice(ws, func(i, j int) bool { return ws[i].Start < ws[j].Start })
	out := []Window{ws[0]}
	for _, w := range ws[1:] {
		last := &out[len(out)-1]
		if w.Start <= last.End+eps {
			if w.End > last.End {
				last.End = w.End
			}
		} else {
			out = append(out, w)
		}
	}
	if len(out) >= 2 {
		first, last := out[0], out[len(out)-1]
		if first.Start <= eps && last.End >= kinematics.FullTurn-eps {
			out = append([]Window{{Start: last.Start, End: first.End}}, out[1:len(out)-1]...)
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Start < out[j].Start })
	return out
}

// ---------- 约束校验 ----------

// 约束类型。
const (
	MaxSimultaneous = "max_simultaneous" // 任意时刻同时在动片数不超过 limit
	NonOverlap      = "non_overlap"      // 点名的两片在动窗口完全不重叠
)

// Constraint 是提交校验的一条约束。
type Constraint struct {
	Type  string   `json:"type"`            // max_simultaneous / non_overlap
	Limit int      `json:"limit,omitempty"` // max_simultaneous：同时在动片数上限（正整数）
	Cams  []string `json:"cams,omitempty"`  // non_overlap：点名的两片凸轮
}

// Validate 在做任何区间计算之前校验约束本身及其与组的关系。
func (c Constraint) Validate(g Group) error {
	switch c.Type {
	case MaxSimultaneous:
		if c.Limit <= 0 {
			return fmt.Errorf("同时在动片数上限必须为正整数，当前为 %d", c.Limit)
		}
	case NonOverlap:
		if len(c.Cams) != 2 {
			return fmt.Errorf("non_overlap 必须恰好点名两片凸轮，当前为 %d 片", len(c.Cams))
		}
		if c.Cams[0] == c.Cams[1] {
			return fmt.Errorf("non_overlap 点名的两片凸轮必须不同（%q 出现了两次，单片不会与自己重叠）", c.Cams[0])
		}
		for _, n := range c.Cams {
			if !g.Has(n) {
				return fmt.Errorf("约束点名的凸轮 %q 不在组 %q 里", n, g.Name)
			}
		}
	default:
		return fmt.Errorf("未知约束类型 %q，支持: %s（同时在动片数上限）、%s（两片在动窗口不重叠）",
			c.Type, MaxSimultaneous, NonOverlap)
	}
	return nil
}

// Conflict 是一段约束冲突：冲突所在的基准转角区间与牵涉的凸轮。
type Conflict struct {
	Window
	Cams         []string `json:"cams"`
	Simultaneous int      `json:"simultaneous,omitempty"` // max_simultaneous：该区间内在动片数
}

// CheckResult 是约束校验结论。
type CheckResult struct {
	Satisfied   bool       `json:"satisfied"`
	Peak        int        `json:"peak_simultaneous,omitempty"` // max_simultaneous：在动片数峰值
	PeakWindows []Window   `json:"peak_windows,omitempty"`      // 峰值出现的基准转角区间
	Conflicts   []Conflict `json:"conflicts"`
}

// Check 在当前相位配置（perCam 为各片在动窗口，键为凸轮名）下校验约束。
// 约束须先经 Validate 通过。
func Check(c Constraint, perCam map[string][]Window) CheckResult {
	res := CheckResult{Conflicts: []Conflict{}}
	switch c.Type {
	case MaxSimultaneous:
		regions := Sweep(perCam)
		for _, r := range regions {
			if r.Count() > res.Peak {
				res.Peak = r.Count()
			}
		}
		for _, r := range regions {
			if r.Count() == res.Peak && res.Peak > 0 {
				res.PeakWindows = append(res.PeakWindows, r.Window)
			}
			if r.Count() > c.Limit {
				res.Conflicts = append(res.Conflicts, Conflict{
					Window: r.Window, Cams: r.Cams, Simultaneous: r.Count(),
				})
			}
		}
		res.Satisfied = res.Peak <= c.Limit
	case NonOverlap:
		for _, w := range OverlapWindows(perCam[c.Cams[0]], perCam[c.Cams[1]]) {
			res.Conflicts = append(res.Conflicts, Conflict{
				Window: w, Cams: []string{c.Cams[0], c.Cams[1]},
			})
		}
		res.Satisfied = len(res.Conflicts) == 0
	}
	return res
}
