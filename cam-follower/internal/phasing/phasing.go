// Package phasing 管理同轴多片凸轮的相位关系：
// 把各片循环档的四段角度按各自相位角平移到公共基准转角轴上，
// 计算每片的“在动”窗口（升程+回程）、窗口间重叠，
// 以及任意时刻同时在动片数的峰值（扫描线，不做离散抽样）。
//
// 约定（全包自洽）：
//   - 角度单位为度，一周 360°；相位角与基准转角允许任意实数，
//     一律先规约到 [0,360) 再参与计算。
//   - 窗口是半开区间 [start,end)：一片窗口的终点恰等于另一片的起点
//     不算重叠（交集长度必须为正值才算重叠）；同一片内部相接的
//     升程/回程（停歇角为 0 时）合并为一条窗口，不会与自己重叠。
//   - 窗口可横跨基准轴首尾（如 350°→20°），内部以 Start+Span 表示，
//     对外 JSON 视图以 start_deg>end_deg 且 wraps=true 表示。
package phasing

import (
	"math"
	"sort"
)

// FullTurn 一周，单位度。
const FullTurn = 360.0

// touchEps 判定“相接/同位置”的容差（度）：端点间距在此范围内视为同一位置。
const touchEps = 1e-9

// Norm360 把任意实数角度规约到 [0,360)；负值与超过一周的值都折回一周内。
func Norm360(x float64) float64 {
	x = math.Mod(x, FullTurn)
	if x < 0 {
		x += FullTurn
	}
	if x == 0 {
		return 0 // 归一 -0
	}
	return x
}

// Cam 是一片凸轮在轴上的登记：循环四段角度 + 相位角。
// 相位角为本片循环 0° 相对基准转角的偏移，允许任意实数（内部规约）。
type Cam struct {
	ID          string
	Phase       float64
	RiseAngle   float64
	OuterDwell  float64
	ReturnAngle float64
	InnerDwell  float64
}

// Arc 是圆周上一条有向窗口：从 Start 沿转角增大方向跨 Span 度。
// Start ∈ [0,360)，Span ∈ (0,360]；Start+Span > 360 时窗口横跨 0°/360° 回卷。
type Arc struct {
	Start float64
	Span  float64
}

// ArcView 是 Arc 的对外 JSON 视图：回卷窗口 end_deg < start_deg 且 wraps=true；
// 整周窗口为 start_deg=0、end_deg=360、span_deg=360。
type ArcView struct {
	StartDeg float64 `json:"start_deg"`
	EndDeg   float64 `json:"end_deg"`
	SpanDeg  float64 `json:"span_deg"`
	Wraps    bool    `json:"wraps"`
}

// View 把内部弧转成对外视图。
func (a Arc) View() ArcView {
	end := a.Start + a.Span
	wraps := false
	if end > FullTurn {
		end -= FullTurn
		wraps = true
	}
	return ArcView{StartDeg: a.Start, EndDeg: end, SpanDeg: a.Span, Wraps: wraps}
}

// interval 是 [0,360] 内的半开区间 [s,e)。
type interval struct{ s, e float64 }

// localActive 返回本片循环内的在动区间（未平移）：升程 [0,β1) 与回程段。
func (c Cam) localActive() [][2]float64 {
	retStart := c.RiseAngle + c.OuterDwell
	return [][2]float64{
		{0, c.RiseAngle},
		{retStart, retStart + c.ReturnAngle},
	}
}

// intervals 返回平移相位并回卷到 [0,360) 的在动半开区间（未合并）。
func (c Cam) intervals() []interval {
	phi := Norm360(c.Phase)
	var out []interval
	for _, lr := range c.localActive() {
		a, b := lr[0]+phi, lr[1]+phi
		if a >= FullTurn {
			a, b = a-FullTurn, b-FullTurn
		}
		if b <= FullTurn {
			out = append(out, interval{a, b})
		} else {
			// 平移后跨过 360°：拆成轴尾一段与轴首一段
			out = append(out, interval{a, FullTurn}, interval{0, b - FullTurn})
		}
	}
	return out
}

// mergeCircular 把 [0,360] 内的半开区间在圆周上合并：
// 重叠或相接（端点间距 ≤ touchEps）的区间并成一条弧，
// 跨过 0°/360° 相接的也算同一条弧。
func mergeCircular(ivs []interval) []Arc {
	if len(ivs) == 0 {
		return nil
	}
	sorted := append([]interval(nil), ivs...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i].s < sorted[j].s })
	merged := []interval{sorted[0]}
	for _, iv := range sorted[1:] {
		last := &merged[len(merged)-1]
		if iv.s <= last.e+touchEps {
			if iv.e > last.e {
				last.e = iv.e
			}
		} else {
			merged = append(merged, iv)
		}
	}
	// 跨零合并：末段抵到 360°、首段从 0° 开始 → 圆周上同一条弧
	if len(merged) > 1 {
		first, last := merged[0], merged[len(merged)-1]
		if first.s <= touchEps && last.e >= FullTurn-touchEps {
			wrap := interval{last.s, first.e + FullTurn}
			merged = append([]interval{wrap}, merged[1:len(merged)-1]...)
		}
	}
	arcs := make([]Arc, 0, len(merged))
	for _, m := range merged {
		if m.e-m.s >= FullTurn-touchEps {
			arcs = append(arcs, Arc{Start: 0, Span: FullTurn})
			continue
		}
		arcs = append(arcs, Arc{Start: Norm360(m.s), Span: m.e - m.s})
	}
	sort.Slice(arcs, func(i, j int) bool { return arcs[i].Start < arcs[j].Start })
	return arcs
}

// Windows 返回该片在基准转角轴上的在动窗口：
// 升程与回程的并集，相接（停歇角为 0）即合并，可回卷。
func (c Cam) Windows() []Arc {
	return mergeCircular(c.intervals())
}

// intersect 两个半开区间的交集；仅端点相触（长度 ≤ touchEps）不算重叠。
func intersect(a, b interval) (interval, bool) {
	s, e := math.Max(a.s, b.s), math.Min(a.e, b.e)
	if e-s <= touchEps {
		return interval{}, false
	}
	return interval{s, e}, true
}

// Overlap 返回两片在动窗口的交集弧（空表示完全不重叠）。
func Overlap(a, b Cam) []Arc {
	var ivs []interval
	for _, x := range a.intervals() {
		for _, y := range b.intervals() {
			if iv, ok := intersect(x, y); ok {
				ivs = append(ivs, iv)
			}
		}
	}
	return mergeCircular(ivs)
}

// seg 是扫描线上相邻事件点之间的基本段：其上在动片数与在动集合恒定。
type seg struct {
	s, e  float64
	count int
	cams  []string
}

// sweepSegs 把各片在动区间打成事件点扫描，输出铺满 [0,360] 的基本段。
// 半开语义：事件点处先离场的片不计入该点右侧段，新进场的计入。
func sweepSegs(cams []Cam) []seg {
	type ev struct {
		pos   float64
		id    string
		delta int
	}
	var evs []ev
	for _, c := range cams {
		for _, iv := range c.intervals() {
			evs = append(evs, ev{iv.s, c.ID, +1}, ev{iv.e, c.ID, -1})
		}
	}
	sort.Slice(evs, func(i, j int) bool {
		if evs[i].pos != evs[j].pos {
			return evs[i].pos < evs[j].pos
		}
		if evs[i].id != evs[j].id {
			return evs[i].id < evs[j].id
		}
		return evs[i].delta < evs[j].delta
	})
	active := map[string]bool{}
	var segs []seg
	pos := 0.0
	i := 0
	for i < len(evs) {
		p := evs[i].pos
		if p > pos {
			segs = append(segs, seg{s: pos, e: p, count: len(active), cams: sortedKeys(active)})
		}
		for i < len(evs) && evs[i].pos == p {
			if evs[i].delta > 0 {
				active[evs[i].id] = true
			} else {
				delete(active, evs[i].id)
			}
			i++
		}
		pos = p
	}
	if pos < FullTurn {
		segs = append(segs, seg{s: pos, e: FullTurn, count: len(active), cams: sortedKeys(active)})
	}
	return segs
}

func sortedKeys(m map[string]bool) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

// Region 是一段基准转角区间及其上牵涉的凸轮。
type Region struct {
	Arc  Arc
	Cams []string // 区间内同时在动的凸轮 id（排序）
	Peak int      // 区间内在动片数峰值
}

// RegionView 是 Region 的对外 JSON 视图。
type RegionView struct {
	ArcView
	Cams []string `json:"cams"`
	Peak int      `json:"peak_count"`
}

// View 把内部区域转成对外视图。
func (r Region) View() RegionView {
	return RegionView{ArcView: r.Arc.View(), Cams: r.Cams, Peak: r.Peak}
}

// regionFromSegs 把一段连续基本段聚成 Region：Cams 取并集，Peak 取最大。
func regionFromSegs(run []seg, arc Arc) Region {
	set := map[string]bool{}
	peak := 0
	for _, s := range run {
		if s.count > peak {
			peak = s.count
		}
		for _, id := range s.cams {
			set[id] = true
		}
	}
	cams := make([]string, 0, len(set))
	for id := range set {
		cams = append(cams, id)
	}
	sort.Strings(cams)
	return Region{Arc: arc, Cams: cams, Peak: peak}
}

// mergeSegs 把满足 pick 的相邻基本段并成 Region；
// 圆周上跨 0°/360° 相接的也算相邻（合成一条回卷弧）。
func mergeSegs(segs []seg, pick func(seg) bool) []Region {
	n := len(segs)
	if n == 0 {
		return nil
	}
	picked := make([]bool, n)
	nPick := 0
	for i, s := range segs {
		picked[i] = pick(s)
		if picked[i] {
			nPick++
		}
	}
	if nPick == 0 {
		return nil
	}
	if nPick == n {
		return []Region{regionFromSegs(segs, Arc{Start: 0, Span: FullTurn})}
	}
	// 找一个未选中的段作为圆周断开点，旋转后线性合并，天然处理跨零。
	cut := 0
	for picked[cut] {
		cut++
	}
	rot := make([]seg, 0, n)
	rot = append(rot, segs[cut+1:]...)
	rot = append(rot, segs[:cut+1]...)
	var out []Region
	for i := 0; i < n; {
		if !pick(rot[i]) {
			i++
			continue
		}
		j := i
		for j+1 < n && pick(rot[j+1]) {
			j++
		}
		run := rot[i : j+1]
		start := run[0].s
		span := run[len(run)-1].e - start
		if span <= 0 { // 跨过断开点（原 0°/360° 处）
			span += FullTurn
		}
		out = append(out, regionFromSegs(run, Arc{Start: start, Span: span}))
		i = j + 1
	}
	return out
}

// SweepResult 是一组凸轮整周在动情况的统计结果。
type SweepResult struct {
	Peak        int      // 任意时刻在动片数峰值
	PeakRegions []Region // 峰值出现的转角区间
	Exceed      []Region // 在动片数超过 limit 的转角区间（空即满足约束）
}

// Sweep 从区间交叠出发推出任意时刻在动片数峰值及其位置，
// 并给出超过 limit 的全部转角区间；不做离散转角抽样。
func Sweep(cams []Cam, limit int) SweepResult {
	segs := sweepSegs(cams)
	res := SweepResult{}
	for _, sg := range segs {
		if sg.count > res.Peak {
			res.Peak = sg.count
		}
	}
	res.PeakRegions = mergeSegs(segs, func(s seg) bool { return s.count == res.Peak && s.count > 0 })
	res.Exceed = mergeSegs(segs, func(s seg) bool { return s.count > limit })
	return res
}
