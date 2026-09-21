package phasing

import (
	"math"
	"testing"
)

func cam(id string, phase, rise, outer, ret, inner float64) Cam {
	return Cam{ID: id, Phase: phase, RiseAngle: rise, OuterDwell: outer, ReturnAngle: ret, InnerDwell: inner}
}

func arcEq(t *testing.T, got Arc, start, span float64) {
	t.Helper()
	if math.Abs(got.Start-start) > 1e-9 || math.Abs(got.Span-span) > 1e-9 {
		t.Fatalf("弧 = {%g,%g}，期望 {%g,%g}", got.Start, got.Span, start, span)
	}
}

// 相位角规约：负值、超过一周、整周倍数都折回 [0,360)。
func TestNorm360(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{0, 0}, {90, 90}, {359.5, 359.5},
		{360, 0}, {720, 0}, {450, 90},
		{-90, 270}, {-360, 0}, {-450, 270},
	}
	for _, c := range cases {
		if got := Norm360(c.in); got != c.want {
			t.Fatalf("Norm360(%g) = %g，期望 %g", c.in, got, c.want)
		}
	}
}

// 常规四段：升程、回程各成一条窗口。
func TestWindowsBasic(t *testing.T) {
	c := cam("a", 0, 60, 30, 90, 180)
	w := c.Windows()
	if len(w) != 2 {
		t.Fatalf("应有 2 条窗口，得到 %d: %+v", len(w), w)
	}
	arcEq(t, w[0], 0, 60)
	arcEq(t, w[1], 90, 90)
}

// 相位平移跨过 360°：窗口回卷到轴首，start>end 且 wraps=true。
func TestWindowsShiftWrap(t *testing.T) {
	c := cam("a", 350, 60, 30, 90, 180)
	w := c.Windows()
	if len(w) != 2 {
		t.Fatalf("应有 2 条窗口，得到 %d: %+v", len(w), w)
	}
	arcEq(t, w[0], 80, 90)  // 回程段 [90,180)+350 = [80,170)
	arcEq(t, w[1], 350, 60) // 升程段 [0,60)+350 = [350,410) 回卷
	v := w[1].View()
	if !v.Wraps || v.EndDeg != 50 || v.StartDeg != 350 || v.SpanDeg != 60 {
		t.Fatalf("回卷窗口视图错误: %+v", v)
	}
}

// 远休止角为 0：升程与回程相接，合并为一条窗口，不得断开。
func TestWindowsZeroOuterDwellMerges(t *testing.T) {
	c := cam("a", 0, 45, 0, 75, 240)
	w := c.Windows()
	if len(w) != 1 {
		t.Fatalf("相接运动段应合并为 1 条窗口，得到 %d: %+v", len(w), w)
	}
	arcEq(t, w[0], 0, 120)
}

// 近休止角为 0：回程终点接 360°/0° 处的升程起点，跨零合并为一条回卷窗口。
func TestWindowsZeroInnerDwellMergesAcrossZero(t *testing.T) {
	c := cam("a", 0, 45, 30, 285, 0)
	w := c.Windows()
	if len(w) != 1 {
		t.Fatalf("跨零相接应合并为 1 条窗口，得到 %d: %+v", len(w), w)
	}
	arcEq(t, w[0], 75, 330) // [75,360) ∪ [0,45)
	v := w[0].View()
	if !v.Wraps || v.EndDeg != 45 {
		t.Fatalf("回卷视图错误: %+v", v)
	}
}

// 两个停歇角都为 0：整周在动。
func TestWindowsFullTurnActive(t *testing.T) {
	c := cam("a", 0, 100, 0, 260, 0)
	w := c.Windows()
	if len(w) != 1 {
		t.Fatalf("应为 1 条整周窗口，得到 %d: %+v", len(w), w)
	}
	arcEq(t, w[0], 0, 360)
	if v := w[0].View(); v.Wraps || v.EndDeg != 360 {
		t.Fatalf("整周窗口视图错误: %+v", v)
	}
}

// 边界取舍：一片终点恰等于另一片起点，不算重叠（半开区间）。
func TestOverlapTouchingIsNotOverlap(t *testing.T) {
	a := cam("a", 0, 100, 0, 50, 210)   // 在动 [0,150)
	b := cam("b", 150, 100, 0, 50, 210) // 在动 [150,300)
	if ov := Overlap(a, b); len(ov) != 0 {
		t.Fatalf("端点相触不应算重叠: %+v", ov)
	}
	res := Sweep([]Cam{a, b}, 1)
	if res.Peak != 1 || len(res.Exceed) != 0 {
		t.Fatalf("上限 1 应满足，峰值 %d 超限区间 %+v", res.Peak, res.Exceed)
	}
	// 峰值区间把相接段合并：[0,300)
	if len(res.PeakRegions) != 1 {
		t.Fatalf("峰值区间应合并为 1 段: %+v", res.PeakRegions)
	}
	arcEq(t, res.PeakRegions[0].Arc, 0, 300)
}

// 部分交叠：交集区间必须精确。
func TestOverlapPartial(t *testing.T) {
	a := cam("a", 0, 60, 30, 90, 180)   // [0,60) [90,180)
	b := cam("b", 150, 60, 30, 90, 180) // [150,210) [240,330)
	ov := Overlap(a, b)
	if len(ov) != 1 {
		t.Fatalf("应有 1 段交集，得到 %d: %+v", len(ov), ov)
	}
	arcEq(t, ov[0], 150, 30)
}

// 三片两两交叠都不超上限 2，但某一瞬间三片叠在一起：
// 两两比较不够，必须由扫描线给出峰值 3 及其区间。
func TestSweepTripleOverlap(t *testing.T) {
	mk := func(id string, phase float64) Cam { return cam(id, phase, 100, 0, 50, 210) }
	a, b, c := mk("a", 0), mk("b", 60), mk("c", 120) // [0,150) [60,210) [120,270)
	res := Sweep([]Cam{a, b, c}, 2)
	if res.Peak != 3 {
		t.Fatalf("峰值应为 3，得到 %d", res.Peak)
	}
	if len(res.Exceed) != 1 {
		t.Fatalf("超限区间应为 1 段: %+v", res.Exceed)
	}
	arcEq(t, res.Exceed[0].Arc, 120, 30)
	if res.Exceed[0].Peak != 3 || len(res.Exceed[0].Cams) != 3 {
		t.Fatalf("超限区间牵涉凸轮错误: %+v", res.Exceed[0])
	}
}

// 三片各占三分之一周期错开（在动块连续 120°，相位各错 120°）：
// 任意时刻恰好一片在动，上限 1 满足。
func TestSweepTilingPeakOne(t *testing.T) {
	mk := func(id string, phase float64) Cam { return cam(id, phase, 60, 0, 60, 240) }
	res := Sweep([]Cam{mk("a", 0), mk("b", 120), mk("c", 240)}, 1)
	if res.Peak != 1 || len(res.Exceed) != 0 {
		t.Fatalf("峰值应为 1 且无超限，峰值 %d 超限 %+v", res.Peak, res.Exceed)
	}
	// 峰值 1 铺满整周
	if len(res.PeakRegions) != 1 {
		t.Fatalf("峰值区间应为整周 1 段: %+v", res.PeakRegions)
	}
	arcEq(t, res.PeakRegions[0].Arc, 0, 360)
	if len(res.PeakRegions[0].Cams) != 3 {
		t.Fatalf("整周峰值区间牵涉全部 3 片: %+v", res.PeakRegions[0].Cams)
	}
}

// 单片凸轮：峰值 1，上限 1 满足，不会判成和自己重叠。
func TestSweepSingleCam(t *testing.T) {
	a := cam("a", 0, 100, 0, 50, 210)
	res := Sweep([]Cam{a}, 1)
	if res.Peak != 1 || len(res.Exceed) != 0 {
		t.Fatalf("单片不应与自己冲突，峰值 %d 超限 %+v", res.Peak, res.Exceed)
	}
}

// 超限区间跨 0°/360° 时合并为一条回卷弧。
func TestSweepExceedWrapsAcrossZero(t *testing.T) {
	mk := func(id string, phase float64) Cam { return cam(id, phase, 100, 0, 50, 210) }
	a, b := mk("a", 300), mk("b", 340) // [300,90) [340,130)：交集 [340,90) 跨零
	res := Sweep([]Cam{a, b}, 1)
	if len(res.Exceed) != 1 {
		t.Fatalf("超限区间应为 1 段: %+v", res.Exceed)
	}
	arcEq(t, res.Exceed[0].Arc, 340, 110) // 340→360 再 0→90
	v := res.Exceed[0].Arc.View()
	if !v.Wraps || v.EndDeg != 90 {
		t.Fatalf("回卷超限区间视图错误: %+v", v)
	}
}
