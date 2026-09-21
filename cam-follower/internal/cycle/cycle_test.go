package cycle

import (
	"math"
	"testing"

	"camfollower/internal/kinematics"
	"camfollower/internal/law"
)

func closeTo(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s = %v, 期望 %v（容差 %v）", name, got, want, tol)
	}
}

func validConfig() Config {
	return Config{
		RiseLaw: law.Cycloid, ReturnLaw: law.Cosine,
		H: 10, Omega: 360,
		RiseAngle: 60, OuterDwell: 30, ReturnAngle: 90, InnerDwell: 180,
	}
}

// 完整循环：拼接角处位移必须连续（含 360°/0° 闭合），不允许位移跳变。
func TestFullCycleDisplacementContinuous(t *testing.T) {
	segs, err := Build(validConfig())
	if err != nil {
		t.Fatal(err)
	}
	joints, err := segs.Discontinuities()
	if err != nil {
		t.Fatal(err)
	}
	if HasDisplacementJump(joints) {
		for _, j := range joints {
			t.Logf("接头 %.1f° %s→%s: s-=%v s+=%v", j.Theta, j.LeftSegment, j.RightSegment, j.Left.S, j.Right.S)
		}
		t.Fatal("循环接头出现位移跳变")
	}
	for _, j := range joints {
		if !j.SCont {
			t.Fatalf("接头 %.1f° 位移不连续", j.Theta)
		}
	}
}

// 停歇段 s 保持 0 或 h，v、a、j 全为 0；段数与段界角度正确。
func TestDwellDerivativesZero(t *testing.T) {
	cfg := validConfig()
	segs, err := Build(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs.All) != 4 {
		t.Fatalf("四段都存在时应有 4 段，得到 %d", len(segs.All))
	}

	// 远休止（60°..90°）：s=h
	for _, theta := range []float64{60, 75, 90} {
		m, err := segs.Point(theta)
		if err != nil {
			t.Fatal(err)
		}
		closeTo(t, "远休止 s", m.S, cfg.H, 1e-9)
		if m.V != 0 || m.A != 0 || m.J != 0 {
			t.Fatalf("远休止导数必须全 0: %+v", m)
		}
	}
	// 近休止（180°..360°）：s=0
	for _, theta := range []float64{180, 270, 360} {
		m, err := segs.Point(theta)
		if err != nil {
			t.Fatal(err)
		}
		closeTo(t, "近休止 s", m.S, 0, 1e-9)
		if m.V != 0 || m.A != 0 || m.J != 0 {
			t.Fatalf("近休止导数必须全 0: %+v", m)
		}
	}

	// 升程终点 s=h，回程终点 s=0
	atRiseEnd, _ := segs.Point(60)
	closeTo(t, "升程终点", atRiseEnd.S, cfg.H, 1e-8)
	atReturnEnd, _ := segs.Point(180)
	closeTo(t, "回程终点", atReturnEnd.S, 0, 1e-8)
	// 起点与一周终点
	at0, _ := segs.Point(0)
	at360, _ := segs.Point(360)
	closeTo(t, "s(0)", at0.S, 0, 1e-9)
	closeTo(t, "s(360)", at360.S, 0, 1e-9)
}

// 停歇角为 0 表示该段省略，相邻两段位移仍要接对。
func TestZeroDwellOmitted(t *testing.T) {
	cfg := Config{
		RiseLaw: law.Parabolic, ReturnLaw: law.Parabolic,
		H: 10, Omega: 360,
		RiseAngle: 90, OuterDwell: 0, ReturnAngle: 90, InnerDwell: 180,
	}
	segs, err := Build(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs.All) != 3 {
		t.Fatalf("远休止为 0 应省略，剩 3 段，得到 %d", len(segs.All))
	}
	// 升程与回程直接相接：接头在 90°，两侧都应是 s=h
	joints, err := segs.Discontinuities()
	if err != nil {
		t.Fatal(err)
	}
	if HasDisplacementJump(joints) {
		t.Fatal("省略停歇段后接头位移不得跳变")
	}
	var boundary *Joint
	for i := range joints {
		if joints[i].Theta == 90 && joints[i].Kind == "boundary" {
			boundary = &joints[i]
		}
	}
	if boundary == nil {
		t.Fatal("未找到升程-回程直接接头")
	}
	closeTo(t, "接头左 s", boundary.Left.S, cfg.H, 1e-8)
	closeTo(t, "接头右 s", boundary.Right.S, cfg.H, 1e-8)

	// 两段停歇皆为 0：升程接回程，回程接升程（首尾闭合）。
	cfg2 := Config{
		RiseLaw: law.Cycloid, ReturnLaw: law.Cycloid,
		H: 4, Omega: 720,
		RiseAngle: 180, OuterDwell: 0, ReturnAngle: 180, InnerDwell: 0,
	}
	segs2, err := Build(cfg2)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs2.All) != 2 {
		t.Fatalf("两段停歇皆为 0 应只剩 2 段，得到 %d", len(segs2.All))
	}
	j2, err := segs2.Discontinuities()
	if err != nil {
		t.Fatal(err)
	}
	if HasDisplacementJump(j2) {
		t.Fatal("无停歇循环仍必须闭合且无位移跳变")
	}
}

// 等加速循环：内部中点接头应报告加速度变号（s、v 连续，a 不连续）。
func TestParabolicSwitchSeam(t *testing.T) {
	cfg := Config{
		RiseLaw: law.Parabolic, ReturnLaw: law.Parabolic,
		H: 10, Omega: 360,
		RiseAngle: 120, OuterDwell: 0, ReturnAngle: 120, InnerDwell: 120,
	}
	segs, err := Build(cfg)
	if err != nil {
		t.Fatal(err)
	}
	joints, err := segs.Discontinuities()
	if err != nil {
		t.Fatal(err)
	}
	var switches int
	for _, j := range joints {
		if j.Kind == "switch" {
			switches++
			if j.ACont || !j.SCont || !j.VCont {
				t.Fatalf("等加速中点应为 s/v 连续、a 变号: %+v", j)
			}
			if !(j.Left.A > 0 && j.Right.A < 0) &&
				!(j.LeftSegment == "return" && j.Left.A < 0 && j.Right.A > 0) {
				t.Fatalf("中点加速度未正确变号: %+v", j)
			}
		}
	}
	if switches != 2 {
		t.Fatalf("升程、回程各一个中点切换点，得到 %d 个", switches)
	}
}

// 网格整周覆盖 0..360，两端点在网格中。
func TestCurveGridCoversFullTurn(t *testing.T) {
	segs, err := Build(validConfig())
	if err != nil {
		t.Fatal(err)
	}
	pts, err := segs.Curve(kinematics.DefaultPoints)
	if err != nil {
		t.Fatal(err)
	}
	if len(pts) != kinematics.DefaultPoints {
		t.Fatalf("点数应为 %d", kinematics.DefaultPoints)
	}
	closeTo(t, "首点 θ", pts[0].Theta, 0, 1e-9)
	closeTo(t, "末点 θ", pts[len(pts)-1].Theta, 360, 1e-9)
}

// 角度和不为一周、未知类型、缺项必须拒绝。
func TestConfigValidation(t *testing.T) {
	bad := []Config{
		func() Config { c := validConfig(); c.RiseAngle = 50; return c }(),      // 和 ≠ 360
		func() Config { c := validConfig(); c.OuterDwell = -1; return c }(),     // 负停歇角
		func() Config { c := validConfig(); c.ReturnLaw = "weird"; return c }(), // 未知类型
		func() Config { c := validConfig(); c.H = 0; return c }(),               // h 缺
		func() Config { c := validConfig(); c.Omega = 0; return c }(),           // ω 缺
		func() Config { c := validConfig(); c.RiseAngle = 360; return c }(),     // β 越界
	}
	for i, c := range bad {
		if err := c.Validate(); err == nil {
			t.Fatalf("非法配置 #%d 必须被拒绝: %+v", i, c)
		}
	}
}
