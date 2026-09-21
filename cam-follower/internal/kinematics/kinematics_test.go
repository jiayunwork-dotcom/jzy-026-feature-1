package kinematics

import (
	"math"
	"testing"

	"camfollower/internal/law"
)

func near(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s = %v, 期望 %v（容差 %v）", name, got, want, tol)
	}
}

// 所有规律共用一套测试参数。
func testSpec(tp string, h, beta, omega float64) Spec {
	return Spec{Type: tp, H: h, Beta: beta, Omega: omega}
}

// 升程段：转角 0 时 s=0，转角 β 时 s=h，各规律端点速度为 0。
func TestLiftEndpointsAllLaws(t *testing.T) {
	for _, tp := range []string{law.Cosine, law.Cycloid, law.Parabolic} {
		sp := testSpec(tp, 10, 60, 360)
		start, err := Sample(sp, 0, 0, 0, false)
		if err != nil {
			t.Fatal(err)
		}
		end, err := Sample(sp, 0, sp.Beta, sp.Beta/sp.Omega, false)
		if err != nil {
			t.Fatal(err)
		}
		near(t, tp+" s(0)", start.S, 0, 1e-10)
		near(t, tp+" s(β)", end.S, sp.H, 1e-9)
		near(t, tp+" v(0)", start.V, 0, 1e-10)
		near(t, tp+" v(β)", end.V, 0, 1e-9)

		// 曲线网格最后一点同样必须正好落在 (β, h)，不能靠差分凑。
		pts, _, err := Curve(sp, 121)
		if err != nil {
			t.Fatal(err)
		}
		last := pts[len(pts)-1]
		near(t, tp+" grid θ", last.Theta, sp.Beta, 1e-9)
		near(t, tp+" grid s", last.S, sp.H, 1e-9)
		first := pts[0]
		near(t, tp+" grid s0", first.S, 0, 1e-12)
	}
}

// 摆线两端加速度为 0。
func TestCycloidEndAccelerationZero(t *testing.T) {
	sp := testSpec(law.Cycloid, 10, 60, 360)
	start, _ := Sample(sp, 0, 0, 0, false)
	end, _ := Sample(sp, 0, sp.Beta, 0, false)
	near(t, "cycloid a(0)", start.A, 0, 1e-9)
	near(t, "cycloid a(β)", end.A, 0, 1e-8)
}

// 等加速两端速度为 0，但加速度不为 0（已知性质，不得偷偷改成 0）。
func TestParabolicEndAccelerationNonZero(t *testing.T) {
	sp := testSpec(law.Parabolic, 10, 60, 360)
	start, _ := Sample(sp, 0, 0, 0, false)
	end, _ := Sample(sp, 0, sp.Beta, 0, false)
	near(t, "parabolic v(0)", start.V, 0, 1e-12)
	near(t, "parabolic v(β)", end.V, 0, 1e-12)
	if math.Abs(start.A) < 1e-9 || math.Abs(end.A) < 1e-9 {
		t.Fatalf("等加速两端加速度必须非 0，得到 a(0)=%v a(β)=%v", start.A, end.A)
	}
	// 闭式：a(0)=4hω²/β²，a(β)=-4hω²/β²
	want := 4 * sp.H * sp.Omega * sp.Omega / (sp.Beta * sp.Beta)
	near(t, "parabolic a(0)=4hω²/β²", start.A, want, 1e-6)
	near(t, "parabolic a(β)=-4hω²/β²", end.A, -want, 1e-6)
}

// 等加速中点加速度变号，速度连续，终点位移仍为 h。
func TestParabolicMidpointSignFlip(t *testing.T) {
	sp := testSpec(law.Parabolic, 10, 90, 180)
	thetaM := sp.Beta / 2
	tm := thetaM / sp.Omega
	lo, err := SampleSide(sp, 0, thetaM, tm, false, -1)
	if err != nil {
		t.Fatal(err)
	}
	hi, err := SampleSide(sp, 0, thetaM, tm, false, +1)
	if err != nil {
		t.Fatal(err)
	}
	if !(lo.A > 0 && hi.A < 0) {
		t.Fatalf("中点加速度应变号: a-=%v a+=%v", lo.A, hi.A)
	}
	near(t, "中点 s 连续", lo.S, hi.S, 1e-10)
	near(t, "中点 v 连续", lo.V, hi.V, 1e-9)
	near(t, "中点 s=h/2", lo.S, sp.H/2, 1e-9)

	seams, err := InternalSeams(sp, 0)
	if err != nil {
		t.Fatal(err)
	}
	if len(seams) != 1 || seams[0].ACont || !seams[0].SCont || !seams[0].VCont {
		t.Fatalf("等加速内部接头应对账为 s/v 连续、a 跳变: %+v", seams)
	}

	end, _ := Sample(sp, 0, sp.Beta, 0, false)
	near(t, "等加速终点 s=h", end.S, sp.H, 1e-9)
}

// 余弦两端速度为 0（加速度不为 0）。
func TestCosineEndVelocityZero(t *testing.T) {
	sp := testSpec(law.Cosine, 5, 45, 720)
	start, _ := Sample(sp, 0, 0, 0, false)
	end, _ := Sample(sp, 0, sp.Beta, 0, false)
	near(t, "cosine v(0)", start.V, 0, 1e-10)
	near(t, "cosine v(β)", end.V, 0, 1e-9)
	if math.Abs(start.A) < 1e-6 {
		t.Fatal("余弦起点加速度按闭式 π²hω²/(2β²) 应非 0")
	}
}

// h 加大：同 β、ω 下全程 s、v、a、j 按同一比例放大。
func TestScalesLinearlyWithH(t *testing.T) {
	k := 3.0
	pts1 := curvePoints(t, testSpec(law.Cycloid, 10, 60, 360), 77)
	pts2 := curvePoints(t, testSpec(law.Cycloid, 10*k, 60, 360), 77)
	for i := range pts1 {
		near(t, "s∝h", pts2[i].S, k*pts1[i].S, 1e-9*k)
		near(t, "v∝h", pts2[i].V, k*pts1[i].V, 1e-8*k)
		near(t, "a∝h", pts2[i].A, k*pts1[i].A, 1e-7*k)
		near(t, "j∝h", pts2[i].J, k*pts1[i].J, 1e-6*k)
	}
}

// β 加大、ω 不变：峰值速度下降。
func TestPeakVelocityDecreasesWithBeta(t *testing.T) {
	p1 := samplePeaks(t, testSpec(law.Cycloid, 10, 60, 360))
	p2 := samplePeaks(t, testSpec(law.Cycloid, 10, 120, 360))
	if !(p2.v < p1.v) {
		t.Fatalf("β 加大峰值速度应下降: %v → %v", p1.v, p2.v)
	}
	// 闭式 v_max=2hω/β，随 β 反比下降
	near(t, "v1=2hω/β", p1.v, 2*10*360/60, 1e-6)
}

// 摆线峰值加速度、峰值跃度必须对得上 h、β、ω 的闭式。
func TestCycloidClosedFormPeaks(t *testing.T) {
	sp := testSpec(law.Cycloid, 12, 75, 400)
	pk, err := ClosedPeaks(sp)
	if err != nil {
		t.Fatal(err)
	}
	fv := sp.H * sp.Omega / sp.Beta
	fa := fv * sp.Omega / sp.Beta
	fj := fa * sp.Omega / sp.Beta
	near(t, "v_max=2hω/β", pk.VMax, 2*fv, 1e-6)
	near(t, "a_max=2πhω²/β²", pk.AMax, 2*math.Pi*fa, 1e-6)
	near(t, "j_max=4π²hω³/β³", pk.JMax, 4*math.Pi*math.Pi*fj, 1e-6)

	// 闭式峰值还要对得上网格上实际取到的峰值。
	got := samplePeaks(t, sp)
	near(t, "网格 |v|max", got.v, pk.VMax, pk.VMax*1e-2)
	near(t, "网格 |a|max", got.a, pk.AMax, pk.AMax*2e-2)
	near(t, "网格 |j|max", got.j, pk.JMax, pk.JMax*2e-2)
}

// 未知规律在求曲线前直接拒绝。
func TestUnknownLawRejected(t *testing.T) {
	sp := testSpec("modified-sine", 10, 60, 360)
	if _, _, err := Curve(sp, 10); err == nil {
		t.Fatal("未知规律必须被拒绝")
	}
	if err := sp.Validate(); err == nil {
		t.Fatal("Validate 必须拒绝未知规律")
	}
	if _, err := ClosedPeaks(sp); err == nil {
		t.Fatal("ClosedPeaks 必须拒绝未知规律")
	}
}

// 参数边界：h>0、0<β<360、ω>0。
func TestSpecValidation(t *testing.T) {
	bad := []Spec{
		{Type: law.Cycloid, H: 0, Beta: 60, Omega: 360},
		{Type: law.Cycloid, H: -1, Beta: 60, Omega: 360},
		{Type: law.Cycloid, H: 10, Beta: 0, Omega: 360},
		{Type: law.Cycloid, H: 10, Beta: 360, Omega: 360},
		{Type: law.Cycloid, H: 10, Beta: 400, Omega: 360},
		{Type: law.Cycloid, H: 10, Beta: 60, Omega: 0},
		{Type: law.Cycloid, H: 10, Beta: 60, Omega: -9},
	}
	for i, sp := range bad {
		if err := sp.Validate(); err == nil {
			t.Fatalf("非法参数组 #%d 必须被拒绝: %+v", i, sp)
		}
	}
}

// 回程镜像：从 h 回到 0，端点连续性与升程相同。
func TestReturnMirror(t *testing.T) {
	sp := testSpec(law.Cycloid, 10, 60, 360)
	start, _ := Sample(sp, 0, 0, 0, true)
	end, _ := Sample(sp, 0, sp.Beta, 0, true)
	near(t, "return s(0)=h", start.S, sp.H, 1e-9)
	near(t, "return s(β)=0", end.S, 0, 1e-9)
	near(t, "return v(0)=0", start.V, 0, 1e-9)
	near(t, "return a(0)=0", start.A, 0, 1e-9)
	near(t, "return a(β)=0", end.A, 0, 1e-8)
}

func curvePoints(t *testing.T, sp Spec, n int) []Motion {
	t.Helper()
	pts, _, err := Curve(sp, n)
	if err != nil {
		t.Fatal(err)
	}
	return pts
}

type gridPeaks struct{ v, a, j float64 }

func samplePeaks(t *testing.T, sp Spec) gridPeaks {
	t.Helper()
	pts := curvePoints(t, sp, 20001)
	var p gridPeaks
	for _, m := range pts {
		p.v = math.Max(p.v, math.Abs(m.V))
		p.a = math.Max(p.a, math.Abs(m.A))
		p.j = math.Max(p.j, math.Abs(m.J))
	}
	return p
}
