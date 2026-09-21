package law

import (
	"math"
	"testing"
)

func approxEq(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Fatalf("%s = %v, 期望 %v（容差 %v）", name, got, want, tol)
	}
}

func TestCosineEndpoints(t *testing.T) {
	v0, err := EvalSide(Cosine, 0, -1)
	if err != nil {
		t.Fatal(err)
	}
	v1, err := EvalSide(Cosine, 1, +1)
	if err != nil {
		t.Fatal(err)
	}
	approxEq(t, "cos y(0)", v0.Y, 0, 1e-15)
	approxEq(t, "cos y(1)", v1.Y, 1, 1e-15)
	approxEq(t, "cos y'(0)", v0.Y1, 0, 1e-15)
	approxEq(t, "cos y'(1)", v1.Y1, 0, 1e-15)
}

func TestCycloidEndpoints(t *testing.T) {
	v0, err := EvalSide(Cycloid, 0, -1)
	if err != nil {
		t.Fatal(err)
	}
	v1, err := EvalSide(Cycloid, 1, +1)
	if err != nil {
		t.Fatal(err)
	}
	approxEq(t, "cyc y(0)", v0.Y, 0, 1e-15)
	approxEq(t, "cyc y(1)", v1.Y, 1, 1e-12)
	approxEq(t, "cyc y'(0)", v0.Y1, 0, 1e-15)
	approxEq(t, "cyc y'(1)", v1.Y1, 0, 1e-12)
	approxEq(t, "cyc y''(0)", v0.Y2, 0, 1e-15)
	approxEq(t, "cyc y''(1)", v1.Y2, 0, 1e-12)
}

func TestParabolicMidpointSwitch(t *testing.T) {
	lo, err := EvalSide(Parabolic, 0.5, -1)
	if err != nil {
		t.Fatal(err)
	}
	hi, err := EvalSide(Parabolic, 0.5, +1)
	if err != nil {
		t.Fatal(err)
	}
	// 位移与速度在中点连续
	approxEq(t, "para y(0.5-)", lo.Y, 0.5, 1e-15)
	approxEq(t, "para y(0.5+)", hi.Y, 0.5, 1e-15)
	approxEq(t, "para y'(0.5-)", lo.Y1, 2, 1e-15)
	approxEq(t, "para y'(0.5+)", hi.Y1, 2, 1e-15)
	// 加速度变号
	if lo.Y2 != 4 || hi.Y2 != -4 {
		t.Fatalf("中点应变号: y''-=%v, y''+=%v", lo.Y2, hi.Y2)
	}
	// 两端位移
	v0, _ := Eval(Parabolic, 0)
	v1, _ := Eval(Parabolic, 1)
	approxEq(t, "para y(0)", v0.Y, 0, 1e-15)
	approxEq(t, "para y(1)", v1.Y, 1, 1e-15)
}

func TestUnknownLawRejected(t *testing.T) {
	if _, err := Eval("sine", 0.5); err != ErrUnknownLaw {
		t.Fatalf("未知规律应返回 ErrUnknownLaw，得到 %v", err)
	}
	if Known("cycloid") != true || Known("nope") != false {
		t.Fatal("Known 判断错误")
	}
}
