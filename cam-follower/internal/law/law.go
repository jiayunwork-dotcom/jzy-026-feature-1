// Package law 实现三条无因次运动规律：余弦（简谐）、摆线、等加速等减速。
// 无因次时间 T 为当前升程段转角 / 升程角，取值区间 [0,1]。
// 各规律给出位移 y=s/h 以及对 T 的前三次导数 y'、y”、y”'，
// 量纲还原（乘 h、β、ω 的相应次方）在 internal/kinematics 包完成。
package law

import (
	"errors"
	"math"
)

const (
	Cosine    = "cosine"    // 余弦（简谐）
	Cycloid   = "cycloid"   // 摆线
	Parabolic = "parabolic" // 等加速等减速（二次抛物线）
)

var ErrUnknownLaw = errors.New("未知规律类型，支持: cosine（余弦）、cycloid（摆线）、parabolic（等加速等减速）")

// Known 报告类型名是否为受支持的规律。
func Known(t string) bool {
	return t == Cosine || t == Cycloid || t == Parabolic
}

// Values 是一条规律在无因次时间 T 处的位移及对 T 的前三阶导数。
type Values struct {
	Y  float64 // s/h
	Y1 float64 // d(s/h)/dT
	Y2 float64 // d²(s/h)/dT²
	Y3 float64 // d³(s/h)/dT³
}

// Eval 按闭式解析式计算 T 处的无因次值。T 会被钳到 [0,1]。
// 注意：等加速等减速在 T=0.5 处 y”' 含冲量（δ 函数），
// 分段多项式取值为 0；该点左右两侧 y” 异号，需用 EvalSide 核对。
func Eval(tp string, T float64) (Values, error) {
	return EvalSide(tp, T, 0)
}

// EvalSide 与 Eval 相同，但对“只在单侧有定义的导数”可按侧取值。
//
//	side = -1：取 T 从下方趋近的值（前半段公式）
//	side =  0：钳到精确点（T==0.5 时中点取左半加速度）
//	side = +1：取 T 从上方趋近的值（后半段公式）
//
// 余弦与摆线全程光滑，side 不影响结果。
func EvalSide(tp string, T, side float64) (Values, error) {
	if T < 0 {
		T = 0
	} else if T > 1 {
		T = 1
	}
	switch tp {
	case Cosine:
		// y = (1 - cos(πT))/2
		// y' = π/2·sin(πT)
		// y'' = π²/2·cos(πT)
		// y''' = -π³/2·sin(πT)
		p := math.Pi * T
		return Values{
			Y:  (1 - math.Cos(p)) / 2,
			Y1: (math.Pi / 2) * math.Sin(p),
			Y2: (math.Pi * math.Pi / 2) * math.Cos(p),
			Y3: -(math.Pi * math.Pi * math.Pi / 2) * math.Sin(p),
		}, nil
	case Cycloid:
		// y = T - sin(2πT)/(2π)
		// y' = 1 - cos(2πT)
		// y'' = 2π·sin(2πT)
		// y''' = 4π²·cos(2πT)
		q := 2 * math.Pi * T
		return Values{
			Y:  T - math.Sin(q)/(2*math.Pi),
			Y1: 1 - math.Cos(q),
			Y2: 2 * math.Pi * math.Sin(q),
			Y3: 4 * math.Pi * math.Pi * math.Cos(q),
		}, nil
	case Parabolic:
		return evalParabolic(T, side), nil
	default:
		return Values{}, ErrUnknownLaw
	}
}

// evalParabolic 等加速（T≤1/2）/等减速（T≥1/2）：
//
//	前半: y=2T²      y'=4T       y''=4        y'''=0
//	后半: y=1-2(1-T)² y'=4(1-T)  y''=-4       y'''=0
//	中点: y=1/2, y'=2（连续），y'' 由 +4 跳到 -4（变号）。
func evalParabolic(T, side float64) Values {
	switch {
	case T < 0.5 || (T == 0.5 && side < 0):
		return Values{Y: 2 * T * T, Y1: 4 * T, Y2: 4, Y3: 0}
	case T > 0.5 || (T == 0.5 && side > 0):
		d := 1 - T
		return Values{Y: 1 - 2*d*d, Y1: 4 * d, Y2: -4, Y3: 0}
	default:
		// 精确中点、未指定侧：位移与速度两侧一致，加速度按跃变点左侧记。
		return Values{Y: 0.5, Y1: 2, Y2: 4, Y3: 0}
	}
}

// Peaks 返回 |y'|、|y”|、|y”'| 的闭式最大值（无因次峰值），
// 供量纲还原后核对闭式峰值。
type Peaks struct {
	V1 float64
	V2 float64
	V3 float64
}

func ClosedPeaks(tp string) (Peaks, error) {
	switch tp {
	case Cosine:
		// |y'|max=π/2，|y''|max=π²/2，|y'''|max=π³/2
		return Peaks{math.Pi / 2, math.Pi * math.Pi / 2, math.Pi * math.Pi * math.Pi / 2}, nil
	case Cycloid:
		// |y'|max=2，|y''|max=2π，|y'''|max=4π²
		return Peaks{2, 2 * math.Pi, 4 * math.Pi * math.Pi}, nil
	case Parabolic:
		// |y'|max=2，|y''|max=4；y''' 仅以中点冲量存在，分段多项式恒为 0
		return Peaks{2, 4, 0}, nil
	default:
		return Peaks{}, ErrUnknownLaw
	}
}
