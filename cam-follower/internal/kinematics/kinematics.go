// Package kinematics 把无因次规律还原成实际量纲：
//
//	T = θ/β（升程段；回程段 T=(θ-θ0)/β 沿回程行程推进）
//	s = h·y(T)                 [与 h 同单位]
//	v = h·ω/β·y'(T)            [位移单位/时间单位]
//	a = h·ω²/β²·y''(T)
//	j = h·ω³/β³·y'''(T)
//
// 全程 s、v、a、j 来自同一套闭式解析式对转角逐次求导，
// 再乘 ω 的 0、1、2、3 次方，不使用有限差分。
//
// 角度单位在本服务内钉死为度：β 以度计，ω 以度/秒计，
// 一周 FullTurn=360°。
package kinematics

import (
	"errors"
	"fmt"
	"math"

	"camfollower/internal/law"
)

const (
	AngleUnit     = "degree" // 全程钉死的角度单位
	AngleSym      = "°"
	OmegaUnit     = "degree/s"
	FullTurn      = 360.0 // 一周，单位度
	DefaultPoints = 361   // 曲线默认网格点数（0°..360° 每度一点）
	MinPoints     = 2
	MaxPoints     = 200001
)

// Spec 是一段升程/回程的运动参数。
type Spec struct {
	Type  string  // 规律类型：cosine / cycloid / parabolic
	H     float64 // 升程（必须为正）
	Beta  float64 // 升程角（度，必须为正且小于一周）
	Omega float64 // 角速度（度/秒，必须为正）
}

func (s Spec) Validate() error {
	if !law.Known(s.Type) {
		return law.ErrUnknownLaw
	}
	if math.IsNaN(s.H) || math.IsInf(s.H, 0) || s.H <= 0 {
		return errors.New("h 必须为正数")
	}
	if math.IsNaN(s.Beta) || math.IsInf(s.Beta, 0) || s.Beta <= 0 {
		return errors.New("β 必须为正数")
	}
	if s.Beta >= FullTurn {
		return fmt.Errorf("β 必须小于一周（%.0f°），当前为 %g°", FullTurn, s.Beta)
	}
	if math.IsNaN(s.Omega) || math.IsInf(s.Omega, 0) || s.Omega <= 0 {
		return errors.New("ω 必须为正数")
	}
	return nil
}

// Motion 是某一转角处的从动件运动学量。
type Motion struct {
	Theta float64 `json:"theta_deg"` // 转角（度）
	Time  float64 `json:"t_s"`       // 该段起算时间（秒）
	S     float64 `json:"s"`         // 位移
	V     float64 `json:"v"`         // 速度
	A     float64 `json:"a"`         // 加速度
	J     float64 `json:"j"`         // 跃度
}

// scales 为无因次导数量纲还原时的三个倍率。
type scales struct {
	v float64 // h·ω/β
	a float64 // h·ω²/β²
	j float64 // h·ω³/β³
}

func (s Spec) factor() scales {
	o := s.Omega / s.Beta
	return scales{v: s.H * o, a: s.H * o * o, j: s.H * o * o * o}
}

// restore 把无因次值乘回量纲（位移乘 h）。returning=true 表示回程：
// 位移对 h 镜像，速度/加速度/跃度同时取负。
func restore(w law.Values, f scales, spec Spec, theta, t float64, returning bool) Motion {
	m := Motion{
		Theta: theta,
		Time:  t,
		S:     spec.H * w.Y,
		V:     f.v * w.Y1,
		A:     f.a * w.Y2,
		J:     f.j * w.Y3,
	}
	if returning {
		m.S = spec.H - m.S
		m.V, m.A, m.J = -m.V, -m.A, -m.J
	}
	return m
}

// Sample 求升程段（或 returning=true 时回程段）在全局转角 theta 处的闭式值。
// theta0 为该段起始转角，t 为该段起算时间。
func Sample(spec Spec, theta0, theta, t float64, returning bool) (Motion, error) {
	if err := spec.Validate(); err != nil {
		return Motion{}, err
	}
	T := (theta - theta0) / spec.Beta
	w, err := law.Eval(spec.Type, T)
	if err != nil {
		return Motion{}, err
	}
	m := restore(w, spec.factor(), spec, theta, t, returning)
	return m, nil
}

// SampleSide 与 Sample 相同，但在等加速中点 T=0.5 处可取单侧
// （side=-1 前半 / +1 后半），供接头连续性核对。
func SampleSide(spec Spec, theta0, theta, t float64, returning bool, side float64) (Motion, error) {
	if err := spec.Validate(); err != nil {
		return Motion{}, err
	}
	T := (theta - theta0) / spec.Beta
	w, err := law.EvalSide(spec.Type, T, side)
	if err != nil {
		return Motion{}, err
	}
	return restore(w, spec.factor(), spec, theta, t, returning), nil
}

// Peaks 为闭式峰值（均为绝对值）。
type Peaks struct {
	VMax float64 `json:"v_max"`
	AMax float64 `json:"a_max"`
	JMax float64 `json:"j_max"`
}

// ClosedPeaks 由 h、β、ω 直接写出闭式峰值：
//
//	v_max = hω/β·|y'|max，a_max = hω²/β²·|y''|max，j_max = hω³/β³·|y'''|max。
func ClosedPeaks(spec Spec) (Peaks, error) {
	if err := spec.Validate(); err != nil {
		return Peaks{}, err
	}
	p, err := law.ClosedPeaks(spec.Type)
	if err != nil {
		return Peaks{}, err
	}
	f := spec.factor()
	return Peaks{VMax: f.v * p.V1, AMax: f.a * p.V2, JMax: f.j * p.V3}, nil
}

// Curve 在升程段（0..β）上等距取 points 个点，端点 0 与 β 必在网格中。
func Curve(spec Spec, points int) ([]Motion, Peaks, error) {
	if err := spec.Validate(); err != nil {
		return nil, Peaks{}, err
	}
	if points < MinPoints {
		points = MinPoints
	}
	if points > MaxPoints {
		return nil, Peaks{}, fmt.Errorf("网格点数超出上限 %d", MaxPoints)
	}
	f := spec.factor()
	out := make([]Motion, points)
	for i := 0; i < points; i++ {
		T := float64(i) / float64(points-1)
		w, err := law.Eval(spec.Type, T)
		if err != nil {
			return nil, Peaks{}, err
		}
		theta := T * spec.Beta
		out[i] = restore(w, f, spec, theta, T*spec.Beta/spec.Omega, false)
	}
	p, err := ClosedPeaks(spec)
	if err != nil {
		return nil, Peaks{}, err
	}
	return out, p, nil
}

// Seam 是一个接头左右两侧的运动学量及连续性核对结果。
type Seam struct {
	Theta float64 `json:"theta_deg"`
	Kind  string  `json:"kind"` // boundary 段接头 / switch 规律内部切换 / cyclic 首尾闭合
	Left  Motion  `json:"left"`
	Right Motion  `json:"right"`
	SCont bool    `json:"s_continuous"`
	VCont bool    `json:"v_continuous"`
	ACont bool    `json:"a_continuous"`
	JCont bool    `json:"j_continuous"`
	SJump bool    `json:"s_jump"` // 位移跳变（任何接头都不应出现）
}

// seamTol 为连续性核对的相对容差，另加 1e-12 绝对地板避免近零误判。
const seamRelTol = 1e-9
const seamAbsFloor = 1e-12

func approxEqual(x, y, scale float64) bool {
	tol := seamAbsFloor + seamRelTol*scale
	return math.Abs(x-y) <= tol
}

// Join 比较同一接头处左右两侧闭式值，逐量核对连续性。
// scale 以两侧 |量| 的较大值给出，升程-回程相邻时两侧量纲倍率相同，
// 因此直接取较大绝对值即可。
func Join(theta float64, kind string, left, right Motion) Seam {
	sScale := math.Max(math.Abs(left.S), math.Abs(right.S))
	// 导数的尺度参考位移为 0 的停歇接头也能工作，取该量自身两侧较大值。
	vScale := math.Max(math.Abs(left.V), math.Abs(right.V))
	aScale := math.Max(math.Abs(left.A), math.Abs(right.A))
	jScale := math.Max(math.Abs(left.J), math.Abs(right.J))
	sm := Seam{
		Theta: theta,
		Kind:  kind,
		Left:  left,
		Right: right,
		SCont: approxEqual(left.S, right.S, sScale),
		VCont: approxEqual(left.V, right.V, vScale),
		ACont: approxEqual(left.A, right.A, aScale),
		JCont: approxEqual(left.J, right.J, jScale),
	}
	sm.SJump = !sm.SCont
	return sm
}

// InternalSeams 返回单段升程曲线内部需要核对的接头：
// 等加速等减速在 T=0.5 的加速度变号点（位移、速度连续，加速度跳变）。
// 余弦与摆线全程光滑，返回 nil。
func InternalSeams(spec Spec, theta0 float64) ([]Seam, error) {
	if err := spec.Validate(); err != nil {
		return nil, err
	}
	if spec.Type != law.Parabolic {
		return nil, nil
	}
	tMid := 0.5 * spec.Beta / spec.Omega
	theta := theta0 + spec.Beta/2
	left, err := SampleSide(spec, theta0, theta, tMid, false, -1)
	if err != nil {
		return nil, err
	}
	right, err := SampleSide(spec, theta0, theta, tMid, false, +1)
	if err != nil {
		return nil, err
	}
	return []Seam{Join(theta, "switch", left, right)}, nil
}
