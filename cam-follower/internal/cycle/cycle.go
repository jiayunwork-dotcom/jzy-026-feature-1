// Package cycle 把升程、远休止、回程、近休止四段拼成一条 360° 循环曲线，
// 负责网格采样、接头两侧闭式值比对与连续性核对。
//
// 段顺序固定为：升程（rise）→ 远休止（outer dwell，s=h）→
// 回程（return，位移镜像，从 h 回到 0）→ 近休止（inner dwell，s=0）。
// 停歇角为 0 时该段省略，相邻两段直接相接。
package cycle

import (
	"errors"
	"fmt"
	"math"

	"camfollower/internal/kinematics"
	"camfollower/internal/law"
)

// Config 是一条完整循环的四段角度与各段规律。
type Config struct {
	RiseLaw     string  `json:"rise_law"`         // 升程规律
	ReturnLaw   string  `json:"return_law"`       // 回程规律（位移镜像）
	H           float64 `json:"h"`                // 升程
	RiseAngle   float64 `json:"rise_angle_deg"`   // 升程角 β1
	OuterDwell  float64 `json:"outer_dwell_deg"`  // 远休止角（0 表示省略）
	ReturnAngle float64 `json:"return_angle_deg"` // 回程角 β2
	InnerDwell  float64 `json:"inner_dwell_deg"`  // 近休止角（0 表示省略）
	Omega       float64 `json:"omega_deg_s"`      // 角速度（度/秒）
}

// Segment 是拼接后实际存在的一段。
type Segment struct {
	Kind  string          `json:"kind"` // rise / outer_dwell / return / inner_dwell
	Law   string          `json:"law,omitempty"`
	Start float64         `json:"start_deg"`
	End   float64         `json:"end_deg"`
	Beta  float64         `json:"beta_deg,omitempty"`
	Spec  kinematics.Spec `json:"-"`
}

// Segments 是拼接结果（已省略角度为 0 的停歇段）。
type Segments struct {
	All        []*Segment
	TotalAngle float64
}

func (c Config) Validate() error {
	if !law.Known(c.RiseLaw) {
		return fmt.Errorf("升程规律无效：%w", law.ErrUnknownLaw)
	}
	if !law.Known(c.ReturnLaw) {
		return fmt.Errorf("回程规律无效：%w", law.ErrUnknownLaw)
	}
	if math.IsNaN(c.H) || math.IsInf(c.H, 0) || c.H <= 0 {
		return errors.New("h 必须为正数")
	}
	if math.IsNaN(c.Omega) || math.IsInf(c.Omega, 0) || c.Omega <= 0 {
		return errors.New("ω 必须为正数")
	}
	if err := checkMotionAngle("升程角", c.RiseAngle); err != nil {
		return err
	}
	if err := checkMotionAngle("回程角", c.ReturnAngle); err != nil {
		return err
	}
	if err := checkDwellAngle("远休止角", c.OuterDwell); err != nil {
		return err
	}
	if err := checkDwellAngle("近休止角", c.InnerDwell); err != nil {
		return err
	}
	total := c.RiseAngle + c.OuterDwell + c.ReturnAngle + c.InnerDwell
	if math.Abs(total-kinematics.FullTurn) > 1e-9 {
		return fmt.Errorf("四段角度之和必须等于一周 %.0f°，当前为 %g°", kinematics.FullTurn, total)
	}
	return nil
}

func checkMotionAngle(name string, b float64) error {
	if math.IsNaN(b) || math.IsInf(b, 0) || b <= 0 {
		return fmt.Errorf("%s必须为正数", name)
	}
	if b >= kinematics.FullTurn {
		return fmt.Errorf("%s必须小于一周（%.0f°）", name, kinematics.FullTurn)
	}
	return nil
}

func checkDwellAngle(name string, b float64) error {
	if math.IsNaN(b) || math.IsInf(b, 0) || b < 0 {
		return fmt.Errorf("%s不能为负", name)
	}
	if b >= kinematics.FullTurn {
		return fmt.Errorf("%s必须小于一周（%.0f°）", name, kinematics.FullTurn)
	}
	return nil
}

// Build 按角度拼接四段，停歇角为 0 的段省略。
func Build(c Config) (*Segments, error) {
	if err := c.Validate(); err != nil {
		return nil, err
	}
	var segs []*Segment
	pos := 0.0
	push := func(kind, tp string, beta float64) {
		if beta <= 0 { // 角度为 0 的停歇段省略；运动段角度恒正（已校验）
			return
		}
		spec := kinematics.Spec{Type: tp, H: c.H, Beta: beta, Omega: c.Omega}
		segs = append(segs, &Segment{
			Kind: kind, Law: tp, Start: pos, End: pos + beta, Beta: beta, Spec: spec,
		})
		pos += beta
	}
	push("rise", c.RiseLaw, c.RiseAngle)
	push("outer_dwell", "", c.OuterDwell)
	push("return", c.ReturnLaw, c.ReturnAngle)
	push("inner_dwell", "", c.InnerDwell)
	return &Segments{All: segs, TotalAngle: pos}, nil
}

// locate 返回 theta 所在段。
// 归属约定：θ 恰在段界时，若有一侧是停歇段则归停歇段（停歇段在端点上
// 同样 s 恒定、导数全 0）；运动段直接相接时归从该点开始的段；
// θ=360 归入最后一段（取其终点值）。另一侧闭式值由 Discontinuities 报告。
func (b *Segments) locate(theta float64) *Segment {
	if theta >= kinematics.FullTurn {
		return b.All[len(b.All)-1]
	}
	// 先找“夹得住”该点的停歇段（含两端点）。
	for _, sg := range b.All {
		if isDwell(sg.Kind) && theta >= sg.Start && theta <= sg.End {
			return sg
		}
	}
	for _, sg := range b.All {
		if theta >= sg.Start && theta < sg.End {
			return sg
		}
	}
	return b.All[len(b.All)-1]
}

func isDwell(kind string) bool { return kind == "outer_dwell" || kind == "inner_dwell" }

func dwellMotion(level, theta, t float64) kinematics.Motion {
	return kinematics.Motion{Theta: theta, Time: t, S: level, V: 0, A: 0, J: 0}
}

// Point 给出循环上任意转角处的闭式运动学量；循环内时间按 θ/ω 从一周起点起算。
func (b *Segments) Point(theta float64) (kinematics.Motion, error) {
	if theta < 0 || theta > kinematics.FullTurn+1e-9 {
		return kinematics.Motion{}, fmt.Errorf("转角超出一周范围 [0, %.0f°]", kinematics.FullTurn)
	}
	if theta > kinematics.FullTurn {
		theta = kinematics.FullTurn
	}
	omega := b.All[0].Spec.Omega // 整周 ω 恒定，各段相同
	sg := b.locate(theta)
	switch sg.Kind {
	case "outer_dwell":
		return dwellMotion(sg.Spec.H, theta, theta/omega), nil
	case "inner_dwell":
		return dwellMotion(0, theta, theta/omega), nil
	case "rise":
		return kinematics.Sample(sg.Spec, sg.Start, theta, sg.Start/omega, false)
	case "return":
		return kinematics.Sample(sg.Spec, sg.Start, theta, sg.Start/omega, true)
	default:
		return kinematics.Motion{}, fmt.Errorf("未知段类型 %q", sg.Kind)
	}
}

// CurveSample 是循环曲线上的一个网格点。
type CurveSample struct {
	Segment string `json:"segment"`
	kinematics.Motion
}

// Curve 在 0..360° 上等距取 points 个点（两端点必在网格中）。
func (b *Segments) Curve(points int) ([]CurveSample, error) {
	if points < kinematics.MinPoints {
		points = kinematics.MinPoints
	}
	if points > kinematics.MaxPoints {
		return nil, fmt.Errorf("网格点数超出上限 %d", kinematics.MaxPoints)
	}
	out := make([]CurveSample, points)
	for i := 0; i < points; i++ {
		theta := kinematics.FullTurn * float64(i) / float64(points-1)
		m, err := b.Point(theta)
		if err != nil {
			return nil, err
		}
		out[i] = CurveSample{Segment: b.locate(theta).Kind, Motion: m}
	}
	return out, nil
}

// Joint 在 Seam 上补充左右所属段信息。
type Joint struct {
	LeftSegment  string `json:"left_segment"`
	RightSegment string `json:"right_segment"`
	kinematics.Seam
}

// seamAt 求某段在其段内角度 theta 处的单侧闭式值。
func (b *Segments) sideMotion(sg *Segment, theta float64, side float64) (kinematics.Motion, error) {
	omega := b.All[0].Spec.Omega
	tLocal := sg.Start / omega
	switch sg.Kind {
	case "outer_dwell":
		return dwellMotion(sg.Spec.H, theta, theta/omega), nil
	case "inner_dwell":
		return dwellMotion(0, theta, theta/omega), nil
	case "rise":
		return kinematics.SampleSide(sg.Spec, sg.Start, theta, tLocal, false, side)
	case "return":
		return kinematics.SampleSide(sg.Spec, sg.Start, theta, tLocal, true, side)
	default:
		return kinematics.Motion{}, fmt.Errorf("未知段类型 %q", sg.Kind)
	}
}

// Discontinuities 核对所有段界接头、等加速段中点加速度变号点，
// 以及 360°/0° 首尾闭合接头，返回逐量连续性结果。
func (b *Segments) Discontinuities() ([]Joint, error) {
	var joints []Joint

	// 段界接头：相邻两段各取自己一侧的闭式值。
	for i := 0; i+1 < len(b.All); i++ {
		left, right := b.All[i], b.All[i+1]
		lv, err := b.sideMotion(left, left.End, +1)
		if err != nil {
			return nil, err
		}
		rv, err := b.sideMotion(right, right.Start, -1)
		if err != nil {
			return nil, err
		}
		joints = append(joints, Joint{
			LeftSegment: left.Kind, RightSegment: right.Kind,
			Seam: kinematics.Join(left.End, "boundary", lv, rv),
		})
	}

	// 规律内部切换点：等加速等减速的 T=0.5。
	for _, sg := range b.All {
		if sg.Law != law.Parabolic {
			continue
		}
		returning := sg.Kind == "return"
		omega := b.All[0].Spec.Omega
		theta := sg.Start + sg.Beta/2
		tm := sg.Start / omega
		lv, err := kinematics.SampleSide(sg.Spec, sg.Start, theta, tm, returning, -1)
		if err != nil {
			return nil, err
		}
		rv, err := kinematics.SampleSide(sg.Spec, sg.Start, theta, tm, returning, +1)
		if err != nil {
			return nil, err
		}
		joints = append(joints, Joint{
			LeftSegment: sg.Kind, RightSegment: sg.Kind,
			Seam: kinematics.Join(theta, "switch", lv, rv),
		})
	}

	// 首尾闭合：360°（最后一段终点）与 0°（第一段起点）必须接上。
	last := b.All[len(b.All)-1]
	first := b.All[0]
	lv, err := b.sideMotion(last, kinematics.FullTurn, +1)
	if err != nil {
		return nil, err
	}
	rv, err := b.sideMotion(first, 0, -1)
	if err != nil {
		return nil, err
	}
	joints = append(joints, Joint{
		LeftSegment: last.Kind, RightSegment: first.Kind,
		Seam: kinematics.Join(kinematics.FullTurn, "cyclic", lv, rv),
	})

	return joints, nil
}

// HasDisplacementJump 报告拼接结果中是否存在任何位移跳变。
func HasDisplacementJump(joints []Joint) bool {
	for _, j := range joints {
		if j.SJump {
			return true
		}
	}
	return false
}
