package main

import (
	"errors"
	"fmt"
	"math"
	"net/http"
	"strconv"

	"camfollower/internal/cycle"
	"camfollower/internal/kinematics"
	"camfollower/internal/phasing"
	"camfollower/internal/store"
)

// ---------- 凸轮组（同轴多片 + 相位） ----------

// groupCamInput 是 POST /groups 中一片凸轮的登记项。
// phase_deg 必须给，允许任意实数（负数、超过一周），登记前规约到 [0,360)。
type groupCamInput struct {
	ID       string   `json:"id"`
	Cycle    string   `json:"cycle"`
	PhaseDeg *float64 `json:"phase_deg"`
}

type groupInput struct {
	Name string          `json:"name"`
	Cams []groupCamInput `json:"cams"`
}

func putGroup(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var in groupInput
		if !decode(w, r, &in) {
			return
		}
		rec := store.GroupRecord{Name: in.Name}
		for i, c := range in.Cams {
			if c.PhaseDeg == nil {
				writeErr(w, http.StatusBadRequest, fmt.Sprintf("第 %d 片凸轮缺少 phase_deg", i+1))
				return
			}
			ph := *c.PhaseDeg
			if math.IsNaN(ph) || math.IsInf(ph, 0) {
				writeErr(w, http.StatusBadRequest, fmt.Sprintf("凸轮 %q 的相位角必须是有限实数", c.ID))
				return
			}
			rec.Cams = append(rec.Cams, store.GroupCam{
				ID: c.ID, Cycle: c.Cycle, PhaseDeg: phasing.Norm360(ph),
			})
		}
		if err := st.SaveGroup(rec); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusCreated, map[string]any{"group": rec})
	}
}

func listGroups(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		gs, err := st.ListGroups()
		if err != nil {
			writeErr(w, http.StatusInternalServerError, err.Error())
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"groups": gs})
	}
}

func getGroup(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		g, err := st.GetGroup(r.PathValue("name"))
		if err != nil {
			groupLoadError(w, err)
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"group": g})
	}
}

// groupLoadError 把组装载/解析错误映射到状态码：
// 组不存在 404；组内点名的循环档未登记等输入问题 400。
func groupLoadError(w http.ResponseWriter, err error) {
	if errors.Is(err, store.ErrNotFound) {
		writeErr(w, http.StatusNotFound, err.Error())
		return
	}
	writeErr(w, http.StatusBadRequest, err.Error())
}

// loadGroupCams 读出凸轮组并把每片解析成可计算的 phasing.Cam 与循环档；
// 组内点名的循环档未登记时，在任何校验/计算之前直接报错拒绝。
func loadGroupCams(st *store.Store, name string) (store.GroupRecord, []phasing.Cam, []store.CycleRecord, error) {
	g, err := st.GetGroup(name)
	if err != nil {
		return g, nil, nil, err
	}
	cams := make([]phasing.Cam, 0, len(g.Cams))
	recs := make([]store.CycleRecord, 0, len(g.Cams))
	for _, gc := range g.Cams {
		rec, err := st.GetCycle(gc.Cycle)
		if err != nil {
			if errors.Is(err, store.ErrNotFound) {
				return g, nil, nil, fmt.Errorf("凸轮 %q 点名的循环档 %q 未登记，请先登记循环档", gc.ID, gc.Cycle)
			}
			return g, nil, nil, err
		}
		recs = append(recs, rec)
		cams = append(cams, phasing.Cam{
			ID: gc.ID, Phase: gc.PhaseDeg,
			RiseAngle: rec.RiseAngle, OuterDwell: rec.OuterDwell,
			ReturnAngle: rec.ReturnAngle, InnerDwell: rec.InnerDwell,
		})
	}
	return g, cams, recs, nil
}

// ---------- 某基准转角下各片所处的段 ----------

type camState struct {
	ID       string  `json:"id"`
	Cycle    string  `json:"cycle"`
	PhaseDeg float64 `json:"phase_deg"`
	LocalDeg float64 `json:"local_deg"` // 本片循环内转角（已按相位折算）
	Segment  string  `json:"segment"`   // rise / outer_dwell / return / inner_dwell
	Active   bool    `json:"active"`    // 升程或回程即在动
}

func groupState(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		g, _, recs, err := loadGroupCams(st, r.PathValue("name"))
		if err != nil {
			groupLoadError(w, err)
			return
		}
		theta, err := strconv.ParseFloat(r.URL.Query().Get("theta_deg"), 64)
		if err != nil || math.IsNaN(theta) || math.IsInf(theta, 0) {
			writeErr(w, http.StatusBadRequest, "theta_deg 必须给定为有限实数（允许任意值，先规约到一周内）")
			return
		}
		theta = phasing.Norm360(theta)
		states := make([]camState, 0, len(g.Cams))
		for i, gc := range g.Cams {
			local := phasing.Norm360(theta - gc.PhaseDeg)
			segs, err := cycle.Build(recs[i].ToConfig(1)) // ω 不影响段界，借正值
			if err != nil {
				writeErr(w, http.StatusInternalServerError, err.Error())
				return
			}
			sg := segs.Locate(local)
			states = append(states, camState{
				ID: gc.ID, Cycle: gc.Cycle, PhaseDeg: gc.PhaseDeg,
				LocalDeg: local, Segment: sg.Kind, Active: cycle.IsMotion(sg.Kind),
			})
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"group": g.Name, "theta_deg": theta,
			"angle_unit": kinematics.AngleUnit, "cams": states,
		})
	}
}

// ---------- 整周基准转角上的在动窗口 ----------

type camWindows struct {
	ID       string            `json:"id"`
	Cycle    string            `json:"cycle"`
	PhaseDeg float64           `json:"phase_deg"`
	Windows  []phasing.ArcView `json:"active_windows"`
}

func groupWindows(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		g, cams, _, err := loadGroupCams(st, r.PathValue("name"))
		if err != nil {
			groupLoadError(w, err)
			return
		}
		want := r.URL.Query().Get("cam")
		out := make([]camWindows, 0, len(cams))
		for i, gc := range g.Cams {
			if want != "" && gc.ID != want {
				continue
			}
			arcs := cams[i].Windows()
			views := make([]phasing.ArcView, 0, len(arcs))
			for _, a := range arcs {
				views = append(views, a.View())
			}
			out = append(out, camWindows{ID: gc.ID, Cycle: gc.Cycle, PhaseDeg: gc.PhaseDeg, Windows: views})
		}
		if want != "" && len(out) == 0 {
			writeErr(w, http.StatusNotFound, fmt.Sprintf("凸轮 %q 不在组 %q 里", want, g.Name))
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"group": g.Name, "angle_unit": kinematics.AngleUnit, "cams": out,
		})
	}
}

// ---------- 约束校验 ----------

// 约束类型：max_concurrent 任意时刻同时在动片数不超过 limit；
// no_overlap 点名的两片在动窗口完全不重叠（等价于这两片上限 1）。
const (
	ctMaxConcurrent = "max_concurrent"
	ctNoOverlap     = "no_overlap"
)

type checkRequest struct {
	Type  string   `json:"type"`
	Cams  []string `json:"cams"`
	Limit int      `json:"limit"`
}

func groupCheck(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var req checkRequest
		if !decode(w, r, &req) {
			return
		}
		g, cams, _, err := loadGroupCams(st, r.PathValue("name"))
		if err != nil {
			groupLoadError(w, err)
			return
		}

		// ---- 校验之前先拒绝非法输入 ----
		if req.Type != ctMaxConcurrent && req.Type != ctNoOverlap {
			writeErr(w, http.StatusBadRequest,
				fmt.Sprintf("未知约束类型 %q，支持 %s / %s", req.Type, ctMaxConcurrent, ctNoOverlap))
			return
		}
		if len(req.Cams) == 0 {
			writeErr(w, http.StatusBadRequest, "约束必须点名至少一片凸轮")
			return
		}
		idx := make(map[string]int, len(g.Cams))
		for i, gc := range g.Cams {
			idx[gc.ID] = i
		}
		seen := map[string]bool{}
		subset := make([]phasing.Cam, 0, len(req.Cams))
		for _, id := range req.Cams {
			if seen[id] {
				writeErr(w, http.StatusBadRequest, fmt.Sprintf("约束重复点名凸轮 %q", id))
				return
			}
			seen[id] = true
			i, ok := idx[id]
			if !ok {
				writeErr(w, http.StatusBadRequest, fmt.Sprintf("约束点名的凸轮 %q 不在组 %q 里", id, g.Name))
				return
			}
			subset = append(subset, cams[i])
		}
		limit := req.Limit
		switch req.Type {
		case ctNoOverlap:
			if len(subset) != 2 {
				writeErr(w, http.StatusBadRequest, "no_overlap 必须恰好点名两片凸轮")
				return
			}
			limit = 1 // 两片完全不重叠 ≡ 这两片任意时刻至多一片在动
		case ctMaxConcurrent:
			if req.Limit <= 0 {
				writeErr(w, http.StatusBadRequest,
					"上限必须为正整数；非正上限意味着任何在动瞬间都违反（必然冲突），按非法输入拒绝")
				return
			}
		}

		res := phasing.Sweep(subset, limit)
		writeJSON(w, http.StatusOK, map[string]any{
			"group": g.Name, "type": req.Type, "cams": req.Cams, "limit": limit,
			"satisfied":    len(res.Exceed) == 0,
			"peak_count":   res.Peak,
			"peak_windows": regionViews(res.PeakRegions),
			"conflicts":    regionViews(res.Exceed),
			"angle_unit":   kinematics.AngleUnit,
		})
	}
}

func regionViews(rs []phasing.Region) []phasing.RegionView {
	out := make([]phasing.RegionView, 0, len(rs))
	for _, r := range rs {
		out = append(out, r.View())
	}
	return out
}
