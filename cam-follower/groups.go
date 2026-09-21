package main

import (
	"errors"
	"fmt"
	"math"
	"net/http"
	"strconv"

	"camfollower/internal/group"
	"camfollower/internal/kinematics"
	"camfollower/internal/store"
)

// ---------- 凸轮组登记与查询 ----------

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

// putGroup 登记一组凸轮：每片点名一份已登记循环档、给一个相位角。
// 相位角允许任意实数，登记前规约到一周以内再落盘。
func putGroup(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var g group.Group
		if !decode(w, r, &g) {
			return
		}
		if err := g.Validate(); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		// 点名的循环档必须都已登记，否则在校验之前拒绝。
		for _, c := range g.Cams {
			if _, err := st.GetCycle(c.Cycle); err != nil {
				if errors.Is(err, store.ErrNotFound) {
					writeErr(w, http.StatusBadRequest,
						fmt.Sprintf("凸轮 %q 点名的循环档 %q 未登记", c.Name, c.Cycle))
					return
				}
				writeErr(w, http.StatusBadRequest, err.Error())
				return
			}
		}
		for i := range g.Cams {
			g.Cams[i].Phase = group.Normalize(g.Cams[i].Phase)
		}
		if err := st.SaveGroup(g); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusCreated, map[string]any{"group": g})
	}
}

func getGroup(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		g, err := st.GetGroup(r.PathValue("name"))
		if err != nil {
			groupStoreError(w, err)
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"group": g})
	}
}

func groupStoreError(w http.ResponseWriter, err error) {
	if errors.Is(err, store.ErrNotFound) {
		writeErr(w, http.StatusNotFound, err.Error())
		return
	}
	writeErr(w, http.StatusBadRequest, err.Error())
}

// mustGroup 取组并校验组内点名的循环档仍然都在（登记后可能被外部移除）。
func mustGroup(w http.ResponseWriter, st *store.Store, name string) (group.Group, bool) {
	g, err := st.GetGroup(name)
	if err != nil {
		groupStoreError(w, err)
		return g, false
	}
	for _, c := range g.Cams {
		if _, err := st.GetCycle(c.Cycle); err != nil {
			if errors.Is(err, store.ErrNotFound) {
				writeErr(w, http.StatusBadRequest,
					fmt.Sprintf("凸轮 %q 点名的循环档 %q 未登记或已被移除", c.Name, c.Cycle))
				return g, false
			}
			writeErr(w, http.StatusBadRequest, err.Error())
			return g, false
		}
	}
	return g, true
}

// movingWindows 算出组内每片凸轮的在动窗口（键为凸轮名）。
func movingWindows(st *store.Store, g group.Group) (map[string][]group.Window, error) {
	perCam := make(map[string][]group.Window, len(g.Cams))
	for _, c := range g.Cams {
		rec, err := st.GetCycle(c.Cycle)
		if err != nil {
			return nil, err
		}
		// ω 与角度拼接无关，借一个正值还原配置。
		ws, err := group.MovingWindows(rec.ToConfig(1), c.Phase)
		if err != nil {
			return nil, err
		}
		perCam[c.Name] = ws
	}
	return perCam, nil
}

// ---------- 各片所处段查询 ----------

// GET /groups/{name}/segments?angle_deg=θ：轴转到基准转角 θ 时各片落在哪一段。
func groupSegments(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		raw := r.URL.Query().Get("angle_deg")
		if raw == "" {
			writeErr(w, http.StatusBadRequest, "必须带 angle_deg 参数（基准转角，度）")
			return
		}
		angle, err := strconv.ParseFloat(raw, 64)
		if err != nil || math.IsNaN(angle) || math.IsInf(angle, 0) {
			writeErr(w, http.StatusBadRequest, "angle_deg 必须是有限实数")
			return
		}
		g, ok := mustGroup(w, st, r.PathValue("name"))
		if !ok {
			return
		}
		hits := make([]group.SegmentHit, 0, len(g.Cams))
		for _, c := range g.Cams {
			rec, _ := st.GetCycle(c.Cycle) // mustGroup 已核对存在
			hit, err := group.SegmentAt(c, rec.ToConfig(1), angle)
			if err != nil {
				writeErr(w, http.StatusBadRequest, err.Error())
				return
			}
			hits = append(hits, hit)
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"group":      g.Name,
			"angle_deg":  group.Normalize(angle),
			"angle_unit": kinematics.AngleUnit,
			"cams":       hits,
		})
	}
}

// ---------- 在动窗口 ----------

type camWindows struct {
	Cam           string         `json:"cam"`
	Cycle         string         `json:"cycle"`
	PhaseDeg      float64        `json:"phase_deg"`
	MovingWindows []group.Window `json:"moving_windows"`
}

// GET /groups/{name}/windows[?cam=x]：某一片或整组在一整周基准转角上的在动窗口。
func groupWindows(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		g, ok := mustGroup(w, st, r.PathValue("name"))
		if !ok {
			return
		}
		perCam, err := movingWindows(st, g)
		if err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		filter := r.URL.Query().Get("cam")
		out := make([]camWindows, 0, len(g.Cams))
		for _, c := range g.Cams {
			if filter != "" && c.Name != filter {
				continue
			}
			out = append(out, camWindows{
				Cam: c.Name, Cycle: c.Cycle, PhaseDeg: c.Phase,
				MovingWindows: perCam[c.Name],
			})
		}
		if filter != "" && len(out) == 0 {
			writeErr(w, http.StatusBadRequest,
				fmt.Sprintf("凸轮 %q 不在组 %q 里", filter, g.Name))
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"group":      g.Name,
			"angle_unit": kinematics.AngleUnit,
			"cams":       out,
		})
	}
}

// ---------- 约束校验 ----------

type checkResponse struct {
	Group      string           `json:"group"`
	Constraint group.Constraint `json:"constraint"`
	group.CheckResult
}

// POST /groups/{name}/constraints：提交一条约束，返回满足或冲突的判断。
// 上限非正数、约束点名的凸轮不在组里等情况在校验之前拒绝。
func checkConstraint(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var con group.Constraint
		if !decode(w, r, &con) {
			return
		}
		g, ok := mustGroup(w, st, r.PathValue("name"))
		if !ok {
			return
		}
		if err := con.Validate(g); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		perCam, err := movingWindows(st, g)
		if err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusOK, checkResponse{
			Group: g.Name, Constraint: con, CheckResult: group.Check(con, perCam),
		})
	}
}
