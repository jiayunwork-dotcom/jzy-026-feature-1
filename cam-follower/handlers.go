package main

import (
	"encoding/json"
	"errors"
	"net/http"
	"strconv"

	"camfollower/internal/cycle"
	"camfollower/internal/kinematics"
	"camfollower/internal/law"
	"camfollower/internal/store"
)

func routes(st *store.Store) http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET /healthz", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
	})
	mux.HandleFunc("GET /recipes", listRecipes(st))
	mux.HandleFunc("POST /recipes", putRecipe(st))
	mux.HandleFunc("GET /recipes/{name}", getRecipe(st))
	mux.HandleFunc("GET /cycles", listCycles(st))
	mux.HandleFunc("POST /cycles", putCycle(st))
	mux.HandleFunc("GET /cycles/{name}", getCycle(st))
	mux.HandleFunc("GET /lift", liftCurveGET(st))
	mux.HandleFunc("POST /lift", liftCurvePOST(st))
	mux.HandleFunc("GET /cycle-curve", cycleCurveGET(st))
	mux.HandleFunc("POST /cycle-curve", cycleCurvePOST(st))
	return mux
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}

func writeErr(w http.ResponseWriter, code int, msg string) {
	writeJSON(w, code, map[string]any{"error": msg, "status": code})
}

func decode(w http.ResponseWriter, r *http.Request, v any) bool {
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(v); err != nil {
		writeErr(w, http.StatusBadRequest, "请求体不是合法 JSON 或含未知字段: "+err.Error())
		return false
	}
	return true
}

// ---------- 配方 ----------

func listRecipes(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		rs, err := st.ListRecipes()
		if err != nil {
			writeErr(w, http.StatusInternalServerError, err.Error())
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"recipes": rs})
	}
}

func putRecipe(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var rec store.Recipe
		if !decode(w, r, &rec) {
			return
		}
		if err := rec.Validate(); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		if err := st.SaveRecipe(rec); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusCreated, map[string]any{"recipe": rec})
	}
}

func getRecipe(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		rec, err := st.GetRecipe(r.PathValue("name"))
		if err != nil {
			recipeStoreError(w, err)
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"recipe": rec})
	}
}

func recipeStoreError(w http.ResponseWriter, err error) {
	if errors.Is(err, store.ErrNotFound) {
		writeErr(w, http.StatusNotFound, err.Error())
		return
	}
	writeErr(w, http.StatusBadRequest, err.Error())
}

// ---------- 循环档 ----------

func listCycles(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		cs, err := st.ListCycles()
		if err != nil {
			writeErr(w, http.StatusInternalServerError, err.Error())
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"cycles": cs})
	}
}

func putCycle(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var rec store.CycleRecord
		if !decode(w, r, &rec) {
			return
		}
		if err := rec.Validate(); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		if err := st.SaveCycle(rec); err != nil {
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusCreated, map[string]any{"cycle": rec})
	}
}

func getCycle(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		rec, err := st.GetCycle(r.PathValue("name"))
		if err != nil {
			if errors.Is(err, store.ErrNotFound) {
				writeErr(w, http.StatusNotFound, err.Error())
				return
			}
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"cycle": rec})
	}
}

// ---------- 升程曲线 ----------

// liftRequest 是 POST /lift 的请求体。
// recipe 非空时按已登记配方取类型/h/β，body 内不得再重复给这三项；
// 否则必须给全 type、h、beta_deg 做临时计算。
type liftRequest struct {
	Recipe string   `json:"recipe"`
	Type   *string  `json:"type"`
	H      *float64 `json:"h"`
	Beta   *float64 `json:"beta_deg"`
	Omega  float64  `json:"omega_deg_s"`
	Points int      `json:"points"`
}

type liftResponse struct {
	Law       string              `json:"law"`
	H         float64             `json:"h"`
	BetaDeg   float64             `json:"beta_deg"`
	OmegaDegS float64             `json:"omega_deg_s"`
	AngleUnit string              `json:"angle_unit"`
	OmegaUnit string              `json:"omega_unit"`
	Points    []kinematics.Motion `json:"points"`
	Peaks     kinematics.Peaks    `json:"closed_form_peaks"`
	Seams     []kinematics.Seam   `json:"internal_seams"`
}

func resolveSpec(st *store.Store, req liftRequest) (kinematics.Spec, error) {
	spec := kinematics.Spec{Omega: req.Omega}
	if req.Recipe != "" {
		if req.Type != nil || req.H != nil || req.Beta != nil {
			return spec, errors.New("已指定 recipe 时不得再在请求体中重给 type/h/beta_deg（以配方为准）")
		}
		rec, err := st.GetRecipe(req.Recipe)
		if err != nil {
			return spec, err
		}
		spec.Type, spec.H, spec.Beta = rec.Type, rec.H, rec.Beta
		return spec, nil
	}
	if req.Type == nil || req.H == nil || req.Beta == nil {
		return spec, errors.New("必须指定 recipe，或给全 type、h、beta_deg")
	}
	spec.Type, spec.H, spec.Beta = *req.Type, *req.H, *req.Beta
	return spec, nil
}

func liftCurveGET(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		q := r.URL.Query()
		name := q.Get("recipe")
		if name == "" {
			writeErr(w, http.StatusBadRequest, "GET /lift 必须带 recipe 参数；临时计算请用 POST /lift")
			return
		}
		omega, err := strconv.ParseFloat(q.Get("omega_deg_s"), 64)
		if err != nil || omega <= 0 {
			writeErr(w, http.StatusBadRequest, "omega_deg_s 必须为正数")
			return
		}
		points := kinematics.DefaultPoints
		if v := q.Get("points"); v != "" {
			points, err = strconv.Atoi(v)
			if err != nil {
				writeErr(w, http.StatusBadRequest, "points 必须是整数")
				return
			}
		}
		serveLift(w, st, liftRequest{Recipe: name, Omega: omega, Points: points})
	}
}

func liftCurvePOST(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var req liftRequest
		if !decode(w, r, &req) {
			return
		}
		if req.Points == 0 {
			req.Points = kinematics.DefaultPoints
		}
		serveLift(w, st, req)
	}
}

func serveLift(w http.ResponseWriter, st *store.Store, req liftRequest) {
	spec, err := resolveSpec(st, req)
	if err != nil {
		if errors.Is(err, store.ErrNotFound) {
			writeErr(w, http.StatusNotFound, err.Error())
			return
		}
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	if err := spec.Validate(); err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	if req.Points < kinematics.MinPoints {
		writeErr(w, http.StatusBadRequest, "points 至少为 2")
		return
	}
	pts, peaks, err := kinematics.Curve(spec, req.Points)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	seams, err := kinematics.InternalSeams(spec, 0)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	writeJSON(w, http.StatusOK, liftResponse{
		Law: spec.Type, H: spec.H, BetaDeg: spec.Beta, OmegaDegS: spec.Omega,
		AngleUnit: kinematics.AngleUnit, OmegaUnit: kinematics.OmegaUnit,
		Points: pts, Peaks: peaks, Seams: seams,
	})
}

// ---------- 整周循环曲线 ----------

// cycleCurveRequest 中 cycle 非空时按已登记循环档取四段配置；
// 否则 body 必须给全四段角度、规律、h，做临时计算。
type cycleCurveRequest struct {
	Cycle string `json:"cycle"`
	cycle.Config
	Points int `json:"points"`
}

type cycleCurveResponse struct {
	H                float64             `json:"h"`
	OmegaDegS        float64             `json:"omega_deg_s"`
	AngleUnit        string              `json:"angle_unit"`
	OmegaUnit        string              `json:"omega_unit"`
	TotalAngleDeg    float64             `json:"total_angle_deg"`
	Segments         []*cycle.Segment    `json:"segments"`
	Points           []cycle.CurveSample `json:"points"`
	Joints           []cycle.Joint       `json:"joints"`
	DisplacementJump bool                `json:"displacement_jump"`
}

func cycleCurveGET(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		q := r.URL.Query()
		name := q.Get("cycle")
		if name == "" {
			writeErr(w, http.StatusBadRequest, "GET /cycle-curve 必须带 cycle 参数；临时计算请用 POST /cycle-curve")
			return
		}
		omega, err := strconv.ParseFloat(q.Get("omega_deg_s"), 64)
		if err != nil || omega <= 0 {
			writeErr(w, http.StatusBadRequest, "omega_deg_s 必须为正数")
			return
		}
		points := kinematics.DefaultPoints
		if v := q.Get("points"); v != "" {
			points, err = strconv.Atoi(v)
			if err != nil {
				writeErr(w, http.StatusBadRequest, "points 必须是整数")
				return
			}
		}
		rec, err := st.GetCycle(name)
		if err != nil {
			if errors.Is(err, store.ErrNotFound) {
				writeErr(w, http.StatusNotFound, err.Error())
				return
			}
			writeErr(w, http.StatusBadRequest, err.Error())
			return
		}
		serveCycle(w, rec.ToConfig(omega), points)
	}
}

func cycleCurvePOST(st *store.Store) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		var req cycleCurveRequest
		if !decode(w, r, &req) {
			return
		}
		if req.Points == 0 {
			req.Points = kinematics.DefaultPoints
		}
		cfg := req.Config
		if req.Cycle != "" {
			rec, err := st.GetCycle(req.Cycle)
			if err != nil {
				if errors.Is(err, store.ErrNotFound) {
					writeErr(w, http.StatusNotFound, err.Error())
					return
				}
				writeErr(w, http.StatusBadRequest, err.Error())
				return
			}
			cfg = rec.ToConfig(req.Omega)
		}
		serveCycle(w, cfg, req.Points)
	}
}

func serveCycle(w http.ResponseWriter, cfg cycle.Config, points int) {
	if err := cfg.Validate(); err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	if points < kinematics.MinPoints {
		writeErr(w, http.StatusBadRequest, "points 至少为 2")
		return
	}
	// 未知规律名在求曲线前直接拒绝（Validate 内已覆盖，这里显式引用 law 保持语义清晰）。
	if !law.Known(cfg.RiseLaw) || !law.Known(cfg.ReturnLaw) {
		writeErr(w, http.StatusBadRequest, law.ErrUnknownLaw.Error())
		return
	}
	segs, err := cycle.Build(cfg)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	pts, err := segs.Curve(points)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	joints, err := segs.Discontinuities()
	if err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, http.StatusOK, cycleCurveResponse{
		H: cfg.H, OmegaDegS: cfg.Omega,
		AngleUnit: kinematics.AngleUnit, OmegaUnit: kinematics.OmegaUnit,
		TotalAngleDeg:    segs.TotalAngle,
		Segments:         segs.All,
		Points:           pts,
		Joints:           joints,
		DisplacementJump: cycle.HasDisplacementJump(joints),
	})
}
