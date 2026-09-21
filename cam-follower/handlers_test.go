package main

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"camfollower/internal/kinematics"
	"camfollower/internal/law"
	"camfollower/internal/store"
)

func newTestServer(t *testing.T) (*httptest.Server, *store.Store) {
	t.Helper()
	st, err := store.Open(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	if err := seed(st); err != nil {
		t.Fatal(err)
	}
	return httptest.NewServer(routes(st)), st
}

func postJSON(t *testing.T, srv *httptest.Server, path string, body any) (int, map[string]any) {
	t.Helper()
	b, _ := json.Marshal(body)
	resp, err := http.Post(srv.URL+path, "application/json", bytes.NewReader(b))
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var m map[string]any
	_ = json.NewDecoder(resp.Body).Decode(&m)
	return resp.StatusCode, m
}

func getJSON(t *testing.T, srv *httptest.Server, path string) (int, map[string]any) {
	t.Helper()
	resp, err := http.Get(srv.URL + path)
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var m map[string]any
	_ = json.NewDecoder(resp.Body).Decode(&m)
	return resp.StatusCode, m
}

// 启动即有摆线升程配方，可直接点名求升程曲线；终点 s=h。
func TestHTTPSeededCycloidLift(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()

	code, body := getJSON(t, srv, "/recipes")
	if code != 200 {
		t.Fatalf("列配方状态 %d", code)
	}
	recipes := body["recipes"].([]any)
	if len(recipes) != 1 {
		t.Fatalf("启动应载入 1 条配方，得到 %d", len(recipes))
	}

	code, body = getJSON(t, srv, "/lift?recipe=cycloid-lift&omega_deg_s=360")
	if code != 200 {
		t.Fatalf("求升程曲线状态 %d: %v", code, body)
	}
	if body["angle_unit"] != kinematics.AngleUnit {
		t.Fatalf("结果必须写明角度单位，得到 %v", body["angle_unit"])
	}
	pts := body["points"].([]any)
	last := pts[len(pts)-1].(map[string]any)
	if d := last["s"].(float64) - 10; d > 1e-6 || d < -1e-6 {
		t.Fatalf("升程终点 s 必须等于 h=10，得到 %v", last["s"])
	}
	first := pts[0].(map[string]any)
	if first["s"].(float64) != 0 {
		t.Fatalf("升程起点 s 必须为 0，得到 %v", first["s"])
	}
	// 闭式峰值应出现在响应中
	peaks := body["closed_form_peaks"].(map[string]any)
	if peaks["a_max"].(float64) <= 0 || peaks["j_max"].(float64) <= 0 {
		t.Fatalf("摆线峰值必须为正: %+v", peaks)
	}
}

// 未知规律名在求曲线前直接拒绝。
func TestHTTPUnknownLawRejected(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	code, body := postJSON(t, srv, "/lift", map[string]any{
		"type": "7th-degree-polynomial", "h": 10, "beta_deg": 60, "omega_deg_s": 360,
	})
	if code != http.StatusBadRequest {
		t.Fatalf("未知规律应 400，得到 %d", code)
	}
	if body["error"] == nil {
		t.Fatal("应给出拒绝原因")
	}
	// 未知配方名
	code, _ = getJSON(t, srv, "/lift?recipe=nope&omega_deg_s=360")
	if code != http.StatusNotFound {
		t.Fatalf("未知配方应 404，得到 %d", code)
	}
}

// 缺项、β 越界拒绝并说明原因。
func TestHTTPBadParamsRejected(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	cases := []map[string]any{
		{"type": law.Cycloid, "beta_deg": 60, "omega_deg_s": 360},         // 缺 h
		{"type": law.Cycloid, "h": 10, "beta_deg": 60},                    // 缺 ω
		{"type": law.Cycloid, "h": 10, "beta_deg": 360, "omega_deg_s": 1}, // β 越界
		{"type": law.Cycloid, "h": -1, "beta_deg": 60, "omega_deg_s": 1},  // h 非法
		{"recipe": "cycloid-lift", "h": 5, "omega_deg_s": 1},              // 配方与重复参数
	}
	for i, c := range cases {
		code, body := postJSON(t, srv, "/lift", c)
		if code != http.StatusBadRequest || body["error"] == nil {
			t.Fatalf("用例 #%d 应 400 且带原因，得到 %d %v", i, code, body)
		}
	}
}

// 整周循环：四段拼一周，接头位移连续，停歇段导数为 0。
func TestHTTPFullCycle(t *testing.T) {
	srv, st := newTestServer(t)
	defer srv.Close()

	// 登记循环档
	rec := map[string]any{
		"name": "week1", "h": 10,
		"rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	}
	if code, body := postJSON(t, srv, "/cycles", rec); code != http.StatusCreated {
		t.Fatalf("登记循环档失败 %d %v", code, body)
	}
	code, list := getJSON(t, srv, "/cycles")
	if code != 200 || len(list["cycles"].([]any)) != 1 {
		t.Fatal("循环档列表应有 1 条")
	}

	code, body := getJSON(t, srv, "/cycle-curve?cycle=week1&omega_deg_s=360")
	if code != 200 {
		t.Fatalf("求循环曲线 %d %v", code, body)
	}
	if body["displacement_jump"].(bool) {
		t.Fatal("整周循环不得有位移跳变")
	}
	for _, j0 := range body["joints"].([]any) {
		j := j0.(map[string]any)
		if !j["s_continuous"].(bool) {
			t.Fatalf("接头 %v 位移不连续", j["theta_deg"])
		}
	}

	// 等加速等减速：临时 POST 循环，中点变号接头应在结果里。
	code, body = postJSON(t, srv, "/cycle-curve", map[string]any{
		"h": 10, "rise_law": law.Parabolic, "return_law": law.Parabolic,
		"rise_angle_deg": 120, "outer_dwell_deg": 0,
		"return_angle_deg": 120, "inner_dwell_deg": 120,
		"omega_deg_s": 360, "points": 361,
	})
	if code != 200 {
		t.Fatalf("等加速循环 %d %v", code, body)
	}
	if body["displacement_jump"].(bool) {
		t.Fatal("等加速循环位移仍必须连续")
	}
	var switches int
	for _, j0 := range body["joints"].([]any) {
		j := j0.(map[string]any)
		if j["kind"] == "switch" {
			switches++
			if j["a_continuous"].(bool) {
				t.Fatal("等加速中点加速度不应连续")
			}
		}
	}
	if switches != 2 {
		t.Fatalf("升程/回程各一个变号点，得到 %d", switches)
	}

	// 角度和不为一周必须拒绝
	code, _ = postJSON(t, srv, "/cycle-curve", map[string]any{
		"h": 10, "rise_law": law.Cycloid, "return_law": law.Cycloid,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 170,
		"omega_deg_s": 360,
	})
	if code != http.StatusBadRequest {
		t.Fatalf("角度和 350 应 400，得到 %d", code)
	}

	_ = st
}

// 配方登记/读取往返。
func TestHTTPRecipeCRUD(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	code, body := postJSON(t, srv, "/recipes", map[string]any{
		"name": "harmonic-1", "type": law.Cosine, "h": 5, "beta_deg": 75,
	})
	if code != http.StatusCreated {
		t.Fatalf("登记配方 %d %v", code, body)
	}
	code, body = getJSON(t, srv, "/recipes/harmonic-1")
	if code != 200 {
		t.Fatalf("读配方 %d", code)
	}
	r := body["recipe"].(map[string]any)
	if r["type"] != law.Cosine || r["h"].(float64) != 5 || r["beta_deg"].(float64) != 75 {
		t.Fatalf("配方读回内容错误: %+v", r)
	}
}
