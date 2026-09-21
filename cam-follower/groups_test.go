package main

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"camfollower/internal/law"
)

func mustCycle(t *testing.T, srv *httptest.Server, rec map[string]any) {
	t.Helper()
	if code, body := postJSON(t, srv, "/cycles", rec); code != http.StatusCreated {
		t.Fatalf("登记循环档失败 %d %v", code, body)
	}
}

func mustGroup(t *testing.T, srv *httptest.Server, name string, cams ...map[string]any) {
	t.Helper()
	if code, body := postJSON(t, srv, "/groups", map[string]any{"name": name, "cams": cams}); code != http.StatusCreated {
		t.Fatalf("登记凸轮组失败 %d %v", code, body)
	}
}

// camRef 造一片凸轮登记项。
func camRef(id, cycle string, phase float64) map[string]any {
	return map[string]any{"id": id, "cycle": cycle, "phase_deg": phase}
}

// windowsOf 取某组（或某片）的在动窗口响应。
func windowsOf(t *testing.T, srv *httptest.Server, path string) []any {
	t.Helper()
	code, body := getJSON(t, srv, path)
	if code != 200 {
		t.Fatalf("取在动窗口 %s 状态 %d: %v", path, code, body)
	}
	return body["cams"].([]any)
}

// 单片凸轮：升程接回程（远休止角 0）并成一条窗口，不会判成和自己重叠。
func TestHTTPGroupSingleCamNoSelfOverlap(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "z1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 100, "outer_dwell_deg": 0,
		"return_angle_deg": 50, "inner_dwell_deg": 210,
	})
	mustGroup(t, srv, "g1", camRef("c1", "z1", 0))

	cams := windowsOf(t, srv, "/groups/g1/windows")
	ws := cams[0].(map[string]any)["active_windows"].([]any)
	if len(ws) != 1 {
		t.Fatalf("远休止为 0 时升程/回程应并成 1 条窗口，得到 %d: %v", len(ws), ws)
	}
	w0 := ws[0].(map[string]any)
	if w0["start_deg"].(float64) != 0 || w0["end_deg"].(float64) != 150 ||
		w0["span_deg"].(float64) != 150 || w0["wraps"].(bool) {
		t.Fatalf("窗口应为 [0,150) 不回卷，得到 %v", w0)
	}

	code, body := postJSON(t, srv, "/groups/g1/check", map[string]any{
		"type": "max_concurrent", "cams": []string{"c1"}, "limit": 1,
	})
	if code != 200 {
		t.Fatalf("校验状态 %d %v", code, body)
	}
	if !body["satisfied"].(bool) || body["peak_count"].(float64) != 1 {
		t.Fatalf("单片上限 1 必须满足且峰值为 1: %v", body)
	}
	if n := len(body["conflicts"].([]any)); n != 0 {
		t.Fatalf("单片不得与自己冲突，得到 %d 段冲突", n)
	}
}

// 两片刻意错开半周、窗口完全不重叠：两两不重叠约束通过。
func TestHTTPGroupHalfTurnApartNoOverlap(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "w1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	})
	mustGroup(t, srv, "g2", camRef("c1", "w1", 0), camRef("c2", "w1", 180))

	code, body := postJSON(t, srv, "/groups/g2/check", map[string]any{
		"type": "no_overlap", "cams": []string{"c1", "c2"},
	})
	if code != 200 || !body["satisfied"].(bool) {
		t.Fatalf("错开半周应满足不重叠约束: %d %v", code, body)
	}
	if n := len(body["conflicts"].([]any)); n != 0 {
		t.Fatalf("不应有冲突区间，得到 %v", body["conflicts"])
	}
	// 同一条约束用上限形式表达也应通过，峰值 1
	code, body = postJSON(t, srv, "/groups/g2/check", map[string]any{
		"type": "max_concurrent", "cams": []string{"c1", "c2"}, "limit": 1,
	})
	if code != 200 || !body["satisfied"].(bool) || body["peak_count"].(float64) != 1 {
		t.Fatalf("上限 1 应满足且峰值 1: %d %v", code, body)
	}
}

// 把其中一片相位挪到窗口部分交叠：报冲突并给出正确区间。
func TestHTTPGroupOverlapConflict(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "w1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	})
	// c1 在动 [0,60)∪[90,180)；c2 相位 150° → [150,210)∪[240,330)
	mustGroup(t, srv, "g3", camRef("c1", "w1", 0), camRef("c2", "w1", 150))

	code, body := postJSON(t, srv, "/groups/g3/check", map[string]any{
		"type": "no_overlap", "cams": []string{"c1", "c2"},
	})
	if code != 200 {
		t.Fatalf("校验状态 %d %v", code, body)
	}
	if body["satisfied"].(bool) {
		t.Fatal("窗口部分交叠必须报冲突")
	}
	cf := body["conflicts"].([]any)
	if len(cf) != 1 {
		t.Fatalf("冲突区间应恰为 1 段，得到 %v", cf)
	}
	c0 := cf[0].(map[string]any)
	if c0["start_deg"].(float64) != 150 || c0["end_deg"].(float64) != 180 ||
		c0["span_deg"].(float64) != 30 || c0["wraps"].(bool) {
		t.Fatalf("冲突区间应为 [150,180)，得到 %v", c0)
	}
	cams := c0["cams"].([]any)
	if len(cams) != 2 || cams[0] != "c1" || cams[1] != "c2" {
		t.Fatalf("冲突牵涉 c1、c2，得到 %v", cams)
	}
	if body["peak_count"].(float64) != 2 {
		t.Fatalf("峰值应为 2，得到 %v", body["peak_count"])
	}
}

// 三片各占三分之一周期错开：任意时刻至多一片在动。
// 上限 1 通过；上限 0 为非正上限，在校验前拒绝（数学上必然冲突）。
func TestHTTPGroupThreeCamsTiling(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "t1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 0,
		"return_angle_deg": 60, "inner_dwell_deg": 240,
	})
	mustGroup(t, srv, "g4",
		camRef("c1", "t1", 0), camRef("c2", "t1", 120), camRef("c3", "t1", 240))

	code, body := postJSON(t, srv, "/groups/g4/check", map[string]any{
		"type": "max_concurrent", "cams": []string{"c1", "c2", "c3"}, "limit": 1,
	})
	if code != 200 || !body["satisfied"].(bool) {
		t.Fatalf("三片错开 1/3 周期，上限 1 应通过: %d %v", code, body)
	}
	if body["peak_count"].(float64) != 1 {
		t.Fatalf("峰值应为 1，得到 %v", body["peak_count"])
	}
	// 峰值 1 铺满整周
	pw := body["peak_windows"].([]any)
	if len(pw) != 1 || pw[0].(map[string]any)["span_deg"].(float64) != 360 {
		t.Fatalf("峰值区间应铺满整周，得到 %v", pw)
	}
	// 上限 0 / 负数：校验前拒绝
	for _, lim := range []int{0, -2} {
		code, body = postJSON(t, srv, "/groups/g4/check", map[string]any{
			"type": "max_concurrent", "cams": []string{"c1", "c2", "c3"}, "limit": lim,
		})
		if code != http.StatusBadRequest || body["error"] == nil {
			t.Fatalf("上限 %d 应 400 并说明原因，得到 %d %v", lim, code, body)
		}
	}
}

// 停歇段角度为 0：在动窗口不该因此断开或多算一段。
func TestHTTPGroupZeroDwellWindows(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{ // 远休止 0：升程接回程
		"name": "z2", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 45, "outer_dwell_deg": 0,
		"return_angle_deg": 75, "inner_dwell_deg": 240,
	})
	mustCycle(t, srv, map[string]any{ // 近休止 0：回程尾接下周升程头，跨零一段
		"name": "z3", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 45, "outer_dwell_deg": 30,
		"return_angle_deg": 285, "inner_dwell_deg": 0,
	})
	mustGroup(t, srv, "g5", camRef("c1", "z2", 0), camRef("c2", "z3", 0))

	cams := windowsOf(t, srv, "/groups/g5/windows")
	w1 := cams[0].(map[string]any)["active_windows"].([]any)
	if len(w1) != 1 || w1[0].(map[string]any)["span_deg"].(float64) != 120 {
		t.Fatalf("远休止 0 应并成 1 条 120° 窗口，得到 %v", w1)
	}
	w2 := cams[1].(map[string]any)["active_windows"].([]any)
	if len(w2) != 1 {
		t.Fatalf("近休止 0 应并成跨零 1 条窗口，得到 %v", w2)
	}
	w := w2[0].(map[string]any)
	if w["start_deg"].(float64) != 75 || w["end_deg"].(float64) != 45 ||
		w["span_deg"].(float64) != 330 || !w["wraps"].(bool) {
		t.Fatalf("跨零窗口应为 75°→45° 回卷 330°，得到 %v", w)
	}
}

// 相位角给负数或大于 360 的值：与规约到一周内的等效相位完全一致。
func TestHTTPGroupPhaseNormalization(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "w1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	})
	mustGroup(t, srv, "g6a", camRef("c1", "w1", -90))
	mustGroup(t, srv, "g6b", camRef("c1", "w1", 270))
	mustGroup(t, srv, "g6c", camRef("c1", "w1", 450))
	mustGroup(t, srv, "g6d", camRef("c1", "w1", -450))

	// 登记后相位已规约到 [0,360)
	code, body := getJSON(t, srv, "/groups/g6a")
	if code != 200 {
		t.Fatalf("读组 %d", code)
	}
	ph := body["group"].(map[string]any)["cams"].([]any)[0].(map[string]any)["phase_deg"].(float64)
	if ph != 270 {
		t.Fatalf("-90° 应规约为 270°，得到 %v", ph)
	}
	_, body = getJSON(t, srv, "/groups/g6c")
	ph = body["group"].(map[string]any)["cams"].([]any)[0].(map[string]any)["phase_deg"].(float64)
	if ph != 90 {
		t.Fatalf("450° 应规约为 90°，得到 %v", ph)
	}

	// 窗口与状态：负相位/超周相位与等效相位完全一致
	eq := func(path string) {
		t.Helper()
		_, b1 := getJSON(t, srv, "/groups/g6a"+path)
		_, b2 := getJSON(t, srv, "/groups/g6b"+path)
		_, b4 := getJSON(t, srv, "/groups/g6d"+path)
		j1, _ := json.Marshal(b1["cams"])
		j2, _ := json.Marshal(b2["cams"])
		j4, _ := json.Marshal(b4["cams"])
		if string(j1) != string(j2) || string(j1) != string(j4) {
			t.Fatalf("%s：-90°/270°/-450° 结果必须一致\n%s\n%s\n%s", path, j1, j2, j4)
		}
	}
	eq("/windows")
	eq("/state?theta_deg=30")
	eq("/state?theta_deg=123.5")
}

// 非法输入在校验之前拒绝并说清原因。
func TestHTTPGroupRejections(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "w1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	})
	mustGroup(t, srv, "g7", camRef("c1", "w1", 0), camRef("c2", "w1", 90))

	bad := []struct {
		name string
		code int
		call func() (int, map[string]any)
	}{
		{"组内点名未登记循环档", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups", map[string]any{
				"name": "gx", "cams": []map[string]any{camRef("c1", "nope", 0)}})
		}},
		{"缺相位角", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups", map[string]any{
				"name": "gx", "cams": []map[string]any{{"id": "c1", "cycle": "w1"}}})
		}},
		{"凸轮 id 重复", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups", map[string]any{
				"name": "gx", "cams": []map[string]any{camRef("c1", "w1", 0), camRef("c1", "w1", 10)}})
		}},
		{"约束点名组外凸轮", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/g7/check", map[string]any{
				"type": "no_overlap", "cams": []string{"c1", "c9"}})
		}},
		{"未知约束类型", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/g7/check", map[string]any{
				"type": "mutex", "cams": []string{"c1", "c2"}})
		}},
		{"no_overlap 只点一片", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/g7/check", map[string]any{
				"type": "no_overlap", "cams": []string{"c1"}})
		}},
		{"约束重复点名", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/g7/check", map[string]any{
				"type": "max_concurrent", "cams": []string{"c1", "c1"}, "limit": 1})
		}},
		{"约束不点凸轮", 400, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/g7/check", map[string]any{
				"type": "max_concurrent", "cams": []string{}, "limit": 1})
		}},
		{"组不存在", 404, func() (int, map[string]any) {
			return getJSON(t, srv, "/groups/nope")
		}},
		{"组不存在的状态查询", 404, func() (int, map[string]any) {
			return getJSON(t, srv, "/groups/nope/state?theta_deg=10")
		}},
		{"组不存在的校验", 404, func() (int, map[string]any) {
			return postJSON(t, srv, "/groups/nope/check", map[string]any{
				"type": "max_concurrent", "cams": []string{"c1"}, "limit": 1})
		}},
		{"窗口查询点名组外凸轮", 404, func() (int, map[string]any) {
			return getJSON(t, srv, "/groups/g7/windows?cam=c9")
		}},
		{"状态查询缺转角", 400, func() (int, map[string]any) {
			return getJSON(t, srv, "/groups/g7/state")
		}},
	}
	for _, c := range bad {
		code, body := c.call()
		if code != c.code || body["error"] == nil {
			t.Fatalf("%s：应 %d 且带原因，得到 %d %v", c.name, c.code, code, body)
		}
	}
}

// 某基准转角下各片所处的段：相位折算、跨零回卷、段界归属。
func TestHTTPGroupState(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "w1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 60, "outer_dwell_deg": 30,
		"return_angle_deg": 90, "inner_dwell_deg": 180,
	})
	mustGroup(t, srv, "g8", camRef("c1", "w1", 350))

	stateAt := func(theta string) map[string]any {
		t.Helper()
		code, body := getJSON(t, srv, "/groups/g8/state?theta_deg="+theta)
		if code != 200 {
			t.Fatalf("状态查询 %s 状态 %d: %v", theta, code, body)
		}
		return body["cams"].([]any)[0].(map[string]any)
	}
	cases := []struct {
		theta   string
		local   float64
		segment string
		active  bool
	}{
		{"20", 30, "rise", true},           // 20-350 回卷到 30
		{"100", 110, "return", true},       // 回程 [90,180)
		{"300", 310, "inner_dwell", false}, // 近休止
		{"50", 60, "outer_dwell", false},   // 段界归停歇段
		{"400", 50, "rise", true},          // 基准转角先规约 400→40
		{"-340", 30, "rise", true},         // 负基准转角规约到 20
	}
	for _, c := range cases {
		st := stateAt(c.theta)
		if st["local_deg"].(float64) != c.local || st["segment"] != c.segment || st["active"].(bool) != c.active {
			t.Fatalf("θ=%s 应为局部 %v° %s active=%v，得到 %v",
				c.theta, c.local, c.segment, c.active, st)
		}
	}
}

// 三片两两交叠都不超上限 2，但某一瞬间三片叠在一起：
// 两两比较不够，峰值必须由区间扫描给出。
func TestHTTPGroupTripleOverlapCaught(t *testing.T) {
	srv, _ := newTestServer(t)
	defer srv.Close()
	mustCycle(t, srv, map[string]any{
		"name": "z1", "h": 10, "rise_law": law.Cycloid, "return_law": law.Cosine,
		"rise_angle_deg": 100, "outer_dwell_deg": 0,
		"return_angle_deg": 50, "inner_dwell_deg": 210,
	})
	// 在动窗口各 150°：c1 [0,150) c2 [60,210) c3 [120,270)
	mustGroup(t, srv, "g9",
		camRef("c1", "z1", 0), camRef("c2", "z1", 60), camRef("c3", "z1", 120))

	code, body := postJSON(t, srv, "/groups/g9/check", map[string]any{
		"type": "max_concurrent", "cams": []string{"c1", "c2", "c3"}, "limit": 2,
	})
	if code != 200 {
		t.Fatalf("校验状态 %d %v", code, body)
	}
	if body["satisfied"].(bool) {
		t.Fatal("三片在 [120,150) 叠在一起，上限 2 必须报冲突")
	}
	if body["peak_count"].(float64) != 3 {
		t.Fatalf("峰值应为 3，得到 %v", body["peak_count"])
	}
	cf := body["conflicts"].([]any)
	if len(cf) != 1 {
		t.Fatalf("冲突区间应恰 1 段，得到 %v", cf)
	}
	c0 := cf[0].(map[string]any)
	if c0["start_deg"].(float64) != 120 || c0["end_deg"].(float64) != 150 ||
		c0["peak_count"].(float64) != 3 || len(c0["cams"].([]any)) != 3 {
		t.Fatalf("冲突应为 [120,150) 牵涉三片峰值 3，得到 %v", c0)
	}
	// 上限放宽到 3 则通过
	code, body = postJSON(t, srv, "/groups/g9/check", map[string]any{
		"type": "max_concurrent", "cams": []string{"c1", "c2", "c3"}, "limit": 3,
	})
	if code != 200 || !body["satisfied"].(bool) {
		t.Fatalf("上限 3 应通过: %d %v", code, body)
	}
}
