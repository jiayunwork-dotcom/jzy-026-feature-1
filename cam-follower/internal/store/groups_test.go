package store

import (
	"errors"
	"testing"

	"camfollower/internal/law"
)

func saveTestCycle(t *testing.T, st *Store, name string) {
	t.Helper()
	rec := CycleRecord{
		Name: name, H: 10, RiseLaw: law.Cycloid, ReturnLaw: law.Cosine,
		RiseAngle: 60, OuterDwell: 30, ReturnAngle: 90, InnerDwell: 180,
	}
	if err := st.SaveCycle(rec); err != nil {
		t.Fatal(err)
	}
}

// 凸轮组登记/读取/列出往返。
func TestGroupSaveGetList(t *testing.T) {
	st := newTestStore(t)
	saveTestCycle(t, st, "w1")
	g := GroupRecord{Name: "shaft-1", Cams: []GroupCam{
		{ID: "c1", Cycle: "w1", PhaseDeg: 0},
		{ID: "c2", Cycle: "w1", PhaseDeg: 270},
	}}
	if err := st.SaveGroup(g); err != nil {
		t.Fatal(err)
	}
	got, err := st.GetGroup("shaft-1")
	if err != nil {
		t.Fatal(err)
	}
	if got.Name != g.Name || len(got.Cams) != 2 || got.Cams[1].PhaseDeg != 270 {
		t.Fatalf("读回不一致: %+v", got)
	}
	list, err := st.ListGroups()
	if err != nil || len(list) != 1 || list[0].Name != "shaft-1" {
		t.Fatalf("列表错误: %v %+v", err, list)
	}
	if _, err := st.GetGroup("missing"); !errors.Is(err, ErrNotFound) {
		t.Fatalf("缺失组应返回 ErrNotFound，得到 %v", err)
	}
}

// 组校验：空名、空组、空 id、id 重复、相位未规约到 [0,360) 都要拒绝。
func TestGroupValidation(t *testing.T) {
	st := newTestStore(t)
	saveTestCycle(t, st, "w1")
	bad := []GroupRecord{
		{Name: "", Cams: []GroupCam{{ID: "c1", Cycle: "w1"}}},
		{Name: "g"},
		{Name: "g", Cams: []GroupCam{{ID: "", Cycle: "w1"}}},
		{Name: "g", Cams: []GroupCam{{ID: "c1", Cycle: "w1"}, {ID: "c1", Cycle: "w1", PhaseDeg: 10}}},
		{Name: "g", Cams: []GroupCam{{ID: "c1", Cycle: "w1", PhaseDeg: -1}}},
		{Name: "g", Cams: []GroupCam{{ID: "c1", Cycle: "w1", PhaseDeg: 360}}},
		{Name: "g", Cams: []GroupCam{{ID: "c1", Cycle: "w1", PhaseDeg: 1e300}}},
	}
	for i, g := range bad {
		if err := st.SaveGroup(g); err == nil {
			t.Fatalf("非法组 #%d 必须被拒绝: %+v", i, g)
		}
	}
}

// 组内点名的循环档必须已登记，否则拒绝。
func TestGroupUnknownCycleRejected(t *testing.T) {
	st := newTestStore(t)
	g := GroupRecord{Name: "g", Cams: []GroupCam{{ID: "c1", Cycle: "nope"}}}
	if err := st.SaveGroup(g); !errors.Is(err, ErrNotFound) {
		t.Fatalf("点名未登记循环档应拒绝（ErrNotFound），得到 %v", err)
	}
}
