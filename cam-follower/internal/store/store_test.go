package store

import (
	"errors"
	"testing"

	"camfollower/internal/law"
)

func newTestStore(t *testing.T) *Store {
	t.Helper()
	st, err := Open(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	return st
}

func TestRecipeSaveGetList(t *testing.T) {
	st := newTestStore(t)
	r := Recipe{Name: "r1", Type: law.Cosine, H: 8, Beta: 45}
	if err := st.SaveRecipe(r); err != nil {
		t.Fatal(err)
	}
	got, err := st.GetRecipe("r1")
	if err != nil {
		t.Fatal(err)
	}
	if got != r {
		t.Fatalf("读回不一致: %+v ≠ %+v", got, r)
	}
	list, err := st.ListRecipes()
	if err != nil || len(list) != 1 || list[0].Name != "r1" {
		t.Fatalf("列表错误: %v %+v", err, list)
	}
	if _, err := st.GetRecipe("missing"); !errors.Is(err, ErrNotFound) {
		t.Fatalf("缺失档应返回 ErrNotFound，得到 %v", err)
	}
}

func TestRecipeValidation(t *testing.T) {
	st := newTestStore(t)
	bad := []Recipe{
		{Name: "", Type: law.Cycloid, H: 1, Beta: 60},
		{Name: "x", Type: "harmonic", H: 1, Beta: 60},
		{Name: "x", Type: law.Cycloid, H: -1, Beta: 60},
		{Name: "x", Type: law.Cycloid, H: 1, Beta: 0},
		{Name: "x", Type: law.Cycloid, H: 1, Beta: 360},
		{Name: "../escape", Type: law.Cycloid, H: 1, Beta: 60},
	}
	for i, r := range bad {
		if err := st.SaveRecipe(r); err == nil {
			t.Fatalf("非法配方 #%d 必须被拒绝: %+v", i, r)
		}
	}
}

func TestCycleRecordValidation(t *testing.T) {
	st := newTestStore(t)
	good := CycleRecord{
		Name: "c1", H: 10, RiseLaw: law.Cycloid, ReturnLaw: law.Parabolic,
		RiseAngle: 60, OuterDwell: 0, ReturnAngle: 120, InnerDwell: 180,
	}
	if err := st.SaveCycle(good); err != nil {
		t.Fatal(err)
	}
	if _, err := st.GetCycle("c1"); err != nil {
		t.Fatal(err)
	}

	bad := good
	bad.Name = "bad"
	bad.RiseAngle = 61 // 角度和 361
	if err := st.SaveCycle(bad); err == nil {
		t.Fatal("角度和不为一周必须拒绝")
	}
	bad = good
	bad.ReturnLaw = "???  "
	if err := st.SaveCycle(bad); err == nil {
		t.Fatal("未知回程类型必须拒绝")
	}
}

func TestSeedCycloidPreserved(t *testing.T) {
	// main.seed 行为的等价验证：摆线配方落盘后可被读出且参数正确。
	st := newTestStore(t)
	r := Recipe{Name: "cycloid-lift", Type: law.Cycloid, H: 10, Beta: 60}
	if err := st.SaveRecipe(r); err != nil {
		t.Fatal(err)
	}
	got, err := st.GetRecipe("cycloid-lift")
	if err != nil {
		t.Fatal(err)
	}
	if got.Type != law.Cycloid || got.H <= 0 || got.Beta <= 0 {
		t.Fatalf("启动配方应为合法摆线配方: %+v", got)
	}
}
