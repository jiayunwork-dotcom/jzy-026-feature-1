// Package store 负责规律配方、循环档与凸轮组的本地文件持久化。
// 每条配方/循环档/凸轮组存为 data 目录下一个 JSON 文件，读写互斥保护。
package store

import (
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
	"sync"

	"camfollower/internal/cycle"
	"camfollower/internal/group"
	"camfollower/internal/law"
)

// Recipe 是一条规律配方：名字、类型、h、β（升程角，度）。
type Recipe struct {
	Name string  `json:"name"`
	Type string  `json:"type"`
	H    float64 `json:"h"`
	Beta float64 `json:"beta_deg"`
}

func (r Recipe) Validate() error {
	if strings.TrimSpace(r.Name) == "" {
		return errors.New("配方名不能为空")
	}
	if !law.Known(r.Type) {
		return law.ErrUnknownLaw
	}
	if math.IsNaN(r.H) || math.IsInf(r.H, 0) || r.H <= 0 {
		return errors.New("h 必须为正数")
	}
	if math.IsNaN(r.Beta) || math.IsInf(r.Beta, 0) || r.Beta <= 0 {
		return errors.New("β 必须为正数")
	}
	if r.Beta >= 360 {
		return fmt.Errorf("β 必须小于一周（360°），当前为 %g°", r.Beta)
	}
	return nil
}

// CycleRecord 是持久化的循环档：四段角度与各段类型。
// ω 不存档，求曲线时由请求给出。
type CycleRecord struct {
	Name        string  `json:"name"`
	H           float64 `json:"h"`
	RiseLaw     string  `json:"rise_law"`
	ReturnLaw   string  `json:"return_law"`
	RiseAngle   float64 `json:"rise_angle_deg"`
	OuterDwell  float64 `json:"outer_dwell_deg"`
	ReturnAngle float64 `json:"return_angle_deg"`
	InnerDwell  float64 `json:"inner_dwell_deg"`
}

func (r CycleRecord) Validate() error {
	if strings.TrimSpace(r.Name) == "" {
		return errors.New("循环档名不能为空")
	}
	cfg := cycle.Config{
		RiseLaw:     r.RiseLaw,
		ReturnLaw:   r.ReturnLaw,
		H:           r.H,
		RiseAngle:   r.RiseAngle,
		OuterDwell:  r.OuterDwell,
		ReturnAngle: r.ReturnAngle,
		InnerDwell:  r.InnerDwell,
		Omega:       1, // ω 不参与角度拼接校验，借一个正值
	}
	return cfg.Validate()
}

// ToConfig 用给定 ω 还原出可计算的循环配置。
func (r CycleRecord) ToConfig(omega float64) cycle.Config {
	return cycle.Config{
		RiseLaw:     r.RiseLaw,
		ReturnLaw:   r.ReturnLaw,
		H:           r.H,
		RiseAngle:   r.RiseAngle,
		OuterDwell:  r.OuterDwell,
		ReturnAngle: r.ReturnAngle,
		InnerDwell:  r.InnerDwell,
		Omega:       omega,
	}
}

var nameRe = regexp.MustCompile(`^[A-Za-z0-9_.\-一-鿿]+$`)

// ErrNotFound 表示点名的档不存在。
var ErrNotFound = errors.New("档不存在")

// Store 是 data 目录上的档读写器。
type Store struct {
	dir string
	mu  sync.Mutex
}

func Open(dir string) (*Store, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, fmt.Errorf("创建数据目录失败: %w", err)
	}
	return &Store{dir: dir}, nil
}

func checkName(name string) error {
	if strings.TrimSpace(name) == "" {
		return errors.New("名字不能为空")
	}
	if !nameRe.MatchString(name) {
		return errors.New("名字只能含字母、数字、下划线、点、连字符与中文")
	}
	if name == "." || name == ".." {
		return errors.New("名字非法")
	}
	return nil
}

// SaveRecipe 校验并原子写入一条配方（同名覆盖）。
func (s *Store) SaveRecipe(r Recipe) error {
	if err := r.Validate(); err != nil {
		return err
	}
	if err := checkName(r.Name); err != nil {
		return err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return atomicWriteJSON(s.dir, "recipe_"+r.Name+".json", r)
}

func (s *Store) GetRecipe(name string) (Recipe, error) {
	if err := checkName(name); err != nil {
		return Recipe{}, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	var r Recipe
	if err := readJSONFile(s.dir, "recipe_"+name+".json", &r); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return Recipe{}, fmt.Errorf("%w: 配方 %q", ErrNotFound, name)
		}
		return Recipe{}, err
	}
	return r, nil
}

// ListRecipes 按名序列出全部配方。
func (s *Store) ListRecipes() ([]Recipe, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	files, err := filepath.Glob(filepath.Join(s.dir, "recipe_*.json"))
	if err != nil {
		return nil, err
	}
	sort.Strings(files)
	out := make([]Recipe, 0, len(files))
	for _, f := range files {
		var r Recipe
		if err := readJSONFile(f, "", &r); err != nil {
			continue // 损坏文件跳过，不影响列表服务
		}
		out = append(out, r)
	}
	return out, nil
}

// SaveCycle 校验并原子写入一条循环档（同名覆盖）。
func (s *Store) SaveCycle(r CycleRecord) error {
	if err := r.Validate(); err != nil {
		return err
	}
	if err := checkName(r.Name); err != nil {
		return err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return atomicWriteJSON(s.dir, "cycle_"+r.Name+".json", r)
}

func (s *Store) GetCycle(name string) (CycleRecord, error) {
	if err := checkName(name); err != nil {
		return CycleRecord{}, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	var r CycleRecord
	if err := readJSONFile(s.dir, "cycle_"+name+".json", &r); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return CycleRecord{}, fmt.Errorf("%w: 循环档 %q", ErrNotFound, name)
		}
		return CycleRecord{}, err
	}
	return r, nil
}

func (s *Store) ListCycles() ([]CycleRecord, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	files, err := filepath.Glob(filepath.Join(s.dir, "cycle_*.json"))
	if err != nil {
		return nil, err
	}
	sort.Strings(files)
	out := make([]CycleRecord, 0, len(files))
	for _, f := range files {
		var r CycleRecord
		if err := readJSONFile(f, "", &r); err != nil {
			continue
		}
		out = append(out, r)
	}
	return out, nil
}

// SaveGroup 校验并原子写入一组凸轮（同名覆盖）。
// 相位角在写入前应由调用方规约到一周以内。
func (s *Store) SaveGroup(g group.Group) error {
	if err := g.Validate(); err != nil {
		return err
	}
	if err := checkName(g.Name); err != nil {
		return err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return atomicWriteJSON(s.dir, "group_"+g.Name+".json", g)
}

func (s *Store) GetGroup(name string) (group.Group, error) {
	if err := checkName(name); err != nil {
		return group.Group{}, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	var g group.Group
	if err := readJSONFile(s.dir, "group_"+name+".json", &g); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return group.Group{}, fmt.Errorf("%w: 凸轮组 %q", ErrNotFound, name)
		}
		return group.Group{}, err
	}
	return g, nil
}

// ListGroups 按名序列出全部凸轮组。
func (s *Store) ListGroups() ([]group.Group, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	files, err := filepath.Glob(filepath.Join(s.dir, "group_*.json"))
	if err != nil {
		return nil, err
	}
	sort.Strings(files)
	out := make([]group.Group, 0, len(files))
	for _, f := range files {
		var g group.Group
		if err := readJSONFile(f, "", &g); err != nil {
			continue // 损坏文件跳过，不影响列表服务
		}
		out = append(out, g)
	}
	return out, nil
}
