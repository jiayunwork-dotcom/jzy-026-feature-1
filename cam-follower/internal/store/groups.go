package store

import (
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// GroupCam 是组内一片凸轮：点名一份已登记的循环档 + 相位角。
// PhaseDeg 为本片循环 0° 相对基准转角的偏移，登记前已规约到 [0,360)。
type GroupCam struct {
	ID       string  `json:"id"`
	Cycle    string  `json:"cycle"`
	PhaseDeg float64 `json:"phase_deg"`
}

// GroupRecord 是持久化的凸轮组：一根轴上装了哪几片凸轮、各自相位。
type GroupRecord struct {
	Name string     `json:"name"`
	Cams []GroupCam `json:"cams"`
}

func (r GroupRecord) Validate() error {
	if strings.TrimSpace(r.Name) == "" {
		return errors.New("凸轮组名不能为空")
	}
	if len(r.Cams) == 0 {
		return errors.New("凸轮组至少要有一片凸轮")
	}
	seen := map[string]bool{}
	for _, c := range r.Cams {
		if strings.TrimSpace(c.ID) == "" {
			return errors.New("凸轮 id 不能为空")
		}
		if seen[c.ID] {
			return fmt.Errorf("凸轮 id 重复：%q", c.ID)
		}
		seen[c.ID] = true
		if strings.TrimSpace(c.Cycle) == "" {
			return fmt.Errorf("凸轮 %q 未点名循环档", c.ID)
		}
		if math.IsNaN(c.PhaseDeg) || math.IsInf(c.PhaseDeg, 0) {
			return fmt.Errorf("凸轮 %q 的相位角必须是有限实数", c.ID)
		}
		if c.PhaseDeg < 0 || c.PhaseDeg >= 360 {
			return fmt.Errorf("凸轮 %q 的相位角须先规约到 [0,360)，当前为 %g°", c.ID, c.PhaseDeg)
		}
	}
	return nil
}

// SaveGroup 校验并原子写入一组凸轮（同名覆盖）；
// 组内点名的循环档必须已登记，否则拒绝。
func (s *Store) SaveGroup(r GroupRecord) error {
	if err := r.Validate(); err != nil {
		return err
	}
	if err := checkName(r.Name); err != nil {
		return err
	}
	for _, c := range r.Cams {
		if _, err := s.GetCycle(c.Cycle); err != nil {
			if errors.Is(err, ErrNotFound) {
				return fmt.Errorf("凸轮 %q 点名的循环档未登记：%w", c.ID, err)
			}
			return err
		}
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return atomicWriteJSON(s.dir, "group_"+r.Name+".json", r)
}

func (s *Store) GetGroup(name string) (GroupRecord, error) {
	if err := checkName(name); err != nil {
		return GroupRecord{}, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	var r GroupRecord
	if err := readJSONFile(s.dir, "group_"+name+".json", &r); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return GroupRecord{}, fmt.Errorf("%w: 凸轮组 %q", ErrNotFound, name)
		}
		return GroupRecord{}, err
	}
	return r, nil
}

// ListGroups 按名序列出全部凸轮组。
func (s *Store) ListGroups() ([]GroupRecord, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	files, err := filepath.Glob(filepath.Join(s.dir, "group_*.json"))
	if err != nil {
		return nil, err
	}
	sort.Strings(files)
	out := make([]GroupRecord, 0, len(files))
	for _, f := range files {
		var r GroupRecord
		if err := readJSONFile(f, "", &r); err != nil {
			continue // 损坏文件跳过，不影响列表服务
		}
		out = append(out, r)
	}
	return out, nil
}
