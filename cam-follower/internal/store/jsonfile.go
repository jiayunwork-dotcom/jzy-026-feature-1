package store

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
)

// readJSONFile 从 dir/file 读 JSON；file 为空时 path 直接是 dir（配合 glob 全路径）。
func readJSONFile(dir, file string, v any) error {
	path := dir
	if file != "" {
		path = filepath.Join(dir, file)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	if err := json.Unmarshal(b, v); err != nil {
		return fmt.Errorf("解析 %s 失败: %w", filepath.Base(path), err)
	}
	return nil
}

// atomicWriteJSON 先写临时文件再改名，避免半写文件被读到。
func atomicWriteJSON(dir, name string, v any) error {
	b, err := json.MarshalIndent(v, "", "  ")
	if err != nil {
		return err
	}
	final := filepath.Join(dir, name)
	tmp := final + ".tmp"
	if err := os.WriteFile(tmp, b, 0o644); err != nil {
		return fmt.Errorf("写入 %s 失败: %w", name, err)
	}
	if err := os.Rename(tmp, final); err != nil {
		_ = os.Remove(tmp)
		return fmt.Errorf("保存 %s 失败: %w", name, err)
	}
	return nil
}
