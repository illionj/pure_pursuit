#!/usr/bin/env python3
# convert_to_csv.py
#
# 将“time    state.x    state.y”这种**以制表符或多个空格分隔**的文本
# 转换成标准逗号分隔 (CSV) 文件。
#
# 用法示例：
#   python3 convert_to_csv.py raw.txt trajectory.csv
#
# ──────────────────────────────────────────────────────────────────────────────

import sys
import pandas as pd

def convert(in_path: str, out_path: str) -> None:
    """
    读取 `in_path`（任意空白符分隔），写出为 `out_path`（CSV）。
    """
    # `sep=r'\s+'` 可以同时处理制表符或若干空格
    df = pd.read_csv(in_path, sep=r'\s+')
    df.to_csv(out_path, index=False)
    print(f"✅  已生成 CSV：{out_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法：python3 convert_to_csv.py <输入文本> <输出 CSV>")
        sys.exit(1)

    convert(sys.argv[1], sys.argv[2])
