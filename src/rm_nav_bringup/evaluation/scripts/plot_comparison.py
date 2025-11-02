#!/usr/bin/env python3
"""绘图脚本：读取 benchmark_summary CSV（或单次 result JSON）并生成 ATE/RPE 箱线图与 CPU 比较图

Usage:
  python3 plot_comparison.py --results-dir ~/sentry_evaluation_data --out-dir ~/sentry_evaluation_data/plots
"""
import argparse
import csv
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def load_from_csv(csv_path: Path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as cf:
        reader = csv.DictReader(cf)
        for r in reader:
            # convert numeric fields when possible
            try:
                r['ate_rmse'] = float(r['ate_rmse'])
            except Exception:
                r['ate_rmse'] = np.nan
            try:
                r['rpe_rmse'] = float(r['rpe_rmse'])
            except Exception:
                r['rpe_rmse'] = np.nan
            try:
                r['cpu_max'] = float(r['cpu_max'])
            except Exception:
                r['cpu_max'] = np.nan
            rows.append(r)
    return rows


def aggregate_by_method(rows):
    data = {}
    for r in rows:
        key = (r['method'], r['scenario'])
        data.setdefault(key, {'ate': [], 'rpe': [], 'cpu': []})
        data[key]['ate'].append(r['ate_rmse'])
        data[key]['rpe'].append(r['rpe_rmse'])
        data[key]['cpu'].append(r['cpu_max'])
    return data


def plot_boxplots(data, out_dir: Path):
    # data: dict[(method,scenario)]->{ate:[], rpe:[], cpu:[]}
    # For each scenario, create a boxplot across methods for ATE and RPE
    scenarios = sorted(list(set(k[1] for k in data.keys())))
    methods = sorted(list(set(k[0] for k in data.keys())))

    for scenario in scenarios:
        ate_groups = []
        rpe_groups = []
        cpu_groups = []
        labels = []
        for method in methods:
            key = (method, scenario)
            if key in data:
                ate_groups.append([v for v in data[key]['ate'] if not np.isnan(v)])
                rpe_groups.append([v for v in data[key]['rpe'] if not np.isnan(v)])
                cpu_groups.append([v for v in data[key]['cpu'] if not np.isnan(v)])
                labels.append(method)
        if not labels:
            continue
        # ATE boxplot
        plt.figure(figsize=(8,6))
        plt.boxplot(ate_groups, labels=labels, showmeans=True)
        plt.ylabel('ATE RMSE (m)')
        plt.title(f'ATE comparison - {scenario}')
        plt.grid(True, linestyle='--', alpha=0.5)
        out_file = out_dir / f'ate_box_{scenario}.png'
        plt.tight_layout()
        plt.savefig(out_file)
        plt.close()

        # RPE boxplot
        plt.figure(figsize=(8,6))
        plt.boxplot(rpe_groups, labels=labels, showmeans=True)
        plt.ylabel('RPE RMSE (m)')
        plt.title(f'RPE comparison - {scenario}')
        plt.grid(True, linestyle='--', alpha=0.5)
        out_file = out_dir / f'rpe_box_{scenario}.png'
        plt.tight_layout()
        plt.savefig(out_file)
        plt.close()

        # CPU bar (mean)
        means = [np.nanmean(g) if len(g)>0 else np.nan for g in cpu_groups]
        plt.figure(figsize=(8,6))
        plt.bar(labels, means)
        plt.ylabel('CPU max (%)')
        plt.title(f'CPU max comparison - {scenario}')
        out_file = out_dir / f'cpu_bar_{scenario}.png'
        plt.tight_layout()
        plt.savefig(out_file)
        plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir', type=str, required=True)
    parser.add_argument('--out-dir', type=str, required=False)
    args = parser.parse_args()

    results_dir = Path(args.results_dir).expanduser()
    out_dir = Path(args.out_dir).expanduser() if args.out_dir else results_dir / 'plots'
    out_dir.mkdir(parents=True, exist_ok=True)

    # find benchmark_summary CSV (pick most recent), else parse result_*.json
    csv_files = sorted(results_dir.glob('benchmark_summary_*.csv'))
    rows = []
    if csv_files:
        rows = load_from_csv(csv_files[-1])
    else:
        # find result_*.json files
        json_files = sorted(results_dir.glob('result_*.json'))
        for jf in json_files:
            try:
                with open(jf, 'r', encoding='utf-8') as f:
                    r = json.load(f)
                ate = r.get('trajectory_metrics', {}).get('ate', {}).get('rmse', None)
                rpe = r.get('trajectory_metrics', {}).get('rpe', {}).get('rmse', None)
                cpu = r.get('performance_metrics', {}).get('cpu', {}).get('max', None) if r.get('performance_metrics') else None
                rows.append({'method': r.get('method',''), 'scenario': r.get('scenario',''), 'repeat': r.get('repeat',1), 'seed': r.get('seed',None), 'ate_rmse': ate, 'rpe_rmse': rpe, 'cpu_max': cpu, 'mem_max': None, 'success': r.get('success', False), 'bag_file': r.get('bag_file',''), 'timestamp': r.get('timestamp',0)})
            except Exception:
                continue

    if not rows:
        print('未找到结果数据，检查 --results-dir 路径')
        return

    data = aggregate_by_method(rows)
    plot_boxplots(data, out_dir)
    print(f'已生成图表，保存在: {out_dir}')

if __name__ == '__main__':
    main()
