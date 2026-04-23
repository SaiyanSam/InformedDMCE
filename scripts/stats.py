#!/usr/bin/env python3

import os, sys
import getopt
import csv
import numpy as np

from functions import gatherData, printMetaData

# Optional plotting (only used if --overlap-plot is set)
def _lazy_import_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def findBreakPoint(x, y, breakPoint):
    """ Assuming y in [0; 1] and y[0] = 0, find the first value of x that corresponds to y >= breakPoint """
    if not len(x) == len(y):
        raise Exception("x and y must have the same length!")
    for i in range(len(x) - 1):
        if y[i] < breakPoint and y[i+1] >= breakPoint:
            gamma = (breakPoint - y[i]) / (y[i+1] - y[i])
            return x[i] + gamma * (x[i+1] - x[i])
    return None


def _read_view_overlap_csv(csv_path):
    """
    Reads view_overlap.csv produced by overlap_logger.

    Expected header (from logger code we discussed):
      run_id,time_s,single_cells,multi_sum,cum_multi_sum,single_area_m2,multi_area_m2,cum_multi_area_m2

    Returns: list of runs, where each run is dict {'t': np.array, 'y': np.array}
             y is cumulative overlapped area in m^2 (cum_multi_area_m2).
    Also supports "appended runs" by splitting a run when time goes backward.
    """
    if not os.path.exists(csv_path):
        return []

    rows = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        for r in reader:
            if not r:
                continue
            # Skip header lines (non-numeric in first column)
            try:
                float(r[1] if len(r) > 1 else r[0])
            except Exception:
                continue
            rows.append(r)

    if len(rows) == 0:
        return []

    # Determine if run_id column exists (8 columns => yes, else we try best-effort)
    has_run_id = (len(rows[0]) >= 8)

    # Group by run_id if available, else group all as one
    groups = {}
    for r in rows:
        if has_run_id:
            try:
                run_id = int(float(r[0]))
                t = float(r[1])
                y = float(r[7])  # cum_multi_area_m2
            except Exception:
                continue
        else:
            try:
                run_id = 0
                t = float(r[0])
                # If no cum_multi_area_m2, fall back to last column
                y = float(r[-1])
            except Exception:
                continue

        groups.setdefault(run_id, []).append((t, y))

    # Split each group into runs if time resets
    runs = []
    for run_id, data in groups.items():
        data.sort(key=lambda x: x[0])  # may already be sorted, but safe

        # If the file was appended without run_id, "sort" destroys run ordering,
        # so we also support a monotonic split without sorting when run_id==0 and no run_id col.
        if not has_run_id and run_id == 0:
            data = []
            with open(csv_path, "r") as f:
                reader = csv.reader(f)
                for r in reader:
                    if not r:
                        continue
                    try:
                        float(r[0])
                    except Exception:
                        continue
                    try:
                        t = float(r[0])
                        y = float(r[-1])
                        data.append((t, y))
                    except Exception:
                        pass

        cur_t = []
        cur_y = []
        prev_t = None
        for (t, y) in data:
            if prev_t is not None and t < prev_t - 1e-6:
                # time went backward => new run starts
                if len(cur_t) > 1:
                    runs.append({"t": np.array(cur_t), "y": np.array(cur_y)})
                cur_t, cur_y = [], []
            cur_t.append(t)
            cur_y.append(y)
            prev_t = t

        if len(cur_t) > 1:
            runs.append({"t": np.array(cur_t), "y": np.array(cur_y)})

    return runs


def _aggregate_runs_time_series(runs, dt=1.0):
    """
    Interpolate each run onto common time grid [0..t_end] and return mean/std.
    """
    if len(runs) == 0:
        return None

    # Common end time: min of each run's final time (so all runs cover the grid)
    t_ends = [float(r["t"][-1]) for r in runs if len(r["t"]) > 1]
    if len(t_ends) == 0:
        return None
    t_end = min(t_ends)
    if t_end <= 0:
        return None

    t_common = np.arange(0.0, t_end + 1e-9, dt)

    ys = []
    for r in runs:
        t = r["t"]
        y = r["y"]
        # Ensure monotonic time for interp
        order = np.argsort(t)
        t = t[order]
        y = y[order]
        # Interp to common grid
        y_i = np.interp(t_common, t, y)
        ys.append(y_i)

    Y = np.vstack(ys)  # shape (nruns, nt)
    mean = np.mean(Y, axis=0)
    std = np.std(Y, axis=0)
    return {"t": t_common, "mean": mean, "std": std}


def _default_overlap_plot_out():
    # Save alongside other dmce_sim plots
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    out_dir = os.path.join(repo_root, "dmce_sim", "plots")
    os.makedirs(out_dir, exist_ok=True)
    return os.path.join(out_dir, "view_overlap.png")


def plot_overlap(aggregates_by_label, out_path):
    plt = _lazy_import_matplotlib()
    fig, ax = plt.subplots(figsize=(7, 4))

    for label, agg in aggregates_by_label.items():
        if agg is None:
            continue
        t = agg["t"]
        m = agg["mean"]
        s = agg["std"]
        ax.plot(t, m, label=label)
        ax.fill_between(t, m - s, m + s, alpha=0.25)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cumulative overlapped area (m^2)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def stats(directory, breakPoints, csvMode, label, labelColWidth=30,
          overlap_dt=1.0, collect_overlap=False):
    if not csvMode:
        print("\n")
    times, scores, distances, metaData, failedRuns = gatherData(directory, csvMode)

    if csvMode:
        print(label + " " * (labelColWidth - len(label)), end=',')
        print(f"{len(times):>7}", end=",")
    else:
        printMetaData(metaData)

        print(f"{'Runs used': >19}: {len(times)}/{len(times)+failedRuns} successful")
        if failedRuns > 0:
            print(f"{'Runs discarded': >19}: {failedRuns} with failures")
        print(f"{'Label': >19}: {label}")
        print("")

    for breakPoint in breakPoints:
        timeBreakPoints = []
        tDNF = 0
        distBreakPoints = []
        dDNF = 0
        for i in range(len(times)):
            tVal = findBreakPoint(times[i], scores[i], breakPoint)
            if not tVal is None:
                timeBreakPoints.append(tVal)
            else:
                tDNF += 1

            dVal = findBreakPoint(distances[i], scores[i], breakPoint)
            if not dVal is None:
                distBreakPoints.append(dVal)
            else:
                dDNF += 1

        t_label = f"T_{breakPoint*100:.0f}"
        t_mean = np.mean(timeBreakPoints) if len(timeBreakPoints) > 0 else float("NaN")
        t_std = np.std(timeBreakPoints) if len(timeBreakPoints) > 0 else float("NaN")
        d_label = f"D_{breakPoint*100:.0f}"
        d_mean = np.mean(distBreakPoints) if len(distBreakPoints) > 0 else float("NaN")
        d_std = np.std(distBreakPoints) if len(distBreakPoints) > 0 else float("NaN")

        if csvMode:
            print(f"{t_mean:10.3f},{t_std:10.3f},{d_mean:10.3f},{d_std:10.3f},", end='')
        else:
            print(f"{t_label : >19}: {t_mean:.2f}s (std: {t_std:.2f}, {tDNF} DNF)")
            print(f"{d_label : >19}: {d_mean:.2f}m (std: {d_std:.2f}, {dDNF} DNF)")

    # ---- New: overlap summary (final cumulative overlapped area) ----
    overlap_agg = None
    if collect_overlap:
        overlap_csv = os.path.join(directory, "view_overlap.csv")
        runs = _read_view_overlap_csv(overlap_csv)
        overlap_agg = _aggregate_runs_time_series(runs, dt=overlap_dt)

        if csvMode:
            if overlap_agg is None:
                print(f"{float('nan'):10.3f},{float('nan'):10.3f},", end='')
            else:
                final_vals = []
                for r in runs:
                    final_vals.append(float(r["y"][-1]))
                m = float(np.mean(final_vals)) if len(final_vals) else float("nan")
                s = float(np.std(final_vals)) if len(final_vals) else float("nan")
                print(f"{m:10.3f},{s:10.3f},", end='')
        else:
            if overlap_agg is None:
                print(f"{'Overlap final': >19}: N/A (no view_overlap.csv)")
            else:
                final_vals = [float(r["y"][-1]) for r in runs]
                m = float(np.mean(final_vals)) if len(final_vals) else float("nan")
                s = float(np.std(final_vals)) if len(final_vals) else float("nan")
                print(f"{'Overlap final': >19}: {m:.2f} m^2 (std: {s:.2f}, {len(final_vals)} runs)")

    print("")
    return overlap_agg


if __name__ == '__main__':
    dirsToScan = []
    breakPoints = []
    defaultBreakPoints = [0.5, 0.95]
    csvMode = False
    labels = []

    plotOverlapFlag = False
    overlapOut = None
    overlapDt = 1.0

    opts, args = getopt.getopt(
        sys.argv[1:],
        'hb:tl:po:d:',
        [
            'help',
            'breakpoint=',
            'table', 'csv',
            'labels=',
            'overlap-plot',
            'overlap-out=',
            'overlap-dt='
        ]
    )
    optnames = [opt[0] for opt in opts]

    if len(args) == 0 or '-h' in optnames or '--help' in optnames:
        print(f"Usage: ./stats.py [options] [directories]")
        print(f"\nRead simulation log data and print performance metrics.\nEach given directory will be scanned for CSV output files.")
        print(f"\nOptions:")
        print(f"-t,--table,--csv\tFormat output as a CSV table. Default: {csvMode}")
        print(f"-b,--breakpoint\t\tAdd a breakpoint in (0; 1). Default: {defaultBreakPoints}")
        print(f"-l, --labels\t\tComma-separated list of labels. Default: directory names.")
        print(f"-p, --overlap-plot\tGenerate overlap plot from view_overlap.csv.")
        print(f"-o, --overlap-out\tOutput path for overlap plot PNG. Default: dmce_sim/plots/view_overlap.png")
        print(f"-d, --overlap-dt\tTime step (s) for aggregating overlap curves. Default: {overlapDt}")
        exit()

    for opt, val in opts:
        if opt in ('-b', '--breakpoint'):
            breakPoints.append(float(val))
        elif opt in ('-t', '--table', '--csv'):
            csvMode = True
        elif opt in ('-l', '--labels'):
            labels = val.split(',')
        elif opt in ('-p', '--overlap-plot'):
            plotOverlapFlag = True
        elif opt in ('-o', '--overlap-out'):
            overlapOut = val
        elif opt in ('-d', '--overlap-dt', '--overlap-dt='):
            overlapDt = float(val)
        elif opt in ('--overlap-out',):
            overlapOut = val
        elif opt in ('--overlap-dt',):
            overlapDt = float(val)

    if len(breakPoints) == 0:
        breakPoints = defaultBreakPoints

    dirsToScan = args

    labelColWidth = 0
    for i in range(len(dirsToScan)):
        hasLabel = i < len(labels)
        label = labels[i] if hasLabel else os.path.basename(os.path.normpath(dirsToScan[i]))
        labelColWidth = max(labelColWidth, len(label))
        if not hasLabel:
            labels.append(label)

    if csvMode:
        print(" " * labelColWidth, end=',')
        print("Samples", end=',')
        for breakPoint in breakPoints:
            print(f"      T_{breakPoint*100:.0f},       std,      D_{breakPoint*100:.0f},       std,", end='')
        # New columns:
        print(" Overlap_m2,       std,", end='')
        print('')

    # Collect overlap aggregates for plotting (mean±std curves)
    overlap_aggs = {}

    for i in range(len(dirsToScan)):
        agg = stats(
            dirsToScan[i], breakPoints, csvMode, labels[i], labelColWidth,
            overlap_dt=overlapDt,
            collect_overlap=(plotOverlapFlag or csvMode)
        )
        if plotOverlapFlag:
            overlap_aggs[labels[i]] = agg

    if plotOverlapFlag:
        out_path = overlapOut if overlapOut is not None else _default_overlap_plot_out()
        plot_overlap(overlap_aggs, out_path)
        print(f"Saved overlap plot to: {out_path}")
