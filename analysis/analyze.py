import re, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

pat = re.compile(r"Error:\s*([\d.]+) px \| Running Average \((\d+) frames\):\s*([\d.]+)")
rows = []
for line in open("gt_log.txt", errors="ignore"):
    m = pat.search(line)
    if m:
        rows.append((int(m.group(2)), float(m.group(1)), float(m.group(3))))

df = pd.DataFrame(rows, columns=["frame", "error_px", "running_avg_px"])
print("Frames analysed:", len(df))
print(df["error_px"].describe())
print("Median error:", df["error_px"].median())

plt.figure(figsize=(8, 4))
plt.plot(df["frame"], df["error_px"], marker="o", markersize=3)
plt.xlabel("Frame"); plt.ylabel("Error (px)")
plt.title("Pose estimation error per frame")
plt.grid(alpha=0.3); plt.tight_layout()
plt.savefig("1_error_per_frame.png", dpi=150); plt.close()

plt.figure(figsize=(8, 4))
plt.plot(df["frame"], df["running_avg_px"], color="red")
plt.xlabel("Frame"); plt.ylabel("Running average error (px)")
plt.title("Running average error")
plt.grid(alpha=0.3); plt.tight_layout()
plt.savefig("2_running_average.png", dpi=150); plt.close()

plt.figure(figsize=(8, 4))
plt.hist(df["error_px"], bins=15, edgecolor="black")
plt.xlabel("Error (px)"); plt.ylabel("Number of frames")
plt.title("Distribution of error")
plt.tight_layout()
plt.savefig("3_error_histogram.png", dpi=150); plt.close()

plt.figure(figsize=(4, 5))
plt.boxplot(df["error_px"])
plt.ylabel("Error (px)"); plt.title("Error spread")
plt.tight_layout()
plt.savefig("4_error_boxplot.png", dpi=150); plt.close()

if os.path.exists("posture_log.txt"):
    labels = [l.split(":", 1)[1].strip() for l in open("posture_log.txt") if l.startswith("data:")]
    if labels:
        counts = pd.Series(labels).value_counts()
        print(counts)
        plt.figure(figsize=(5, 4))
        counts.plot(kind="bar")
        plt.ylabel("Number of frames"); plt.title("Posture classification")
        plt.tight_layout()
        plt.savefig("5_posture_counts.png", dpi=150); plt.close()

df.to_csv("accuracy_data.csv", index=False)
print("Saved graphs and accuracy_data.csv")
