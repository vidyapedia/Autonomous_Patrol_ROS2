import csv, pathlib
import matplotlib.pyplot as plt

p = pathlib.Path.home() / "damn_patrol" / "results" / "patrol_metrics.csv"
rows = list(csv.DictReader(open(p)))

t = [float(r["time_to_goal_sec"]) for r in rows]
succ = [1 if r["success"].lower() == "true" else 0 for r in rows]
retries = [int(r["retries_used"]) for r in rows]

plt.figure()
plt.plot(t)
plt.title("Time to Goal (s)")
plt.xlabel("Attempt")
plt.ylabel("seconds")
plt.show()

plt.figure()
plt.plot(succ)
plt.title("Success per Attempt (1/0)")
plt.xlabel("Attempt")
plt.ylabel("success")
plt.show()

plt.figure()
plt.plot(retries)
plt.title("Retries per Attempt")
plt.xlabel("Attempt")
plt.ylabel("retries")
plt.show()
