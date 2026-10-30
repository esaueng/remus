# Compare two twiggy-attributed WASM builds per function, with the ::h<hash> suffix stripped.
import csv, re, subprocess, sys
from collections import Counter
def load(p):
    out = subprocess.run(["twiggy","top","-n","10000000","-f","csv",p],check=True,capture_output=True,text=True).stdout
    c = Counter(); n = Counter()
    for r in csv.DictReader(out.splitlines()):
        name = re.sub(r"::h[0-9a-f]{16}$", "", r["Name"])
        c[name] += int(r["ShallowSize"]); n[name] += 1
    return c, n
b, bn = load(sys.argv[1]); c, cn = load(sys.argv[2])
keys = set(b) | set(c)
new = [k for k in keys if k not in b]
gone = [k for k in keys if k not in c]
print(f"functions: base {len(b)}  cur {len(c)}  new {len(new)} ({sum(c[k] for k in new):,} B)  removed {len(gone)} ({sum(b[k] for k in gone):,} B)")
print(f"existing functions that grew: {sum(max(0,c[k]-b[k]) for k in keys if k in b and k in c):,} B   shrank: {sum(min(0,c[k]-b[k]) for k in keys if k in b and k in c):,} B\n")
print("Largest growth among existing functions:")
for k in sorted((k for k in keys if k in b and k in c), key=lambda k: b[k]-c[k])[:25]:
    print(f"{c[k]-b[k]:>+9,} {b[k]:>9,} -> {c[k]:>9,}  {k[:130]}")
print("\nLargest new functions:")
for k in sorted(new, key=lambda k: -c[k])[:25]:
    print(f"{c[k]:>9,}  {k[:140]}")
