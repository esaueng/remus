import os,re,json,sys
def split_cells(line):
    """Split a markdown table row on | that is neither escaped nor inside backticks."""
    out=[];cur=[];i=0;tick=False
    while i<len(line):
        c=line[i]
        if c=="\\" and i+1<len(line) and line[i+1]=="|":
            cur.append("|");i+=2;continue
        if c=="`": tick=not tick;cur.append(c);i+=1;continue
        if c=="|" and not tick:
            out.append("".join(cur));cur=[];i+=1;continue
        cur.append(c);i+=1
    out.append("".join(cur))
    return out
def cell(s):
    s=s.strip()
    if s.startswith("`") and s.endswith("`") and len(s)>1: s=s[1:-1]
    return s.strip()
def parse(path, expected):
    """Parse a 3-column table; tolerate rows with or without leading/trailing pipes."""
    rows=[]
    exp_set=set(expected)
    for line in open(path):
        line=line.rstrip("\n")
        if "|" not in line: continue
        cells=[c for c in split_cells(line)]
        # drop empty leading/trailing cells produced by outer pipes
        if cells and cells[0].strip()=="": cells=cells[1:]
        if cells and cells[-1].strip()=="": cells=cells[:-1]
        if len(cells)<3: continue
        m=cell(cells[0])
        if m=="Mutant" or set(m)<=set("- :"): continue
        rows.append((m,cell(cells[1])," ".join(cell(c) for c in cells[2:]).strip()))
    return rows
