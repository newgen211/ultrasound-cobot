import glob, sys
sys.path.insert(0, "src/segment")
from segment_tube import candidates, load_frame

for sec in ["section_106", "section_108", "section_109", "section_111"]:
    frames = sorted(glob.glob(f"data/clarius_sessions/{sec}/*.bin"))
    if not frames:
        print(sec, "no bins found"); continue
    hit = sum(1 for f in frames if candidates(load_frame(f)))
    print(f"{sec}: {hit}/{len(frames)} = {100*hit/len(frames):.0f}% duty")
