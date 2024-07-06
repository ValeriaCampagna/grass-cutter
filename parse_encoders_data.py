import  re

sp = re.compile(r"(\d+)")

with open("ticks_over_time.txt", "w") as w:
    with open("logs.txt", "r") as f:
        for line in f:
            if "Left ticks" in line:
                left, right, *_ = line.split("|")
                l_ticks = sp.findall(left)[0]
                r_ticks = sp.findall(right)[0]
                w.write(f"{l_ticks},{r_ticks}\n")
