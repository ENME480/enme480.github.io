#!/usr/bin/env python3
import re, sys, yaml, pathlib

root = pathlib.Path(__file__).resolve().parents[1]
index_md = root / "docs" / "index.md"
data_yml = root / "data" / "this_week.yml"

data = yaml.safe_load(data_yml.read_text(encoding="utf-8"))
week = data.get("current_week")
weeks = {w["week"]: w for w in data.get("weeks", [])}
now = weeks.get(week, {})

def render_list(items):
    return "\n".join(f"- {it}" for it in items or [])

def replace_block(text, marker, body):
    pat = re.compile(
        rf"(<!-- {marker}:start -->)(.*?)(<!-- {marker}:end -->)",
        re.DOTALL,
    )
    repl = rf"\1\n{body}\n\3"
    return pat.sub(repl, text)

index = index_md.read_text(encoding="utf-8")

index = replace_block(index, "THIS_WEEK:auto",
                      render_list(now.get("this_week")) or "_(no items for this week)_")

index = replace_block(index, "WHATS_NEW:auto",
                      render_list(now.get("whats_new")) or "_(no announcements)_")

index_md.write_text(index, encoding="utf-8")
print(f"Updated homepage for week {week}")
