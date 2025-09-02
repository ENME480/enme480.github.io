#!/usr/bin/env python3
import os, re, sys, shutil, subprocess, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
MKDOCS = ROOT / "mkdocs.yml"
REQS = ROOT / "requirements.txt"
LAB_SUBMODULE = ROOT / "labs" / "Lab-Code"
MIRROR = DOCS / "_labcode"
LABS_DIR = DOCS / "labs"
SCRIPTS_DIR = ROOT / "scripts"

MARK_START = "<!-- BEGIN:AUTO-INCLUDE-README -->"
MARK_END   = "<!-- END:AUTO-INCLUDE-README -->"

# choose best MD in a folder by filename heuristic
RANK = [
    "readme", "handout", "lab", "instructions",
    "guide", "overview", "index", "assignment"
]
IMG_EXTS = {".png",".jpg",".jpeg",".gif",".svg",".webp"}
KEEP_EXTS = {".md",".pdf", *IMG_EXTS}

def die(msg): print(f"ERROR: {msg}", file=sys.stderr); sys.exit(1)

def run(*cmd): subprocess.run(cmd, check=True)

def ensure_repo_root():
    if not (ROOT / ".git").exists():
        die("Run from the root of enme480.github.io (no .git found).")
    if not MKDOCS.exists():
        die("mkdocs.yml not found.")
    if not LAB_SUBMODULE.exists():
        die("labs/Lab-Code not found. Add the submodule first.")

def ensure_requirements_and_plugin():
    REQS.parent.mkdir(exist_ok=True, parents=True)
    if not REQS.exists():
        REQS.write_text("mkdocs>=1.5\nmkdocs-material>=9.5\nmike>=2.1.0\npymdown-extensions>=10.8\n", encoding="utf-8")
    req = REQS.read_text(encoding="utf-8")
    if "mkdocs-include-markdown-plugin" not in req:
        REQS.write_text(req.rstrip()+"\nmkdocs-include-markdown-plugin\n", encoding="utf-8")

    y = MKDOCS.read_text(encoding="utf-8")
    if "plugins:" not in y:
        y += "\nplugins:\n  - search\n  - blog\n"
    if "include-markdown" not in y:
        y = re.sub(r"(?m)^plugins:\s*$", "plugins:\n  - search\n  - blog\n  - include-markdown", y)
    if "not_in_nav:" not in y:
        y += "\nnot_in_nav:\n  - _labcode/**\n"
    elif "_labcode/**" not in y:
        y = y.rstrip() + "\nnot_in_nav:\n  - _labcode/**\n"
    MKDOCS.write_text(y, encoding="utf-8")

def mirror_lab_assets_local():
    if MIRROR.exists():
        shutil.rmtree(MIRROR)
    for root, dirs, files in os.walk(LAB_SUBMODULE):
        rel = pathlib.Path(root).relative_to(LAB_SUBMODULE)
        out_dir = MIRROR / rel
        out_dir.mkdir(parents=True, exist_ok=True)
        for f in files:
            ext = pathlib.Path(f).suffix.lower()
            if ext in KEEP_EXTS:
                shutil.copy2(os.path.join(root,f), out_dir / f)

def best_md_in(folder: pathlib.Path):
    cands = [p for p in folder.glob("*.md")]
    if not cands:
        return None
    def score(p: pathlib.Path):
        name = p.stem.lower()
        s = 0
        for i, key in enumerate(RANK):
            if key in name:
                s += (len(RANK)-i)*10
        # prefer shorter names mildly
        return s - len(name)
    cands.sort(key=score, reverse=True)
    return cands[0]

def week_from_name(name: str):
    m = re.search(r"week\s*(\d+)", name, re.I)
    return int(m.group(1)) if m else None

def ensure_week_page(week: int) -> pathlib.Path:
    p = LABS_DIR / f"week-{week:02d}.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    if not p.exists():
        p.write_text(f"# Week {week:02d}\n\n", encoding="utf-8")
    return p

def ensure_final_page() -> pathlib.Path:
    p = LABS_DIR / "final-project.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    if not p.exists():
        p.write_text("# Final Project — Vision-Enabled Pick & Place\n\n", encoding="utf-8")
    return p

def github_folder_url(folder_name: str):
    from urllib.parse import quote
    return f"https://github.com/ENME480/Lab-Code/tree/main/{quote(folder_name)}"

def upsert_include_block(md_path: pathlib.Path, rel_include: str, folder_name: str, title_prefix: str):
    md = md_path.read_text(encoding="utf-8") if md_path.exists() else ""
    gh = github_folder_url(folder_name)
    block = (
        f"{MARK_START}\n"
        f"## Lab handout (from Lab-Code)\n\n"
        f"[View this lab folder on GitHub]({gh})\n\n"
        f"{{% include-markdown \"{rel_include}\" heading-offset=1 %}}\n"
        f"{MARK_END}\n"
    )
    if MARK_START in md and MARK_END in md:
        new = re.sub(re.escape(MARK_START)+r".*?"+re.escape(MARK_END), block, md, flags=re.S)
    else:
        new = md.rstrip()+"\n\n"+block if md.strip() else block
    md_path.write_text(new, encoding="utf-8")

def main():
    ensure_repo_root()
    ensure_requirements_and_plugin()
    mirror_lab_assets_local()

    # map each Lab-Code subfolder
    for p in LAB_SUBMODULE.iterdir():
        if not p.is_dir():
            continue

        title = p.name
        # Final project
        if re.search(r"final\s*project", title, re.I):
            target = ensure_final_page()
            mdfile = best_md_in(p)
            if not mdfile:
                print(f"Skip (no .md) -> {title}")
                continue
            rel = f"../_labcode/{title}/{mdfile.name}"  # from docs/labs/*.md
            upsert_include_block(target, rel, title, "Final Project")
            continue

        wk = week_from_name(title)
        if not wk:
            continue
        mdfile = best_md_in(p)
        if not mdfile:
            print(f"Skip (no .md) -> {title}")
            continue
        target = ensure_week_page(wk)
        rel = f"../_labcode/{title}/{mdfile.name}"      # from docs/labs/week-XX.md
        upsert_include_block(target, rel, title, f"Week {wk:02d}")

    # commit & push
    try:
        run("git","add","docs","mkdocs.yml","requirements.txt","scripts/inline_lab_markdowns.py")
        run("git","commit","-m","Inline Lab-Code markdown (auto); mirror assets into docs/_labcode")
        run("git","push","origin","main")
        print("Committed and pushed.")
    except subprocess.CalledProcessError:
        print("No changes to commit/push (or git failed).")

if __name__ == "__main__":
    main()
