"""Download official, version-pinned source archives (no git submodules required)."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]


def extract_archive(archive, destination):
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        prefixes = {m.name.split("/")[0] for m in members}
        if len(prefixes) != 1:
            raise ValueError("Expected one top-level archive directory")
        for member in members:
            relative = Path(*Path(member.name).parts[1:])
            if not relative.parts:
                continue
            target = (destination/relative).resolve()
            if not target.is_relative_to(destination):
                raise ValueError("Unsafe archive path")
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                source = tar.extractfile(member)
                with target.open("wb") as output:
                    import shutil
                    shutil.copyfileobj(source, output)
            else:
                raise ValueError(f"Unsupported archive link/device: {member.name}")


def fetch(name):
    lock = json.loads((ROOT/"dependencies.json").read_text())
    item = lock[name]
    folder = ROOT/"third_party"/({"orb_slam3": "ORB_SLAM3", "pangolin": "Pangolin"}[name])
    marker = folder/".source-ref"
    if marker.exists():
        if marker.read_text().strip() != item["ref"]:
            raise ValueError(f"{folder} has another version; choose a fresh checkout")
        print(f"Already downloaded {name}: {item['ref']}")
        return
    if folder.exists() and any(folder.iterdir()):
        raise ValueError(f"Incomplete/unrecognized source at {folder}; rename it before retrying")
    repo = item["repository"].removeprefix("https://github.com/")
    url = f"https://codeload.github.com/{repo}/tar.gz/{item['ref']}"
    archive = ROOT/"third_party"/f"{name}.tar.gz"
    archive.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(4):
        try:
            request = Request(url, headers={"User-Agent": "SLAM-test-setup"})
            with urlopen(request, timeout=120) as response, archive.open("wb") as output:
                import shutil
                shutil.copyfileobj(response, output)
            # Read entire tar stream before extraction to catch truncated downloads.
            with tarfile.open(archive) as check:
                check.getmembers()
            break
        except Exception:
            if attempt == 3:
                raise
            print(f"Retry download {name}: attempt {attempt+2}", flush=True)
            time.sleep(2)
    hasher = hashlib.sha256()
    with archive.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): hasher.update(block)
    digest = hasher.hexdigest()
    if item.get('archive_sha256') and digest != item['archive_sha256']:
        raise ValueError(f'Archive checksum mismatch for {name}; source not extracted')
    extract_archive(archive, folder)
    marker.write_text(item["ref"]+"\n")
    (folder/".archive-sha256").write_text(digest+"\n")
    print(f"Downloaded {name} {item['ref']} SHA256={digest}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", choices=["orb_slam3", "pangolin", "all"], default="all", nargs="?")
    args = parser.parse_args()
    for name in (["orb_slam3", "pangolin"] if args.name == "all" else [args.name]):
        fetch(name)
