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


def archive_digest(path):
    hasher = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): hasher.update(block)
    return hasher.hexdigest()


def resolve_archive(name, item):
    """Prefer a verified offline bundle, then a complete official archive cache."""
    bundled = ROOT/item.get('bundled_archive', f'vendor/{name}-source.tar.gz')
    if bundled.is_file():
        digest = archive_digest(bundled)
        if not item.get('bundled_sha256') or digest != item['bundled_sha256']:
            raise ValueError(f'Offline archive checksum mismatch: {bundled}; update/re-upload this file')
        print(f'Using verified offline archive: {bundled}', flush=True)
        return bundled, digest
    repo = item['repository'].removeprefix('https://github.com/')
    url = f"https://codeload.github.com/{repo}/tar.gz/{item['ref']}"
    archive = ROOT/'third_party'/f'{name}.tar.gz'
    archive.parent.mkdir(parents=True, exist_ok=True)
    if archive.is_file() and archive_digest(archive) == item['archive_sha256']:
        print(f'Using verified cached official archive: {archive}', flush=True)
        return archive, item['archive_sha256']
    partial = archive.with_suffix(archive.suffix+'.partial')
    for attempt in range(4):
        try:
            request = Request(url, headers={'User-Agent': 'SLAM-test-setup', 'Accept-Encoding': 'identity'})
            with urlopen(request, timeout=120) as response, partial.open('wb') as output:
                import shutil
                shutil.copyfileobj(response, output)
            digest = archive_digest(partial)
            if digest != item['archive_sha256']:
                raise ValueError('Downloaded archive is incomplete or has a wrong checksum')
            partial.replace(archive)  # Never expose a partial transfer as a valid cache.
            return archive, digest
        except Exception as exc:
            if attempt == 3:
                raise RuntimeError(
                    f'{name} download failed: {type(exc).__name__}: {exc}. '
                    'Update the repository to obtain vendor/*-source.tar.gz, or upload the '
                    'verified offline archives from your local project; do not keep retrying '
                    'a broken GitHub route.') from exc
            print(f'Retry download {name}: attempt {attempt+2} ({type(exc).__name__})', flush=True)
            time.sleep(2)


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
    archive, digest = resolve_archive(name, item)
    extract_archive(archive, folder)
    marker.write_text(item["ref"]+"\n")
    (folder/".archive-sha256").write_text(digest+"\n")
    print(f"Downloaded {name} {item['ref']} SHA256={digest}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", choices=["orb_slam3", "pangolin", "all"], default="all", nargs="?")
    args = parser.parse_args()
    try:
        for name in (["orb_slam3", "pangolin"] if args.name == "all" else [args.name]):
            fetch(name)
    except (OSError, ValueError, RuntimeError, tarfile.TarError) as exc:
        parser.exit(1, f'Dependency setup failed: {exc}\n')
