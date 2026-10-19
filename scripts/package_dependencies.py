"""Repackage verified official source for offline builds, omitting example datasets."""
from __future__ import annotations
import copy
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): digest.update(block)
    return digest.hexdigest()


def keep_orb_member(name):
    relative = Path(*Path(name).parts[1:])
    if not relative.parts:
        return False
    top = relative.parts[0]
    if len(relative.parts) == 1:
        return True  # Including the original README, LICENSE, and dependency notices.
    if top in {'src', 'include', 'Thirdparty', 'Vocabulary', 'cmake_modules'}:
        return True
    if top in {'Examples', 'Examples_old'}:
        return relative.suffix in {'.cc', '.cpp', '.h', '.hpp', '.yaml', '.yml', '.in'} or relative.name == 'CMakeLists.txt'
    return False


def package():
    lock_path = ROOT/'dependencies.json'
    lock = json.loads(lock_path.read_text())
    vendor = ROOT/'vendor'
    vendor.mkdir(exist_ok=True)
    for name, item in lock.items():
        source = ROOT/'third_party'/f'{name}.tar.gz'
        if sha256(source) != item['archive_sha256']:
            raise ValueError(f'Official archive checksum mismatch: {source}')
        filename = 'orb_slam3-source.tar.gz' if name == 'orb_slam3' else 'pangolin-source.tar.gz'
        destination = vendor/filename
        if name == 'orb_slam3':
            with tarfile.open(source, 'r:gz') as original, destination.open('wb') as stream:
                with gzip.GzipFile(filename='', fileobj=stream, mode='wb', mtime=0) as compressed:
                    with tarfile.open(fileobj=compressed, mode='w') as reduced:
                        for member in original:
                            if not member.isfile() or not keep_orb_member(member.name): continue
                            info = copy.copy(member)
                            info.uid = info.gid = info.mtime = 0
                            info.uname = info.gname = ''
                            with original.extractfile(member) as content:
                                reduced.addfile(info, content)
        else:
            shutil.copyfile(source, destination)
        item['bundled_archive'] = destination.relative_to(ROOT).as_posix()
        item['bundled_sha256'] = sha256(destination)
        print(filename, destination.stat().st_size, item['bundled_sha256'])
    lock_path.write_text(json.dumps(lock, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    package()
