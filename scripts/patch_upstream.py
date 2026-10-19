"""Apply a small, idempotent export hook to the pinned upstream source."""
from pathlib import Path
import argparse
import shutil

ROOT = Path(__file__).resolve().parents[1]


def patch(source):
    source = Path(source)
    header = source/"include/System.h"
    text = header.read_text()
    declaration = "    void ExportMonocularReplay(const std::string& directory);\n\n"
    if declaration.strip() not in text:
        if text.count("private:") != 1:
            raise ValueError("Unexpected System.h layout; export hook not applied")
        text = text.replace("private:", declaration+"private:", 1)
        header.write_text(text)
    cmake = source/"CMakeLists.txt"
    text = cmake.read_text()
    # Pinned Sophus/Pangolin headers build with a modern language standard.
    text = text.replace("-std=c++11", "-std=c++17").replace("-std=c++0x", "-std=c++17")
    block = '\n# SLAM-test export hook\ntarget_sources(${PROJECT_NAME} PRIVATE src/ExportReplay.cc)\n'
    if block not in text:
        text += block
    cmake.write_text(text)
    shutil.copyfile(ROOT/"cpp/ExportReplay.cc", source/"src/ExportReplay.cc")
    print(f"Patched export hook: {source}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", default=ROOT/"third_party/ORB_SLAM3")
    patch(parser.parse_args().source)
