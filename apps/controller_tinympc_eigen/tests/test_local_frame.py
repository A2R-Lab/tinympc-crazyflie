#!/usr/bin/env python3
"""Compile the production frame/reference functions against real Eigen/math3d."""
from pathlib import Path
import os
import subprocess
import tempfile


def definition(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + (";" if marker.startswith("struct ") else "")


def main():
    app = Path(__file__).resolve().parents[1]
    repo = app.parents[1]
    source = (app / "src/controller_tinympc.cpp").read_text()
    markers = [
        "struct MpcLocalFrame {",
        "static Eigen::Vector3f worldVectorToLocal(",
        "static struct vec worldQuaternionToLocalRodrigues(",
        "void updateInitialState(",
        "static void setLocalReferenceState(",
        "void updateHorizonReference(",
    ]
    with tempfile.TemporaryDirectory(prefix="tinympc-local-frame-") as directory:
        temp = Path(directory)
        # Globals are declared after the frame type, before the functions.
        chunks = [definition(source, markers[0]), "static MpcLocalFrame active_local_frame;"]
        chunks.extend(definition(source, marker) for marker in markers[1:])
        (temp / "production_frame.h").write_text("\n\n".join(chunks))
        binary = temp / "test_local_frame"
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++17", "-O1", "-g",
            "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
            "-isystem", str(app / "TinyMPC-ADMM/ext/Eigen"),
            "-isystem", str(repo / "crazyflie-firmware/src/modules/interface"),
            "-I", str(app / "src"), "-I", str(temp),
            str(app / "tests/test_local_frame.cpp"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
