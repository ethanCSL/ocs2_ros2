#!/usr/bin/env python3

from __future__ import annotations

import argparse
import struct
from pathlib import Path


def is_binary_stl(path: Path) -> bool:
    size = path.stat().st_size
    if size < 84:
        return False
    with path.open("rb") as stream:
        header = stream.read(80)
        count_bytes = stream.read(4)
    triangle_count = struct.unpack("<I", count_bytes)[0]
    expected_size = 84 + triangle_count * 50
    if expected_size == size:
        return True
    return not header.lstrip().startswith(b"solid")


def load_binary_stl(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    triangles = []
    with path.open("rb") as stream:
        stream.read(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        for _ in range(triangle_count):
            stream.read(12)  # normal
            vertices = []
            for _ in range(3):
                vertices.append(struct.unpack("<3f", stream.read(12)))
            stream.read(2)  # attribute byte count
            triangles.append(tuple(vertices))
    return triangles


def load_ascii_stl(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    triangles = []
    current_vertices: list[tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped.startswith("vertex "):
                continue
            _, x, y, z = stripped.split()
            current_vertices.append((float(x), float(y), float(z)))
            if len(current_vertices) == 3:
                triangles.append(tuple(current_vertices))
                current_vertices = []
    return triangles


def load_stl(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    if is_binary_stl(path):
        return load_binary_stl(path)
    return load_ascii_stl(path)


def write_obj(path: Path, triangles: list[tuple[tuple[float, float, float], ...]]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write(f"# Generated from {path.stem}.STL\n")
        vertex_index = 1
        for triangle in triangles:
            for vertex in triangle:
                stream.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")
            stream.write(f"f {vertex_index} {vertex_index + 1} {vertex_index + 2}\n")
            vertex_index += 3


def convert_directory(mesh_dir: Path) -> int:
    converted = 0
    for stl_path in sorted(mesh_dir.glob("*.STL")):
        obj_path = stl_path.with_suffix(".obj")
        triangles = load_stl(stl_path)
        write_obj(obj_path, triangles)
        converted += 1
    return converted


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert STL meshes to OBJ for MuJoCo.")
    parser.add_argument("mesh_dir", type=Path, help="Directory containing *.STL meshes.")
    args = parser.parse_args()

    converted = convert_directory(args.mesh_dir)
    print(converted)


if __name__ == "__main__":
    main()
