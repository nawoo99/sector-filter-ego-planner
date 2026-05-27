#!/usr/bin/env python3
"""
실행 중인 Gazebo에 랜덤 기둥을 스폰하는 스크립트. (gz service 사용)
드론을 먼저 실행한 뒤 이 스크립트를 실행하세요.

사용법: python3 spawn_pillars.py [--seed 42] [--count 50] [--world default]
"""

import argparse
import random
import math
import struct
import subprocess
import os
import sys
import time

# === 설정 ===
RADIUS = 0.25
HEIGHT = 5.0
AREA_SIZE = 30.0
MIN_DIST = 0.5
SPAWN_MARGIN = 3.0  # 중앙 보호 반경 (드론 이륙 지점)

COLORS = {
    "cylinder": "0.8 0.2 0.2 1",
    "box":      "0.2 0.6 0.8 1",
    "triangle": "0.2 0.8 0.3 1",
    "hexagon":  "0.9 0.7 0.1 1",
}
TYPES = list(COLORS.keys())

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(SCRIPT_DIR, "meshes")


# ── STL 메시 생성 ──

def compute_normal(v1, v2, v3):
    u = (v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2])
    v = (v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2])
    n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    l = math.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
    return (n[0]/l, n[1]/l, n[2]/l) if l else (0, 0, 1)


def write_stl(filepath, triangles):
    with open(filepath, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for nm, v1, v2, v3 in triangles:
            for v in (nm, v1, v2, v3):
                f.write(struct.pack("<fff", *v))
            f.write(struct.pack("<H", 0))


def make_prism_stl(n_sides, radius, height, filepath):
    bot, top = [], []
    for i in range(n_sides):
        a = 2 * math.pi * i / n_sides - math.pi / 2
        bot.append((radius*math.cos(a), radius*math.sin(a), 0.0))
        top.append((radius*math.cos(a), radius*math.sin(a), height))
    tris = []
    for i in range(n_sides):
        j = (i+1) % n_sides
        n = compute_normal(bot[i], bot[j], top[i])
        tris.append((n, bot[i], bot[j], top[i]))
        n = compute_normal(top[i], bot[j], top[j])
        tris.append((n, top[i], bot[j], top[j]))
    bc, tc = (0, 0, 0.0), (0, 0, height)
    for i in range(n_sides):
        j = (i+1) % n_sides
        n = compute_normal(bc, bot[j], bot[i])
        tris.append((n, bc, bot[j], bot[i]))
        n = compute_normal(tc, top[i], top[j])
        tris.append((n, tc, top[i], top[j]))
    write_stl(filepath, tris)


def ensure_meshes():
    os.makedirs(MESH_DIR, exist_ok=True)
    for name, sides in [("triangle", 3), ("hexagon", 6)]:
        path = os.path.join(MESH_DIR, f"{name}_prism.stl")
        if not os.path.exists(path):
            make_prism_stl(sides, RADIUS, HEIGHT, path)
            print(f"  메시 생성: {path}")


# ── 위치 생성 ──

def generate_positions(n, area, min_dist, margin):
    positions = []
    half = area / 2.0
    attempts = 0
    while len(positions) < n and attempts < n * 200:
        x = random.uniform(-half, half)
        y = random.uniform(-half, half)
        if math.hypot(x, y) < margin:
            attempts += 1
            continue
        if all(math.hypot(x-px, y-py) >= min_dist for px, py in positions):
            positions.append((x, y))
        attempts += 1
    return positions


# ── SDF 인라인 문자열 생성 ──

def make_sdf_inline(name, ptype, x, y, yaw):
    color = COLORS[ptype]
    z = HEIGHT / 2.0

    if ptype == "cylinder":
        geom = f"<cylinder><radius>{RADIUS}</radius><length>{HEIGHT}</length></cylinder>"
    elif ptype == "box":
        s = RADIUS * math.sqrt(2)
        geom = f"<box><size>{s:.4f} {s:.4f} {HEIGHT}</size></box>"
    else:
        mesh_path = os.path.join(MESH_DIR, f"{ptype}_prism.stl")
        geom = f"<mesh><uri>file://{mesh_path}</uri></mesh>"
        z = 0.0

    return (
        f'<sdf version=\\"1.8\\">'
        f'<model name=\\"{name}\\">'
        f'<static>true</static>'
        f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>'
        f'<link name=\\"l\\">'
        f'<collision name=\\"c\\"><geometry>{geom}</geometry></collision>'
        f'<visual name=\\"v\\"><geometry>{geom}</geometry>'
        f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
        f'</visual></link></model></sdf>'
    )


# ── 스폰 ──

def spawn(sdf_inline, world_name):
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", f'sdf: "{sdf_inline}"'
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return "data: true" in result.stdout


def main():
    parser = argparse.ArgumentParser(description="실행 중인 Gazebo에 랜덤 기둥 스폰")
    parser.add_argument("--seed", type=int, default=1, help="랜덤 시드 (기본: 1)")
    parser.add_argument("--count", type=int, default=100, help="기둥 개수 (기본: 100)")
    parser.add_argument("--area", type=float, default=AREA_SIZE, help="영역 크기 m (기본: 30)")
    parser.add_argument("--margin", type=float, default=SPAWN_MARGIN, help="중앙 보호 반경 m (기본: 3)")
    parser.add_argument("--world", type=str, default="default", help="월드 이름 (기본: default)")
    parser.add_argument("--delay", type=float, default=0.05, help="스폰 간 딜레이 초 (기본: 0.1)")
    args = parser.parse_args()

    # 서비스 존재 확인
    check = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True)
    if f"/world/{args.world}/create" not in check.stdout:
        print(f"오류: 월드 '{args.world}'의 create 서비스가 없습니다.")
        print("Gazebo가 실행 중인지 확인하세요.")
        sys.exit(1)

    random.seed(args.seed)
    print(f"시드: {args.seed} | 개수: {args.count} | 영역: {args.area}x{args.area}m | 중앙보호: {args.margin}m")

    ensure_meshes()

    positions = generate_positions(args.count, args.area, MIN_DIST, args.margin)
    print(f"배치 위치 {len(positions)}개 생성\n")

    success, fail = 0, 0
    for i, (x, y) in enumerate(positions):
        ptype = random.choice(TYPES)
        yaw = random.uniform(0, 2 * math.pi)
        name = f"pillar_{args.seed}_{i:03d}"
        sdf = make_sdf_inline(name, ptype, x, y, yaw)

        ok = spawn(sdf, args.world)
        status = "✓" if ok else "✗"
        print(f"  [{i+1:3d}/{len(positions)}] {status} {name} ({ptype}) @ ({x:.1f}, {y:.1f})")
        if ok:
            success += 1
        else:
            fail += 1
        time.sleep(args.delay)

    print(f"\n완료: 성공 {success} / 실패 {fail}")


if __name__ == "__main__":
    main()