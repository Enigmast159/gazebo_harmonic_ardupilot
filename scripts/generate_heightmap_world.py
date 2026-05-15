#!/usr/bin/env python3
"""
Generate the canonical Gazebo Harmonic world using native SDF heightmap terrain.

This script intentionally does not use STL / OBJ for the terrain visual. The
terrain is rendered as a native Gazebo heightmap, which avoids mesh winding,
backface-culling and stale visual mesh-cache artifacts. Collision is a generated
STL because the available Bullet backend does not support heightmap collision.

This generator owns the canonical `random_terrain_ardupilot.sdf` world used by
the final simulator stack.
"""

import argparse
import csv
import datetime
import json
import math
import os
import struct

import numpy as np
from opensimplex import OpenSimplex
from PIL import Image, ImageDraw


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)


def repo_path(*parts):
    return os.path.join(REPO_ROOT, *parts)


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)
    return path


def smooth_heightmap(data, passes):
    smoothed = data.astype(np.float32)
    for _ in range(max(0, passes)):
        padded = np.pad(smoothed, 1, mode="edge")
        smoothed = (
            padded[:-2, :-2] + 2 * padded[:-2, 1:-1] + padded[:-2, 2:]
            + 2 * padded[1:-1, :-2] + 4 * padded[1:-1, 1:-1] + 2 * padded[1:-1, 2:]
            + padded[2:, :-2] + 2 * padded[2:, 1:-1] + padded[2:, 2:]
        ) / 16.0
    return smoothed


def generate_heightmap(size, scale, octaves, persistence, lacunarity, seed, smooth_passes):
    if seed is None:
        seed = int(np.random.default_rng().integers(0, 1_000_000))

    simplex = OpenSimplex(seed)
    data = np.zeros((size, size), dtype=np.float32)

    for row in range(size):
        for col in range(size):
            nx = col / scale
            ny = row / scale
            amplitude = 1.0
            frequency = 1.0
            value = 0.0
            norm = 0.0

            for _ in range(octaves):
                value += simplex.noise2(nx * frequency, ny * frequency) * amplitude
                norm += amplitude
                amplitude *= persistence
                frequency *= lacunarity

            data[row, col] = value / max(norm, 1e-9)

    data = smooth_heightmap(data, smooth_passes)
    span = float(data.max() - data.min())
    if span <= 1e-9:
        normalized = np.zeros_like(data, dtype=np.float32)
    else:
        normalized = (data - data.min()) / span

    return (normalized * 255.0).clip(0, 255).astype(np.uint8), seed


def terrain_height_at(heightmap, x_m, y_m, scale_x, scale_y, scale_z):
    rows, cols = heightmap.shape
    u = (x_m + scale_x / 2.0) / scale_x * (cols - 1)
    v = (y_m + scale_y / 2.0) / scale_y * (rows - 1)
    u = float(np.clip(u, 0, cols - 1))
    v = float(np.clip(v, 0, rows - 1))

    x0 = int(math.floor(u))
    y0 = int(math.floor(v))
    x1 = min(x0 + 1, cols - 1)
    y1 = min(y0 + 1, rows - 1)
    tx = u - x0
    ty = v - y0

    h00 = float(heightmap[y0, x0])
    h10 = float(heightmap[y0, x1])
    h01 = float(heightmap[y1, x0])
    h11 = float(heightmap[y1, x1])
    h0 = h00 * (1.0 - tx) + h10 * tx
    h1 = h01 * (1.0 - tx) + h11 * tx
    return (h0 * (1.0 - ty) + h1 * ty) / 255.0 * scale_z


def generate_tree_poses(heightmap, seed, count, scale_x, scale_y, scale_z,
                        platform_x, platform_y, exclusion_radius, terrain_margin=8.0):
    rng = np.random.default_rng((seed if seed is not None else 0) + 1009)
    poses = []
    attempts = 0
    max_attempts = max(1, count * 100)
    min_spacing = 5.0

    while len(poses) < count and attempts < max_attempts:
        attempts += 1
        x_m = rng.uniform(-scale_x / 2.0 + terrain_margin, scale_x / 2.0 - terrain_margin)
        y_m = rng.uniform(-scale_y / 2.0 + terrain_margin, scale_y / 2.0 - terrain_margin)

        if math.hypot(x_m - platform_x, y_m - platform_y) < exclusion_radius:
            continue
        if any(math.hypot(x_m - px, y_m - py) < min_spacing for px, py, _, _ in poses):
            continue

        z_m = terrain_height_at(heightmap, x_m, y_m, scale_x, scale_y, scale_z)
        yaw = rng.uniform(-math.pi, math.pi)
        poses.append((round(x_m, 3), round(y_m, 3), round(z_m, 3), round(yaw, 6)))

    return poses


def save_texture(path, base_color, accent_color):
    img = Image.new("RGB", (256, 256), base_color)
    draw = ImageDraw.Draw(img)
    rng = np.random.default_rng(sum(base_color) * 17 + sum(accent_color) * 31)
    for _ in range(900):
        x = int(rng.integers(0, 256))
        y = int(rng.integers(0, 256))
        r = int(rng.integers(1, 4))
        color = tuple(
            int(np.clip(accent_color[i] + rng.integers(-18, 19), 0, 255))
            for i in range(3)
        )
        draw.ellipse((x - r, y - r, x + r, y + r), fill=color)
    img.save(path)


def create_flat_normal(path):
    Image.new("RGB", (16, 16), (128, 128, 255)).save(path)


def write_binary_stl(path, vertices, faces):
    with open(path, "wb") as f:
        f.write(b"heightmap collision mesh".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(faces)))
        for face in faces:
            v0 = np.array(vertices[face[0]], dtype=np.float32)
            v1 = np.array(vertices[face[1]], dtype=np.float32)
            v2 = np.array(vertices[face[2]], dtype=np.float32)

            normal = np.cross(v1 - v0, v2 - v0)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-9:
                normal = normal / norm
            else:
                normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)

            f.write(struct.pack(
                "<12fH",
                float(normal[0]), float(normal[1]), float(normal[2]),
                float(v0[0]), float(v0[1]), float(v0[2]),
                float(v1[0]), float(v1[1]), float(v1[2]),
                float(v2[0]), float(v2[1]), float(v2[2]),
                0,
            ))


def create_collision_mesh(path, heightmap, args):
    rows, cols = heightmap.shape
    cell_size_x = args.terrain_x / (cols - 1)
    cell_size_y = args.terrain_y / (rows - 1)
    vertices = []
    faces = []

    for row in range(rows):
        for col in range(cols):
            x = col * cell_size_x - args.terrain_x / 2.0
            y = row * cell_size_y - args.terrain_y / 2.0
            z = float(heightmap[row, col]) / 255.0 * args.terrain_z
            vertices.append((x, y, z))

    for row in range(rows - 1):
        for col in range(cols - 1):
            top_left = row * cols + col
            top_right = top_left + 1
            bottom_left = (row + 1) * cols + col
            bottom_right = bottom_left + 1
            faces.append((top_left, top_right, bottom_left))
            faces.append((top_right, bottom_right, bottom_left))

    write_binary_stl(path, vertices, faces)
    return len(vertices), len(faces)


def create_heightmap_model(args, heightmap):
    model_dir = ensure_dir(repo_path("models", args.terrain_model))
    texture_dir = ensure_dir(os.path.join(model_dir, "materials", "textures"))

    heightmap_path = os.path.join(model_dir, "heightmap.png")
    Image.fromarray(heightmap, mode="L").save(heightmap_path)
    collision_mesh_path = os.path.join(model_dir, "collision.stl")
    create_collision_mesh(collision_mesh_path, heightmap, args)

    save_texture(os.path.join(texture_dir, "soil.png"), (104, 86, 55), (128, 111, 72))
    save_texture(os.path.join(texture_dir, "grass.png"), (73, 121, 54), (95, 145, 66))
    save_texture(os.path.join(texture_dir, "rock.png"), (118, 116, 103), (145, 142, 126))
    create_flat_normal(os.path.join(texture_dir, "flat_normal.png"))

    model_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="{args.terrain_model}">
    <static>true</static>
    <link name="terrain_link">
      <visual name="terrain_visual">
        <cast_shadows>true</cast_shadows>
        <transparency>0</transparency>
        <geometry>
          <heightmap>
            <use_terrain_paging>false</use_terrain_paging>
            <sampling>{args.sampling}</sampling>
            <texture>
              <diffuse>model://{args.terrain_model}/materials/textures/soil.png</diffuse>
              <normal>model://{args.terrain_model}/materials/textures/flat_normal.png</normal>
              <size>8</size>
            </texture>
            <texture>
              <diffuse>model://{args.terrain_model}/materials/textures/grass.png</diffuse>
              <normal>model://{args.terrain_model}/materials/textures/flat_normal.png</normal>
              <size>8</size>
            </texture>
            <texture>
              <diffuse>model://{args.terrain_model}/materials/textures/rock.png</diffuse>
              <normal>model://{args.terrain_model}/materials/textures/flat_normal.png</normal>
              <size>10</size>
            </texture>
            <blend>
              <min_height>{args.terrain_z * 0.30:.3f}</min_height>
              <fade_dist>{max(args.terrain_z * 0.08, 0.25):.3f}</fade_dist>
            </blend>
            <blend>
              <min_height>{args.terrain_z * 0.72:.3f}</min_height>
              <fade_dist>{max(args.terrain_z * 0.10, 0.25):.3f}</fade_dist>
            </blend>
            <uri>model://{args.terrain_model}/heightmap.png</uri>
            <size>{args.terrain_x:.6f} {args.terrain_y:.6f} {args.terrain_z:.6f}</size>
            <pos>0 0 0</pos>
          </heightmap>
        </geometry>
      </visual>
      <collision name="terrain_collision">
        <geometry>
          <mesh>
            <uri>model://{args.terrain_model}/collision.stl</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>100</mu2>
            </ode>
            <bullet>
              <friction>1.0</friction>
              <friction2>1.0</friction2>
              <rolling_friction>0.0</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
'''
    with open(os.path.join(model_dir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(model_sdf)

    model_config = f'''<?xml version="1.0"?>
<model>
  <name>{args.terrain_model}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Generated by generate_heightmap_world.py</name>
    <email>none</email>
  </author>
  <description>Native Gazebo heightmap terrain generated without STL meshes.</description>
</model>
'''
    with open(os.path.join(model_dir, "model.config"), "w", encoding="utf-8") as f:
        f.write(model_config)

    return model_dir


def write_ground_truth(model_dir, heightmap, args):
    gt_path = os.path.join(model_dir, "terrain_ground_truth.csv")
    rows, cols = heightmap.shape
    cell_size_x = args.terrain_x / (cols - 1) if cols > 1 else 0.0
    cell_size_y = args.terrain_y / (rows - 1) if rows > 1 else 0.0

    with open(gt_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["row", "col", "x_m", "y_m", "z_m"])
        for row in range(rows):
            for col in range(cols):
                x_m = col * cell_size_x - args.terrain_x / 2.0
                y_m = row * cell_size_y - args.terrain_y / 2.0
                z_m = float(heightmap[row, col]) / 255.0 * args.terrain_z
                writer.writerow([row, col, round(x_m, 6), round(y_m, 6), round(z_m, 6)])
    return gt_path


def include_block(uri, name, pose):
    return f'''    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{pose}</pose>
    </include>'''


def create_world(args, drone_z, platform_z, tree_poses):
    tree_blocks = [
        include_block(
            f"model://{args.tree_model}",
            f"{args.tree_model}_{idx:02d}",
            f"{x} {y} {z} 0 0 {yaw}",
        )
        for idx, (x, y, z, yaw) in enumerate(tree_poses)
    ]
    trees_sdf = "\n\n".join(tree_blocks)

    world_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="{args.world}">
    <physics type="ignored" name="default_physics">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.68 0.74 0.80 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 25 0 0 0</pose>
      <diffuse>0.82 0.80 0.74 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <attenuation>
        <range>10000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.45 0.20 -0.87</direction>
    </light>

    <include>
      <uri>model://{args.terrain_model}</uri>
      <name>{args.terrain_model}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <model name="drone_platform">
      <pose>{args.platform_x} {args.platform_y} {platform_z} 0 0 0</pose>
      <static>true</static>
      <link name="platform_link">
        <visual name="platform_visual">
          <geometry>
            <box>
              <size>4.0 4.0 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.44 0.44 0.40 1</ambient>
            <diffuse>0.58 0.58 0.52 1</diffuse>
            <specular>0.18 0.18 0.16 1</specular>
          </material>
        </visual>
        <collision name="platform_collision">
          <geometry>
            <box>
              <size>4.0 4.0 0.5</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
              <bullet>
                <friction>1.0</friction>
                <friction2>1.0</friction2>
                <rolling_friction>0.0</rolling_friction>
              </bullet>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <include>
      <uri>model://iris_with_ardupilot</uri>
      <name>iris_with_ardupilot</name>
      <pose>{args.drone_x} {args.drone_y} {drone_z} 0 0 0</pose>
    </include>

{trees_sdf}

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <wind>
      <linear_velocity>0 0 0</linear_velocity>
    </wind>
  </world>
</sdf>
'''
    worlds_dir = ensure_dir(repo_path("worlds"))
    world_path = os.path.join(worlds_dir, f"{args.world}.sdf")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(world_sdf)
    return world_path


def write_manifest(model_dir, args, seed, drone_z, platform_z, tree_poses):
    manifest = {
        "generator": "generate_heightmap_world.py",
        "generated_at": datetime.datetime.utcnow().isoformat() + "Z",
        "seed": seed,
        "terrain_representation": "sdf_heightmap",
        "generator_params": vars(args),
        "terrain_xy_extents_m": {"x": args.terrain_x, "y": args.terrain_y},
        "terrain_z_scale_m": args.terrain_z,
        "vehicle_spawn": {"x_m": args.drone_x, "y_m": args.drone_y, "z_m": drone_z},
        "platform_spawn": {"x_m": args.platform_x, "y_m": args.platform_y, "z_m": platform_z},
        "vegetation": {
            "tree_model": args.tree_model,
            "tree_count": len(tree_poses),
            "tree_exclusion_radius_m": args.tree_exclusion_radius,
            "tree_poses": tree_poses,
        },
        "canonical_file_paths": {
            "heightmap_model_dir": model_dir,
            "heightmap_png": os.path.join(model_dir, "heightmap.png"),
            "collision_stl": os.path.join(model_dir, "collision.stl"),
            "model_sdf": os.path.join(model_dir, "model.sdf"),
            "ground_truth_csv": os.path.join(model_dir, "terrain_ground_truth.csv"),
            "world_sdf": repo_path("worlds", f"{args.world}.sdf"),
        },
    }
    manifest_path = os.path.join(model_dir, "terrain_manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return manifest_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate the canonical Gazebo world with native heightmap terrain."
    )
    parser.add_argument("--world", default="random_terrain_ardupilot")
    parser.add_argument("--terrain-model", default="terrain_heightmap")
    parser.add_argument("--size", type=int, default=257)
    parser.add_argument("--scale", type=float, default=28.0)
    parser.add_argument("--octaves", type=int, default=4)
    parser.add_argument("--persistence", type=float, default=0.42)
    parser.add_argument("--lacunarity", type=float, default=2.0)
    parser.add_argument("--smooth-passes", type=int, default=3)
    parser.add_argument("--terrain-x", type=float, default=100.0)
    parser.add_argument("--terrain-y", type=float, default=100.0)
    parser.add_argument("--terrain-z", type=float, default=8.0)
    parser.add_argument("--sampling", type=int, default=2)
    parser.add_argument("--seed", type=int, default=5001)
    parser.add_argument("--drone-x", type=float, default=-44.0)
    parser.add_argument("--drone-y", type=float, default=-44.0)
    parser.add_argument("--drone-z", type=float, default=None)
    parser.add_argument("--platform-x", type=float, default=-44.0)
    parser.add_argument("--platform-y", type=float, default=-44.0)
    parser.add_argument("--platform-z", type=float, default=None)
    parser.add_argument("--tree-model", default="pine_tree")
    parser.add_argument("--tree-count", type=int, default=0)
    parser.add_argument("--tree-exclusion-radius", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()

    print("Generating native Gazebo heightmap world")
    print(f"Repo: {REPO_ROOT}")
    print(f"World: {args.world}")
    print(f"Terrain model: {args.terrain_model}")

    heightmap, seed = generate_heightmap(
        args.size,
        args.scale,
        args.octaves,
        args.persistence,
        args.lacunarity,
        args.seed,
        args.smooth_passes,
    )

    worlds_heightmap = repo_path("worlds", "random_terrain_heightmap.png")
    Image.fromarray(heightmap, mode="L").save(worlds_heightmap)

    terrain_at_platform = terrain_height_at(
        heightmap,
        args.platform_x,
        args.platform_y,
        args.terrain_x,
        args.terrain_y,
        args.terrain_z,
    )
    platform_z = args.platform_z if args.platform_z is not None else terrain_at_platform + 0.25
    drone_z = args.drone_z if args.drone_z is not None else platform_z + 0.47
    platform_z = round(platform_z, 3)
    drone_z = round(drone_z, 3)

    tree_poses = generate_tree_poses(
        heightmap,
        seed,
        args.tree_count,
        args.terrain_x,
        args.terrain_y,
        args.terrain_z,
        args.platform_x,
        args.platform_y,
        args.tree_exclusion_radius,
    )

    model_dir = create_heightmap_model(args, heightmap)
    gt_path = write_ground_truth(model_dir, heightmap, args)
    world_path = create_world(args, drone_z, platform_z, tree_poses)
    manifest_path = write_manifest(model_dir, args, seed, drone_z, platform_z, tree_poses)

    print(f"Heightmap PNG: {os.path.join(model_dir, 'heightmap.png')}")
    print(f"Ground truth: {gt_path}")
    print(f"Manifest: {manifest_path}")
    print(f"World SDF: {world_path}")
    print(f"Spawn Z: drone={drone_z}, platform={platform_z}")
    print("Done. Launch with ./run_with_ardupilot.sh")


if __name__ == "__main__":
    main()
