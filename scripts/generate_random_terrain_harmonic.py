#!/usr/bin/env python3
"""
Генерация случайного рельефа для Gazebo Harmonic с поддержкой ArduPilot
Создаёт heightmap и terrain для симуляции дрона с лидаром

Canonical output:
  - models/terrain_mesh/model_visual.stl — active visual mesh used by Gazebo
  - models/terrain_mesh/model_collision.stl — collision mesh used by Gazebo
  - models/terrain_mesh/model.sdf      — SDF model descriptor
  - models/terrain_mesh/model.config   — Gazebo model config
  - models/terrain_mesh/terrain_manifest.json  — machine-readable manifest
  - models/terrain_mesh/terrain_ground_truth.csv — machine-readable ground truth
  - worlds/<name>.png                  — heightmap image
  - worlds/<name>.sdf                  — world file (unless --no-world)
"""
import numpy as np
from PIL import Image
from opensimplex import OpenSimplex
import argparse
import os
import json
import csv
import datetime
import struct

# ---------------------------------------------------------------------------
# Repo-relative path resolution (robust, no home-directory assumptions)
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(_SCRIPT_DIR)  # scripts/ is a direct child of repo root


def get_repo_root():
    """Return the absolute path to the repository root."""
    return REPO_ROOT


def smooth_heightmap(heightmap, passes=2):
    """Apply a small weighted blur to remove unrealistic needle-like terrain."""
    smoothed = heightmap.astype(np.float32)
    for _ in range(max(0, passes)):
        padded = np.pad(smoothed, 1, mode='edge')
        smoothed = (
            padded[:-2, :-2] + 2 * padded[:-2, 1:-1] + padded[:-2, 2:] +
            2 * padded[1:-1, :-2] + 4 * padded[1:-1, 1:-1] + 2 * padded[1:-1, 2:] +
            padded[2:, :-2] + 2 * padded[2:, 1:-1] + padded[2:, 2:]
        ) / 16.0
    return smoothed


def generate_random_heightmap(size=513, scale=50.0, octaves=4, persistence=0.5,
                              lacunarity=2.0, seed=None, smooth_passes=2):
    """
    Генерация случайной heightmap используя OpenSimplex noise

    Args:
        size: Размер heightmap (должно быть 2^n + 1 для лучшей совместимости)
        scale: Масштаб шума (меньше = более детальный рельеф)
        octaves: Количество октав шума (больше = более детальный)
        persistence: Персистентность (амплитуда каждой следующей октавы)
        lacunarity: Лакунарность (частота каждой следующей октавы)
        seed: Seed для генерации (None = случайный)
        smooth_passes: Количество проходов сглаживания после генерации шума

    Returns:
        heightmap: numpy массив uint8 (0-255)
        used_seed: использованный seed
    """
    if seed is None:
        seed = np.random.randint(0, 10000)

    simplex = OpenSimplex(seed)
    heightmap = np.zeros((size, size))

    for y in range(size):
        for x in range(size):
            nx, ny = x / scale, y / scale
            amplitude = 1.0
            frequency = 1.0
            noise_value = 0.0

            for _ in range(octaves):
                noise_value += simplex.noise2(nx * frequency, ny * frequency) * amplitude
                amplitude *= persistence
                frequency *= lacunarity

            heightmap[y, x] = noise_value

    heightmap = smooth_heightmap(heightmap, smooth_passes)

    # Нормализация к диапазону 0-255
    height_range = heightmap.max() - heightmap.min()
    if height_range <= 1e-9:
        heightmap = np.zeros_like(heightmap)
    else:
        heightmap = (heightmap - heightmap.min()) / height_range * 255

    return heightmap.astype(np.uint8), seed


def terrain_height_at(heightmap, x_m, y_m, scale_x=100.0, scale_y=100.0, scale_z=8.0):
    """Bilinearly sample terrain height in world coordinates."""
    rows, cols = heightmap.shape
    u = (x_m + scale_x / 2.0) / scale_x * (cols - 1)
    v = (y_m + scale_y / 2.0) / scale_y * (rows - 1)
    u = float(np.clip(u, 0, cols - 1))
    v = float(np.clip(v, 0, rows - 1))

    x0 = int(np.floor(u))
    y0 = int(np.floor(v))
    x1 = min(x0 + 1, cols - 1)
    y1 = min(y0 + 1, rows - 1)
    tx = u - x0
    ty = v - y0

    h00 = heightmap[y0, x0]
    h10 = heightmap[y0, x1]
    h01 = heightmap[y1, x0]
    h11 = heightmap[y1, x1]
    h0 = h00 * (1.0 - tx) + h10 * tx
    h1 = h01 * (1.0 - tx) + h11 * tx
    return float((h0 * (1.0 - ty) + h1 * ty) / 255.0 * scale_z)


def generate_tree_poses(heightmap, seed, count=18, scale_z=8.0,
                        platform_x=-44.0, platform_y=-44.0,
                        exclusion_radius=10.0, terrain_margin=8.0,
                        scale_x=100.0, scale_y=100.0):
    """Generate deterministic tree poses away from the launch platform."""
    if count <= 0:
        return []

    rng = np.random.default_rng((seed if seed is not None else 0) + 1009)
    poses = []
    attempts = 0
    max_attempts = count * 80
    min_spacing = 5.0

    while len(poses) < count and attempts < max_attempts:
        attempts += 1
        x_m = rng.uniform(-scale_x / 2.0 + terrain_margin, scale_x / 2.0 - terrain_margin)
        y_m = rng.uniform(-scale_y / 2.0 + terrain_margin, scale_y / 2.0 - terrain_margin)

        if np.hypot(x_m - platform_x, y_m - platform_y) < exclusion_radius:
            continue
        if any(np.hypot(x_m - px, y_m - py) < min_spacing for px, py, _, _ in poses):
            continue

        z_m = terrain_height_at(heightmap, x_m, y_m, scale_x, scale_y, scale_z)
        yaw = rng.uniform(-np.pi, np.pi)
        poses.append((round(x_m, 3), round(y_m, 3), round(z_m, 3), round(yaw, 6)))

    return poses


def _write_binary_stl(output_stl_path, vertices, faces, double_sided=False):
    """Write a binary STL. When double_sided is true, emit both face windings."""
    stl_faces = list(faces)
    if double_sided:
        stl_faces.extend((face[0], face[2], face[1]) for face in faces)

    with open(output_stl_path, 'wb') as f:
        header = b"terrain generated by generate_random_terrain_harmonic.py"
        f.write(header.ljust(80, b"\0")[:80])
        f.write(struct.pack("<I", len(stl_faces)))
        for face in stl_faces:
            v0 = np.array(vertices[face[0]])
            v1 = np.array(vertices[face[1]])
            v2 = np.array(vertices[face[2]])

            edge1 = v1 - v0
            edge2 = v2 - v0
            normal = np.cross(edge1, edge2)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            else:
                normal = np.array([0, 0, 1], dtype=np.float32)

            f.write(struct.pack(
                "<12fH",
                float(normal[0]), float(normal[1]), float(normal[2]),
                float(v0[0]), float(v0[1]), float(v0[2]),
                float(v1[0]), float(v1[1]), float(v1[2]),
                float(v2[0]), float(v2[1]), float(v2[2]),
                0,
            ))


def generate_terrain_mesh_from_heightmap(heightmap_path, output_stl_path, downsample_skip=1,
                                         scale_x=100.0, scale_y=100.0, scale_z=12.0,
                                         collision_stl_path=None):
    """
    Создание mesh из heightmap для Gazebo Harmonic

    Returns:
        (num_vertices, num_faces, height_data_mesh, downsample_skip, scale_x, scale_y, scale_z)
    """
    img = Image.open(heightmap_path).convert('L')
    height_data = np.array(img, dtype=np.float32) / 255.0

    # Уменьшаем разрешение для производительности
    height_data_ds = height_data[::downsample_skip, ::downsample_skip]

    rows, cols = height_data_ds.shape

    cell_size_x = scale_x / (cols - 1)
    cell_size_y = scale_y / (rows - 1)

    vertices = []
    faces = []

    # Генерация вершин
    for i in range(rows):
        for j in range(cols):
            x = j * cell_size_x - scale_x / 2
            y = i * cell_size_y - scale_y / 2
            z = height_data_ds[i, j] * scale_z
            vertices.append((x, y, z))

    # Генерация граней
    for i in range(rows - 1):
        for j in range(cols - 1):
            top_left = i * cols + j
            top_right = top_left + 1
            bottom_left = (i + 1) * cols + j
            bottom_right = bottom_left + 1

            faces.append((top_left, top_right, bottom_left))
            faces.append((top_right, bottom_right, bottom_left))

    # Gazebo / Ogre can disagree about STL front-face winding. The visual STL is
    # emitted with both windings so the terrain remains opaque from above and below.
    _write_binary_stl(output_stl_path, vertices, faces, double_sided=True)
    if collision_stl_path:
        _write_binary_stl(collision_stl_path, vertices, faces, double_sided=False)

    return len(vertices), len(faces), height_data_ds, downsample_skip, scale_x, scale_y, scale_z

def create_terrain_model(base_dir, heightmap_path, model_name="terrain_mesh",
                          generator_params=None, used_seed=None):
    """
    Создание модели terrain для Gazebo Harmonic.
    Также пишет manifest и ground-truth artifact.
    """
    model_dir = os.path.join(base_dir, "models", model_name)
    os.makedirs(model_dir, exist_ok=True)

    stl_path = os.path.join(model_dir, "model_visual.stl")
    collision_stl_path = os.path.join(model_dir, "model_collision.stl")

    print(f"🔨 Генерация mesh из heightmap...")
    mesh_downsample_skip = generator_params.get("mesh_downsample_skip", 1) if generator_params else 1
    terrain_z_scale = generator_params.get("terrain_z", 12.0) if generator_params else 12.0
    num_vertices, num_faces, height_data_mesh, downsample_skip, scale_x, scale_y, scale_z = \
        generate_terrain_mesh_from_heightmap(
            heightmap_path,
            stl_path,
            downsample_skip=mesh_downsample_skip,
            scale_z=terrain_z_scale,
            collision_stl_path=collision_stl_path,
        )

    # Создаём model.sdf для Gazebo Harmonic (SDF 1.10)
    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="{model_name}">
    <static>true</static>
    <link name="terrain_link">
      <visual name="terrain_visual">
        <transparency>0</transparency>
        <cast_shadows>true</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://{model_name}/model_visual.stl</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.35 0.55 0.25 1</ambient>
          <diffuse>0.35 0.55 0.25 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <double_sided>true</double_sided>
        </material>
      </visual>
      <collision name="terrain_collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/model_collision.stl</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>100</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>'''

    with open(os.path.join(model_dir, "model.sdf"), 'w') as f:
        f.write(sdf_content)

    # Создаём model.config
    config_content = f'''<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Generated by generate_random_terrain_harmonic.py</name>
    <email>none</email>
  </author>
  <description>Terrain mesh generated from heightmap for Gazebo Harmonic</description>
</model>'''

    with open(os.path.join(model_dir, "model.config"), 'w') as f:
        f.write(config_content)

    # ------------------------------------------------------------------
    # Write machine-readable manifest
    # ------------------------------------------------------------------
    rows_raw, cols_raw = height_data_mesh.shape
    terrain_extent_x = scale_x
    terrain_extent_y = scale_y
    # Grid coordinates: center of each cell in the downsampled grid
    cols_ds = cols_raw  # already the downsampled dimension
    rows_ds = rows_raw

    manifest = {
        "generator": "generate_random_terrain_harmonic.py",
        "generated_at": datetime.datetime.utcnow().isoformat() + "Z",
        "seed": used_seed,
        "generator_params": {
            "size": generator_params.get("size", 257) if generator_params else 257,
            "scale": generator_params.get("scale", 28.0) if generator_params else 28.0,
            "octaves": generator_params.get("octaves", 4) if generator_params else 4,
            "persistence": generator_params.get("persistence", 0.42) if generator_params else 0.42,
            "lacunarity": generator_params.get("lacunarity", 2.0) if generator_params else 2.0,
            "smooth_passes": generator_params.get("smooth_passes", 3) if generator_params else 3,
            "mesh_downsample_skip": downsample_skip,
            "terrain_z": scale_z,
        },
        "mesh_downsample_skip": downsample_skip,
        "terrain_extents": {
            "x_min": -scale_x / 2,
            "x_max": scale_x / 2,
            "y_min": -scale_y / 2,
            "y_max": scale_y / 2,
            "z_min": 0.0,
            "z_max": scale_z,
        },
        "terrain_xy_extents_m": {"x": scale_x, "y": scale_y},
        "terrain_z_scale_m": scale_z,
        "grid_shape_raw": {"rows": rows_raw, "cols": cols_raw},
        "world_name": generator_params.get("world", "random_terrain_ardupilot") if generator_params else "random_terrain_ardupilot",
        "vehicle_spawn": {
            "x_m": generator_params.get("drone_x", -44.0) if generator_params else -44.0,
            "y_m": generator_params.get("drone_y", -44.0) if generator_params else -44.0,
            "z_m": generator_params.get("drone_z", 13.46) if generator_params else 13.46,
        },
        "platform_spawn": {
            "x_m": generator_params.get("platform_x", -44.0) if generator_params else -44.0,
            "y_m": generator_params.get("platform_y", -44.0) if generator_params else -44.0,
            "z_m": generator_params.get("platform_z", 13.0) if generator_params else 13.0,
        },
        "vehicle_spawn_height_m": generator_params.get("drone_z", 13.46) if generator_params else 13.46,
        "vegetation": {
            "tree_model": generator_params.get("tree_model", "pine_tree") if generator_params else "pine_tree",
            "tree_count": generator_params.get("tree_count", 0) if generator_params else 0,
            "tree_exclusion_radius_m": generator_params.get("tree_exclusion_radius", 10.0) if generator_params else 10.0,
            "tree_poses": generator_params.get("tree_poses", []) if generator_params else [],
        },
        "canonical_file_paths": {
            "visual_stl_mesh": os.path.join(model_dir, "model_visual.stl"),
            "collision_stl_mesh": os.path.join(model_dir, "model_collision.stl"),
            "model_sdf": os.path.join(model_dir, "model.sdf"),
            "model_config": os.path.join(model_dir, "model.config"),
            "manifest": os.path.join(model_dir, "terrain_manifest.json"),
            "ground_truth_csv": os.path.join(model_dir, "terrain_ground_truth.csv"),
        },
    }

    manifest_path = os.path.join(model_dir, "terrain_manifest.json")
    with open(manifest_path, 'w') as f:
        json.dump(manifest, f, indent=2)
    print(f"✅ Manifest saved: {manifest_path}")

    # ------------------------------------------------------------------
    # Write machine-readable ground-truth terrain artifact
    #
    # Format: CSV with columns row, col, x_m, y_m, z_m
    # - row, col: 0-based indices into the downsampled height grid
    # - x_m, y_m: world coordinates in meters (origin at terrain center)
    # - z_m: terrain height in meters (0..scale_z)
    #
    # The grid covers [x_min, x_max] × [y_min, y_max] uniformly.
    # Cell (row, col) corresponds to the terrain point at that grid position.
    # This matches the vertex layout used to build the STL mesh.
    # ------------------------------------------------------------------
    gt_path = os.path.join(model_dir, "terrain_ground_truth.csv")
    cell_size_x = scale_x / (cols_raw - 1) if cols_raw > 1 else 0
    cell_size_y = scale_y / (rows_raw - 1) if rows_raw > 1 else 0

    with open(gt_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["row", "col", "x_m", "y_m", "z_m"])
        for i in range(rows_raw):
            for j in range(cols_raw):
                x_m = j * cell_size_x - scale_x / 2
                y_m = i * cell_size_y - scale_y / 2
                z_m = float(height_data_mesh[i, j]) * scale_z
                writer.writerow([i, j, round(x_m, 6), round(y_m, 6), round(z_m, 6)])
    print(f"✅ Ground truth saved: {gt_path} ({rows_raw * cols_raw} cells)")

    print(f"✅ Модель terrain создана: {model_dir} ({num_vertices} вершин, {num_faces} граней)")
    return model_dir

def create_gazebo_harmonic_world(base_dir, world_name="random_terrain",
                                 drone_pose_x=-44.0, drone_pose_y=-44.0, drone_pose_z=13.46,
                                 platform_pose_x=-44.0, platform_pose_y=-44.0, platform_pose_z=13.0,
                                 terrain_model_name="terrain_mesh",
                                 tree_model_name="pine_tree", tree_poses=None):
    """
    Создание SDF файла мира для Gazebo Harmonic
    """
    world_path = os.path.join(base_dir, "worlds", f"{world_name}.sdf")
    stl_path = os.path.join(base_dir, "models", terrain_model_name, "model.stl")
    tree_poses = tree_poses or []
    tree_includes = []
    for index, (tree_x, tree_y, tree_z, tree_yaw) in enumerate(tree_poses):
        tree_includes.append(f'''    <include>
      <uri>model://{tree_model_name}</uri>
      <name>{tree_model_name}_{index:02d}</name>
      <pose>{tree_x} {tree_y} {tree_z} 0 0 {tree_yaw}</pose>
    </include>''')
    tree_include_block = "\n\n".join(tree_includes)

    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="{world_name}">
    <!-- Physics settings for Gazebo Harmonic -->
    <physics type="dart" name="default_physics">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
      <sky>
        <atmosphere type="adiabatic"/>
      </sky>
    </scene>

    <!-- Sun light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>10000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <!-- Keep the fallback plane below the terrain mesh so it does not visually
           cut through valleys or make the terrain look translucent. -->
      <pose>0 0 -2.0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
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
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Terrain mesh (use include for proper loading) -->
    <include>
      <uri>model://{terrain_model_name}</uri>
      <name>terrain_mesh</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- Drone platform in the terrain corner.
         Platform top is z + 0.25. The drone is spawned just above it to avoid startup bounce. -->
    <model name="drone_platform">
      <pose>{platform_pose_x} {platform_pose_y} {platform_pose_z} 0 0 0</pose>
      <static>true</static>
      <link name="platform_link">
        <visual name="platform_visual">
          <geometry>
            <box>
              <size>4.0 4.0 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
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

    <!-- Drone with ArduPilot plugin at specified height -->
    <include>
      <uri>model://iris_with_ardupilot</uri>
      <name>iris_with_ardupilot</name>
      <pose>{drone_pose_x} {drone_pose_y} {drone_pose_z} 0 0 0</pose>
    </include>

    <!-- Sparse vegetation for visual context and LiDAR obstacles. -->
{tree_include_block if tree_include_block else '    <!-- Vegetation disabled: tree_count is 0. -->'}

    <!-- Spherical coordinates -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- Wind for realistic flight simulation -->
    <wind>
      <linear_velocity>0 0 0</linear_velocity>
    </wind>
  </world>
</sdf>'''

    with open(world_path, 'w') as f:
        f.write(sdf_content)

    print(f"✅ Мир сохранён: {world_path}")
    return world_path

def print_launch_instructions(base_dir, world_path):
    """
    Вывод инструкций для запуска Gazebo Harmonic
    """
    print("\n" + "="*60)
    print("🚀 Для запуска Gazebo Harmonic выполните:")
    print("="*60)
    print(f"""
# 1. Установите Gazebo Harmonic (если ещё не установлен):
   # Добавьте репозиторий OSRF
   sudo apt-get update
   sudo apt-get install -y lsb-release wget gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   sudo apt-get update
   sudo apt-get install -y gz-harmonic libgz-harmonic-dev

# 2. Запустите Gazebo Harmonic с canonical script:
   cd {base_dir}
   ./run_with_ardupilot.sh

# 3. Для запуска ArduPilot SITL:
   # В другом терминале:
   cd {base_dir}
   ./run_ardupilot_sitl.sh

# 4. Ручной запуск Gazebo, если нужен:
   export GZ_SIM_SYSTEM_PLUGIN_PATH={os.path.dirname(base_dir)}/ardupilot_gazebo/build:${{GZ_SIM_SYSTEM_PLUGIN_PATH:-}}
   export GZ_SIM_RESOURCE_PATH={base_dir}/worlds:{base_dir}/models:${{GZ_SIM_RESOURCE_PATH:-}}
   gz sim -v4 -r --physics-engine gz-physics-bullet-featherstone-plugin {world_path}
""")

def main():
    parser = argparse.ArgumentParser(
        description='Генерация случайного рельефа для Gazebo Harmonic',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры использования:
  %(prog)s --seed 42 --world my_terrain
  %(prog)s --size 257 --scale 28 --octaves 4
  %(prog)s --drone-x -44 --drone-y -44 --output custom_terrain.png
        """
    )
    parser.add_argument('--size', type=int, default=257,
                        help='Размер heightmap (должно быть 2^n + 1, по умолчанию 257)')
    parser.add_argument('--scale', type=float, default=28.0,
                        help='Масштаб шума (меньше = более частый рельеф, по умолчанию 28.0)')
    parser.add_argument('--octaves', type=int, default=4,
                        help='Количество октав шума (по умолчанию 4)')
    parser.add_argument('--persistence', type=float, default=0.42,
                        help='Персистентность (по умолчанию 0.42)')
    parser.add_argument('--lacunarity', type=float, default=2.0,
                        help='Лакунарность (по умолчанию 2.0)')
    parser.add_argument('--smooth-passes', type=int, default=3,
                        help='Количество проходов сглаживания heightmap (по умолчанию 3)')
    parser.add_argument('--mesh-downsample-skip', type=int, default=2,
                        help='Шаг прореживания mesh; 1 = максимальная детализация, 2 = быстрее загрузка (по умолчанию 2)')
    parser.add_argument('--terrain-z', type=float, default=8.0,
                        help='Вертикальный масштаб рельефа в метрах (по умолчанию 8.0)')
    parser.add_argument('--seed', type=int, default=None,
                        help='Seed для генерации (по умолчанию случайный)')
    parser.add_argument('--output', type=str, default='random_terrain.png',
                        help='Имя выходного файла heightmap (по умолчанию random_terrain.png)')
    parser.add_argument('--world', type=str, default='random_terrain_ardupilot',
                        help='Имя мира Gazebo (по умолчанию random_terrain_ardupilot — canonical world)')
    parser.add_argument('--drone-x', type=float, default=-44.0,
                        help='Начальная X-позиция дрона в метрах (по умолчанию -44.0)')
    parser.add_argument('--drone-y', type=float, default=-44.0,
                        help='Начальная Y-позиция дрона в метрах (по умолчанию -44.0)')
    parser.add_argument('--drone-z', type=float, default=None,
                        help='Начальная высота дрона в метрах (по умолчанию: platform_z + 0.47)')
    parser.add_argument('--platform-x', type=float, default=-44.0,
                        help='X-позиция платформы в метрах (по умолчанию -44.0)')
    parser.add_argument('--platform-y', type=float, default=-44.0,
                        help='Y-позиция платформы в метрах (по умолчанию -44.0)')
    parser.add_argument('--platform-z', type=float, default=None,
                        help='Высота центра платформы в метрах (по умолчанию: terrain height + 0.25)')
    parser.add_argument('--terrain-model', type=str, default='terrain_mesh',
                        help='Имя модели terrain (по умолчанию terrain_mesh)')
    parser.add_argument('--tree-model', type=str, default='pine_tree',
                        help='Имя модели деревьев Gazebo (по умолчанию pine_tree)')
    parser.add_argument('--tree-count', type=int, default=18,
                        help='Количество деревьев в мире; 0 отключает растительность (по умолчанию 18)')
    parser.add_argument('--tree-exclusion-radius', type=float, default=10.0,
                        help='Радиус зоны без деревьев вокруг стартовой платформы, м (по умолчанию 10.0)')
    parser.add_argument('--no-world', action='store_true',
                        help='Не создавать мир, только heightmap и terrain модель')

    args = parser.parse_args()

    # Use repo-root-relative paths — no home-directory assumptions
    base_dir = get_repo_root()

    print(f"\n🎯 Gazebo Harmonic - Генерация случайного рельефа")
    print(f"📁 Базовая директория: {base_dir}")
    print(f"🎲 Параметры генерации:")
    print(f"   - Размер heightmap: {args.size}x{args.size}")
    print(f"   - Scale: {args.scale}, Octaves: {args.octaves}")
    print(f"   - Persistence: {args.persistence}, Lacunarity: {args.lacunarity}, Smooth passes: {args.smooth_passes}")
    print(f"   - Mesh downsample skip: {args.mesh_downsample_skip}, Terrain Z: {args.terrain_z}")
    print(f"   - Spawn XY: drone=({args.drone_x}, {args.drone_y}), "
          f"platform=({args.platform_x}, {args.platform_y})")
    print(f"   - Trees: model={args.tree_model}, count={args.tree_count}, "
          f"exclusion radius={args.tree_exclusion_radius}")
    print(f"   - Seed: {args.seed if args.seed else 'случайный'}")
    print()

    # Генерация heightmap
    print(f"🎨 Генерация heightmap...")
    heightmap, used_seed = generate_random_heightmap(
        size=args.size,
        scale=args.scale,
        octaves=args.octaves,
        persistence=args.persistence,
        lacunarity=args.lacunarity,
        seed=args.seed,
        smooth_passes=args.smooth_passes
    )

    # Сохранение heightmap
    worlds_dir = os.path.join(base_dir, "worlds")
    os.makedirs(worlds_dir, exist_ok=True)
    output_path = os.path.join(worlds_dir, args.output)
    img = Image.fromarray(heightmap)
    img.save(output_path)
    print(f"✅ Heightmap сохранена: {output_path} (seed={used_seed})")

    platform_z = args.platform_z
    if platform_z is None:
        platform_z = terrain_height_at(
            heightmap,
            args.platform_x,
            args.platform_y,
            scale_z=args.terrain_z,
        ) + 0.25

    drone_z = args.drone_z
    if drone_z is None:
        drone_z = platform_z + 0.47

    platform_z = round(platform_z, 3)
    drone_z = round(drone_z, 3)
    print(f"✅ Spawn Z resolved: drone={drone_z}, platform={platform_z}")

    tree_poses = generate_tree_poses(
        heightmap,
        used_seed,
        count=args.tree_count,
        scale_z=args.terrain_z,
        platform_x=args.platform_x,
        platform_y=args.platform_y,
        exclusion_radius=args.tree_exclusion_radius,
    )
    print(f"✅ Сгенерированы позиции деревьев: {len(tree_poses)}")

    # Передаём параметры для manifest
    gen_params = {
        "size": args.size,
        "scale": args.scale,
        "octaves": args.octaves,
        "persistence": args.persistence,
        "lacunarity": args.lacunarity,
        "smooth_passes": args.smooth_passes,
        "mesh_downsample_skip": args.mesh_downsample_skip,
        "terrain_z": args.terrain_z,
        "world": args.world,
        "drone_x": args.drone_x,
        "drone_y": args.drone_y,
        "drone_z": drone_z,
        "platform_x": args.platform_x,
        "platform_y": args.platform_y,
        "platform_z": platform_z,
        "tree_model": args.tree_model,
        "tree_count": args.tree_count,
        "tree_exclusion_radius": args.tree_exclusion_radius,
        "tree_poses": tree_poses,
    }

    # Создание модели terrain (включая manifest + ground truth)
    model_dir = create_terrain_model(base_dir, output_path, args.terrain_model,
                                      generator_params=gen_params, used_seed=used_seed)

    # Создание мира Gazebo (если не указан --no-world)
    if not args.no_world:
        world_path = create_gazebo_harmonic_world(
            base_dir,
            args.world,
            args.drone_x,
            args.drone_y,
            drone_z,
            args.platform_x,
            args.platform_y,
            platform_z,
            args.terrain_model,
            args.tree_model,
            tree_poses
        )
        print_launch_instructions(base_dir, world_path)
    else:
        print("\n💡 Мир не создан (указан флаг --no-world)")
        print(f"   Используйте созданную модель: {model_dir}")

if __name__ == "__main__":
    main()
