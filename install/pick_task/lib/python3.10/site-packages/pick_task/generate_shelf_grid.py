#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import math
import csv
import argparse
import sys

# ==========================================
# FUNZIONI MATEMATICHE (Quaternioni e Trasformazioni)
# ==========================================
def quat_from_rpy(roll: float, pitch: float, yaw: float) -> list:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [x, y, z, w]

def quat_multiply(q1: list, q2: list) -> list:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]

def rotate_vector_by_quat(v: list, q: list) -> list:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return [rx, ry, rz]

def combine_transforms(pos1, quat1, pos2, quat2):
    rotated_pos2 = rotate_vector_by_quat(pos2, quat1)
    new_pos = [
        pos1[0] + rotated_pos2[0],
        pos1[1] + rotated_pos2[1],
        pos1[2] + rotated_pos2[2],
    ]
    new_quat = quat_multiply(quat1, quat2)
    return new_pos, new_quat

def parse_pose_string(pose_str: str):
    vals = [float(v) for v in pose_str.strip().split()]
    pos = vals[0:3]
    quat = quat_from_rpy(vals[3], vals[4], vals[5])
    return pos, quat

# ==========================================
# PARSER WORLD E SDF
# ==========================================
def get_model_pose_from_world(world_path: str, model_name: str):
    try:
        tree = ET.parse(world_path)
        root = tree.getroot()
    except Exception as e:
        print(f"[ERRORE] Impossibile leggere il file world: {e}")
        sys.exit(1)

    # CASO 1: Cerca il modello se definito direttamente come <model name="s3_bookshelf">
    for model in root.findall('.//model'):
        if model.get('name') == model_name:
            pose_elem = model.find('pose')
            if pose_elem is not None:
                print(f"[OK] Posa base trovata per '{model_name}' nel tag <model>.")
                return parse_pose_string(pose_elem.text)
            else:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]

    # CASO 2: Cerca il modello se importato tramite <include>
    for include in root.findall('.//include'):
        name_elem = include.find('name')
        if name_elem is not None and name_elem.text == model_name:
            pose_elem = include.find('pose')
            if pose_elem is not None:
                print(f"[OK] Posa base trovata per '{model_name}' nel tag <include>.")
                return parse_pose_string(pose_elem.text)
            else:
                # Se l'include non specifica la posa, Gazebo assume che sia all'origine
                print(f"[AVVISO] Tag <pose> non trovato nell'include di '{model_name}'. Uso l'origine (0,0,0).")
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    
    print(f"[ERRORE] Modello '{model_name}' non trovato nel file world (né come <model> né come <include>).")
    sys.exit(1)

def generate_csv_from_sdf(sdf_path: str, base_pos: list, base_quat: list, can_diameter: float, output_csv: str):
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
    except Exception as e:
        print(f"[ERRORE] Impossibile leggere il file SDF: {e}")
        sys.exit(1)

    cells = []
    cell_id = 1
    shelf_id = 1
    radius = can_diameter / 2.0

    links = root.findall('.//link')
    for link in links:
        link_pose_elem = link.find('pose')
        if link_pose_elem is not None:
            link_local_pos, link_local_quat = parse_pose_string(link_pose_elem.text)
        else:
            link_local_pos = [0.0, 0.0, 0.0]
            link_local_quat = [0.0, 0.0, 0.0, 1.0]

        link_world_pos, link_world_quat = combine_transforms(
            base_pos, base_quat, link_local_pos, link_local_quat
        )

        collisions = link.findall('.//collision')
        for coll in collisions:
            box_elem = coll.find('.//geometry/box/size')
            if box_elem is None:
                continue
            
            dim_x, dim_y, dim_z = [float(val) for val in box_elem.text.split()]

            # EURISTICA: Identifica i ripiani (Z è lo spessore, molto minore di X e Y)
            if dim_z >= dim_x or dim_z >= dim_y:
                continue

            pose_elem = coll.find('pose')
            if pose_elem is not None:
                elem_local_pos, elem_local_quat = parse_pose_string(pose_elem.text)
            else:
                elem_local_pos = [0.0, 0.0, 0.0]
                elem_local_quat = [0.0, 0.0, 0.0, 1.0]

            final_world_pos, final_world_quat = combine_transforms(
                link_world_pos, link_world_quat, elem_local_pos, elem_local_quat
            )
            
            # Griglia (sottraendo il raggio per non far sporgere la lattina)
            x_val = -dim_x / 2.0 + radius
            while x_val <= dim_x / 2.0 - radius:
                y_val = -dim_y / 2.0 + radius
                while y_val <= dim_y / 2.0 - radius:
                    # Il punto di appoggio è sulla faccia superiore del ripiano
                    local_pt = [x_val, y_val, dim_z / 2.0]
                    
                    rotated_pt = rotate_vector_by_quat(local_pt, final_world_quat)
                    world_x = final_world_pos[0] + rotated_pt[0]
                    world_y = final_world_pos[1] + rotated_pt[1]
                    world_z = final_world_pos[2] + rotated_pt[2]
                    
                    cells.append([cell_id, shelf_id, round(world_x, 4), round(world_y, 4), round(world_z, 4)])
                    cell_id += 1
                    y_val += can_diameter
                x_val += can_diameter
                
            shelf_id += 1
            print(f"[OK] Ripiano {shelf_id-1} processato: generate celle da {cell_id - len(cells)} a {cell_id - 1}.")

    if cells:
        try:
            with open(output_csv, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Cell_ID', 'Shelf_ID', 'X', 'Y', 'Z'])
                writer.writerows(cells)
            print(f"\n[SUCCESSO] Esportate {len(cells)} celle nel file: {output_csv}")
        except Exception as e:
            print(f"[ERRORE] Impossibile scrivere il file CSV: {e}")
    else:
        print("[AVVISO] Nessuna cella generata. Verifica la geometria nel file SDF.")

# ==========================================
# ENTRY POINT
# ==========================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generatore offline di celle raggiungibili per ripiani (da World e SDF).")
    parser.add_argument("--world",  default="/home/zinfollino/exchange/workspace/src/pal_gazebo_worlds/worlds/poliBaMaster.world",help="Percorso del file .world di Gazebo")
    parser.add_argument("--sdf",  default="/home/zinfollino/exchange/workspace/src/pal_gazebo_worlds/models/reemc_bookshelf/bookshelf.sdf",help="Percorso del file .sdf del modello della libreria")
    parser.add_argument("--model_name", default="s3_bookshelf", help="Nome del modello nel file .world (default: s3_bookshelf)")
    parser.add_argument("--can_diameter", type=float, default=0.04, help="Diametro della lattina in metri (default: 0.065)")
    parser.add_argument("--output", default="reachability_targets.csv", help="Nome del file CSV in output")
    
    args = parser.parse_args()

    print("=== Avvio Pre-calcolo Griglia Ripiani ===")
    
    # 1. Trova la posa della libreria nel mondo
    base_pos, base_quat = get_model_pose_from_world(args.world, args.model_name)
    
    # 2. Genera il CSV analizzando l'SDF
    generate_csv_from_sdf(args.sdf, base_pos, base_quat, args.can_diameter, args.output)