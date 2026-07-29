#!/usr/bin/env python3
import math
import os
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
import trimesh


class GazeboToMoveItSpawner(Node):

  def __init__(self):
    super().__init__('scene_spawner_node')

    self.get_logger().info(
        '=== [1/4] Inizializzazione nodo GazeboToMoveItSpawner ==='
    )

    self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
    self.frame_id = 'base_link'
    self.first_sync_done = False
    self.scene_cleaned = False

    # Client per interrogare MoveIt sui contenuti attuali della Planning Scene
    self.get_scene_client = self.create_client(
        GetPlanningScene, 'get_planning_scene'
    )

    # 1. Ricerca dinamica del file SDF della bookshelf (bookshelf.sdf)
    self.bookshelf_sdf_path = self.find_bookshelf_sdf()

    # 2. Timer per attendere il servizio e svuotare completamente la scena all'avvio
    self.get_logger().info(
        '=== [2/4] In attesa del servizio MoveIt per il reset totale della'
        ' scena... ==='
    )
    self.clean_timer = self.create_timer(1.0, self.query_and_clear_scene)

    self.gazebo_sub = self.create_subscription(
        ModelStates,
        '/gazebo/model_states',
        self.model_states_callback,
        10,
    )
    self.get_logger().info(
        '=== [3/4] In attesa dei dati da /gazebo/model_states... ==='
    )

  def query_and_clear_scene(self):
    """Interroga MoveIt via servizio per scoprire qualsiasi oggetto attivo e lo elimina."""
    if not self.get_scene_client.service_is_ready():
      self.get_logger().info(
          '[ATTESA] Servizio get_planning_scene non ancora pronto, riprovo...'
      )
      return

    self.clean_timer.cancel()

    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    future = self.get_scene_client.call_async(req)
    future.add_done_callback(self.clear_scene_callback)

  def clear_scene_callback(self, future):
    try:
      response = future.result()
      scene_msg = PlanningScene()
      scene_msg.is_diff = True

      ids_to_remove = []
      if response and response.scene and response.scene.world.collision_objects:
        for obj in response.scene.world.collision_objects:
          if 'ground' in obj.id.lower():
            continue
          ids_to_remove.append(obj.id)

      fallback_names = ['s3_table', 's3_cocacola', 's3_bookshelf']
      for name in fallback_names:
        if name not in ids_to_remove:
          ids_to_remove.append(name)

      if ids_to_remove:
        self.get_logger().info(
            f'[PULIZIA TOTALE] Rimozione forzata di {len(ids_to_remove)}'
            f' oggetti trovati in memoria: {ids_to_remove}'
        )
        for obj_id in ids_to_remove:
          rem_obj = CollisionObject()
          rem_obj.header.frame_id = self.frame_id
          rem_obj.header.stamp = self.get_clock().now().to_msg()
          rem_obj.id = obj_id
          rem_obj.operation = CollisionObject.REMOVE
          scene_msg.world.collision_objects.append(rem_obj)

        for _ in range(3):
          self.scene_pub.publish(scene_msg)
          time.sleep(0.05)
      else:
        self.get_logger().info(
            '[PULIZIA TOTALE] La Planning Scene di MoveIt è già completamente'
            ' vuota.'
        )

    except Exception as e:
      self.get_logger().error(
          f'Errore durante la pulizia della planning scene: {e}'
      )

    self.scene_cleaned = True
    self.get_logger().info(
        '=== [PULIZIA COMPLETATA] Scena azzerata. Pronti per la sincronizzazione'
        ' da Gazebo. ==='
    )

  def find_bookshelf_sdf(self) -> str:
    """Cerca il file bookshelf.sdf nei percorsi standard di ROS 2 / pal_gazebo_worlds."""
    try:
      pal_dir = get_package_share_directory('pal_gazebo_worlds')
      candidate = os.path.join(
          pal_dir, 'models', 'reemc_bookshelf', 'bookshelf.sdf'
      )
      if os.path.exists(candidate):
        self.get_logger().info(f'[SDF TROVATO] {candidate}')
        return candidate

      candidate_alt = os.path.join(
          pal_dir,
          'share',
          'pal_gazebo_worlds',
          'models',
          'reemc_bookshelf',
          'bookshelf.sdf',
      )
      if os.path.exists(candidate_alt):
        self.get_logger().info(f'[SDF TROVATO] {candidate_alt}')
        return candidate_alt
    except Exception as e:
      self.get_logger().warn(
          f"[SDF WARNING] Impossibile accedere a pal_gazebo_worlds: {e}"
      )

    self.get_logger().error(
        '[SDF NON TROVATO] Impossibile trovare bookshelf.sdf per'
        ' reemc_bookshelf! (Lo scaffale verrà saltato).'
    )
    return None

  def model_states_callback(self, msg: ModelStates):
    if not self.scene_cleaned:
      return

    robot_idx = None
    for i, name in enumerate(msg.name):
      if any(key in name.lower() for key in ['tiago', 'robot', 'pal']):
        robot_idx = i
        break

    if robot_idx is None:
      self.get_logger().warn_once(
          '[ATTENZIONE] Robot non trovato in /gazebo/model_states!'
      )
      return

    robot_pose = msg.pose[robot_idx]

    verbose = not self.first_sync_done
    if verbose:
      self.get_logger().info(
          '=== [4/4] Prima sincronizzazione scena in corso ==='
      )
      self.get_logger().info(
          f"[SYNC] Robot di riferimento individuato: '{msg.name[robot_idx]}'"
      )

    scene_msg = PlanningScene()
    scene_msg.is_diff = True
    new_active_ids = set()

    allowed_models = ['s3_table', 's3_cocacola', 's3_bookshelf']

    for idx, model_name in enumerate(msg.name):
      if model_name not in allowed_models:
        continue

      world_pose = msg.pose[idx]

      # 1. TAVOLO
      if model_name == 's3_table':
        if verbose:
          self.get_logger().info(f"[SYNC -> TAVOLO] Elaborazione '{model_name}'")
        obj = self.create_single_primitive_object(
            model_name,
            world_pose,
            robot_pose,
            'BOX',
            [1.0, 0.8, 0.75],
            z_offset=(0.75 / 2.0),
            
        )
        scene_msg.world.collision_objects.append(obj)
        new_active_ids.add(model_name)

      # 2. LATTINA
      elif model_name == 's3_cocacola':
        if verbose:
          self.get_logger().info(
              f"[SYNC -> LATTINA] Elaborazione '{model_name}'"
          )
        obj = self.create_single_primitive_object(
            model_name,
            world_pose,
            robot_pose,
            'CYLINDER',
            [0.15, 0.04],
            z_offset=0.00,
        )
        scene_msg.world.collision_objects.append(obj)
        new_active_ids.add(model_name)

      # 3. SCAFFALE
      elif model_name == 's3_bookshelf':
        if not self.bookshelf_sdf_path or not os.path.exists(
            self.bookshelf_sdf_path
        ):
          if verbose:
            self.get_logger().error(
                f"[SKIP BOOKSHELF] File SDF non trovato per '{model_name}' al"
                f" path : '{self.bookshelf_sdf_path}'"
            )
          continue

        if verbose:
          self.get_logger().info(
              f"[SYNC -> BOOKSHELF] Elaborazione '{model_name}' tramite"
              ' parser SDF...'
          )
        obj = self.create_multibox_from_sdf(
            model_name,
            world_pose,
            robot_pose,
            self.bookshelf_sdf_path,
            verbose=verbose,
        )

        if obj is None:
          if verbose:
            self.get_logger().error(
                f"[SKIP BOOKSHELF] Impossibile estrarre geometrie per '{model_name}'."
            )
          continue

        scene_msg.world.collision_objects.append(obj)
        new_active_ids.add(model_name)

    if scene_msg.world.collision_objects:
      self.scene_pub.publish(scene_msg)
      if verbose:
        self.get_logger().info(
            '[SUCCESSO] Scena pubblicata su /planning_scene con'
            f' {len(new_active_ids)} modelli attivi!'
        )
        self.first_sync_done = True

  def create_single_primitive_object(
      self,
      name: str,
      world_pose: Pose,
      robot_pose: Pose,
      prim_type: str,
      dims: list,
      z_offset=0.0,
  ) -> CollisionObject:
    obj = CollisionObject()
    obj.header.frame_id = self.frame_id
    obj.header.stamp = self.get_clock().now().to_msg()
    obj.id = name
    obj.operation = CollisionObject.ADD

    adjusted_world_pose = Pose()
    adjusted_world_pose.position.x = world_pose.position.x
    adjusted_world_pose.position.y = world_pose.position.y
    adjusted_world_pose.position.z = world_pose.position.z + z_offset
    adjusted_world_pose.orientation = world_pose.orientation

    rel_pose = self.get_relative_pose(adjusted_world_pose, robot_pose)

    primitive = SolidPrimitive()
    if prim_type == 'CYLINDER':
      primitive.type = SolidPrimitive.CYLINDER
    elif prim_type == 'BOX':
      primitive.type = SolidPrimitive.BOX

    primitive.dimensions = dims
    obj.primitives.append(primitive)
    obj.primitive_poses.append(rel_pose)
    return obj

  def create_multibox_from_sdf(
      self,
      name: str,
      base_world_pose: Pose,
      robot_pose: Pose,
      sdf_path: str,
      verbose=False,
  ) -> CollisionObject:
    try:
      tree = ET.parse(sdf_path)
      root = tree.getroot()
    except Exception as e:
      self.get_logger().error(
          f'[SDF ERROR] Errore di parsing per {sdf_path}: {e}'
      )
      return None

    obj = CollisionObject()
    obj.header.frame_id = self.frame_id
    obj.header.stamp = self.get_clock().now().to_msg()
    obj.id = name
    obj.operation = CollisionObject.ADD

    all_coll_boxes = root.findall('.//collision//geometry/box/size')
    all_vis_boxes = root.findall('.//visual//geometry/box/size')
    all_meshes = root.findall('.//geometry//mesh/uri')

    use_mode = 'collision_box'
    if len(all_coll_boxes) > 1:
      use_mode = 'collision_box'
    elif len(all_vis_boxes) > 1:
      use_mode = 'visual_box'
    elif len(all_meshes) > 0:
      use_mode = 'mesh'
    else:
      return None

    base_pos = [
        base_world_pose.position.x,
        base_world_pose.position.y,
        base_world_pose.position.z,
    ]
    base_quat = [
        base_world_pose.orientation.x,
        base_world_pose.orientation.y,
        base_world_pose.orientation.z,
        base_world_pose.orientation.w,
    ]

    links = root.findall('.//link')
    for link in links:
      link_pose_elem = link.find('pose')
      if link_pose_elem is not None:
        vals = [float(v) for v in link_pose_elem.text.split()]
        link_local_pos = vals[0:3]
        link_local_quat = self.quat_from_rpy(vals[3], vals[4], vals[5])
      else:
        link_local_pos = [0.0, 0.0, 0.0]
        link_local_quat = [0.0, 0.0, 0.0, 1.0]

      link_world_pos, link_world_quat = self.combine_transforms(
          base_pos, base_quat, link_local_pos, link_local_quat
      )

      if use_mode == 'collision_box':
        elements = link.findall('.//collision')
      elif use_mode == 'visual_box':
        elements = link.findall('.//visual')
      else:
        elements = [
            e
            for e in link.findall('.//visual') + link.findall('.//collision')
            if e.find('.//geometry/mesh/uri') is not None
        ]
        if len(elements) > 1:
          elements = [elements[0]]

      for elem in elements:
        pose_elem = elem.find('pose')
        if pose_elem is not None:
          vals = [float(v) for v in pose_elem.text.split()]
          elem_local_pos = vals[0:3]
          elem_local_quat = self.quat_from_rpy(vals[3], vals[4], vals[5])
        else:
          elem_local_pos = [0.0, 0.0, 0.0]
          elem_local_quat = [0.0, 0.0, 0.0, 1.0]

        final_world_pos, final_world_quat = self.combine_transforms(
            link_world_pos,
            link_world_quat,
            elem_local_pos,
            elem_local_quat,
        )

        box_world_pose = Pose()
        box_world_pose.position.x = final_world_pos[0]
        box_world_pose.position.y = final_world_pos[1]
        box_world_pose.position.z = final_world_pos[2]
        box_world_pose.orientation.x = final_world_quat[0]
        box_world_pose.orientation.y = final_world_quat[1]
        box_world_pose.orientation.z = final_world_quat[2]
        box_world_pose.orientation.w = final_world_quat[3]

        rel_pose = self.get_relative_pose(box_world_pose, robot_pose)

        if use_mode in ('collision_box', 'visual_box'):
          box_elem = elem.find('.//geometry/box/size')
          if box_elem is None:
            continue
          dims = [float(val) for val in box_elem.text.split()]
          primitive = SolidPrimitive()
          primitive.type = SolidPrimitive.BOX
          primitive.dimensions = dims
          obj.primitives.append(primitive)
          obj.primitive_poses.append(rel_pose)
        else:
          uri_elem = elem.find('.//geometry/mesh/uri')
          if uri_elem is None:
            continue
          mesh_path = self.resolve_mesh_uri(uri_elem.text, sdf_path)
          scale_elem = elem.find('.//geometry/mesh/scale')
          scale = (
              [float(v) for v in scale_elem.text.split()]
              if scale_elem is not None
              else (1.0, 1.0, 1.0)
          )

          mesh_msg = self.load_mesh_to_msg(mesh_path, scale=scale)
          if mesh_msg is not None:
            obj.meshes.append(mesh_msg)
            obj.mesh_poses.append(rel_pose)

    return (
        obj
        if (len(obj.primitives) > 0 or len(obj.meshes) > 0)
        else None
    )

  def resolve_mesh_uri(self, uri: str, sdf_path: str) -> str:
    sdf_dir = os.path.dirname(sdf_path)
    if '://' in uri:
      parts = uri.split('://')[1].split('/')
      return os.path.join(sdf_dir, *parts[1:])
    return os.path.join(sdf_dir, uri)

  def load_mesh_to_msg(self, filepath: str, scale=(1.0, 1.0, 1.0)) -> Mesh:
    if not os.path.exists(filepath):
      return None
    try:
      mesh = trimesh.load(filepath, force='mesh')
      mesh_msg = Mesh()
      for vert in mesh.vertices:
        p = Point()
        p.x = float(vert[0] * scale[0])
        p.y = float(vert[1] * scale[1])
        p.z = float(vert[2] * scale[2])
        mesh_msg.vertices.append(p)
      for face in mesh.faces:
        t = MeshTriangle()
        t.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
        mesh_msg.triangles.append(t)
      return mesh_msg
    except Exception:
      return None

  def combine_transforms(self, pos1, quat1, pos2, quat2):
    rotated_pos2 = self.rotate_vector_by_quat(pos2, quat1)
    new_pos = [
        pos1[0] + rotated_pos2[0],
        pos1[1] + rotated_pos2[1],
        pos1[2] + rotated_pos2[2],
    ]
    new_quat = self.quat_multiply(quat1, quat2)
    return new_pos, new_quat

  def get_relative_pose(self, target_pose: Pose, reference_pose: Pose) -> Pose:
    dx = target_pose.position.x - reference_pose.position.x
    dy = target_pose.position.y - reference_pose.position.y
    dz = target_pose.position.z - reference_pose.position.z

    ref_q_inv = [
        -reference_pose.orientation.x,
        -reference_pose.orientation.y,
        -reference_pose.orientation.z,
        reference_pose.orientation.w,
    ]

    rel_pos = self.rotate_vector_by_quat([dx, dy, dz], ref_q_inv)

    target_q = [
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ]
    rel_quat = self.quat_multiply(ref_q_inv, target_q)

    rel_pose = Pose()
    rel_pose.position.x = rel_pos[0]
    rel_pose.position.y = rel_pos[1]
    rel_pose.position.z = rel_pos[2]
    rel_pose.orientation.x = rel_quat[0]
    rel_pose.orientation.y = rel_quat[1]
    rel_pose.orientation.z = rel_quat[2]
    rel_pose.orientation.w = rel_quat[3]
    return rel_pose

  @staticmethod
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

  @staticmethod
  def quat_multiply(q1: list, q2: list) -> list:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]

  @staticmethod
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


def main(args=None):
  rclpy.init(args=args)
  node = GazeboToMoveItSpawner()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()