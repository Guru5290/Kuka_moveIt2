#!/usr/bin/env python3
"""
Enhanced WAAM Transpiler with 3D Visualization:
- Dynamic torch orientation based on path direction
- E-delta based welding detection
- Fixed distance calculations using transformed coordinates
- Speed ramping implementation
- Matplotlib 3D path visualization
- Torch orientation vector display
"""

import sys
import os
import re
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum

# Visualization imports (optional)
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False
    print("Warning: matplotlib not available. Visualization disabled.")

class MoveType(Enum):
    RAPID = "G0"
    LINEAR = "G1"

class PositionMode(Enum):
    ABSOLUTE = "G90"
    RELATIVE = "G91"

@dataclass
class Point:
    x: float  # Original G-code coordinates
    y: float
    z: float
    x_base: float  # Transformed BASE frame coordinates (with arc offset if welding)
    y_base: float
    z_base: float
    a: float = 129.7  # Orientation angles (calculated dynamically)
    b: float = -46.5
    c: float = 162.4
    move_type: MoveType = MoveType.LINEAR
    welding: bool = False
    e_delta: float = 0.0  # E change from previous move
    is_layer_change: bool = False
    layer_num: Optional[int] = None
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate Euclidean distance using transformed coordinates"""
        return math.sqrt(
            (self.x_base - other.x_base)**2 + 
            (self.y_base - other.y_base)**2 + 
            (self.z_base - other.z_base)**2
        )
    
    def get_orientation_vector(self, length: float = 20.0) -> Tuple[float, float, float]:
        """Calculate torch direction vector from orientation angles for visualization"""
        # Convert angles to radians
        a_rad = math.radians(self.a)
        b_rad = math.radians(self.b)
        c_rad = math.radians(self.c)
        
        # Simplified: torch points in direction based primarily on C (yaw) and B (pitch)
        # This is an approximation for visualization
        vx = length * math.cos(b_rad) * math.cos(c_rad)
        vy = length * math.cos(b_rad) * math.sin(c_rad)
        vz = length * math.sin(b_rad)
        
        return (vx, vy, vz)

# KUKA KR6 R900-2 Specifications
ROBOT_SPECS = {
    'reach': 900,  # mm
    'payload': 6,  # kg
    'max_tcp_speed': 0.3,  # m/s
    'default_travel_speed': 0.010,  # m/s (10 mm/s)
    'default_weld_speed': 0.005,  # m/s (5 mm/s)
}

# Coordinate transformation
PRUSASLICER_ORIGIN = {'x': 0, 'y': 0, 'z': 0}

# WAAM Parameters
WAAM_PARAMS = {
    'arc_length': 2.0,
    'pre_weld_delay': 0.2,
    'arc_strike_delay': 0.3,
    'arc_stabilization': 0.2,
    'arc_off_delay': 0.15,
    'post_weld_delay': 0.1,
    'inter_layer_delay': 1.0,
    'layer_height': 2.0,
    'start_speed_factor': 0.4,
    'end_speed_factor': 0.5,
    'ramp_distance': 5.0,
    'max_blend_distance': 2.0,
    'min_blend_distance': 0.1,
    'corner_angle_threshold': 45,
    'min_extrusion_threshold': 0.001,
    'torch_output': 1,
    'orientation_lookahead': 3,
}

# KRL Templates
HEADER_SRC = """DEF {program_name}()
;FOLD INI
  ;FOLD BASISTECH INI
    BAS (#INITMOV,0)
  ;ENDFOLD (BASISTECH INI)
  ;FOLD USER INI
    ;WAAM-optimized initialization
  ;ENDFOLD (USER INI)
;ENDFOLD (INI)

BASE_DATA[8] = {{FRAME: X 168.4, Y -507.55, Z 688.59, A 179.6, B -1.15, C 0.29}}
TOOL_DATA[4] = {{FRAME: X -17.74, Y -37.22, Z 294.62, A 0, B 0, C 0}}
$BASE = BASE_DATA[8]
$TOOL = TOOL_DATA[4]

BAS (#VEL_PTP,100)
BAS (#ACC_PTP,100)
BAS (#VEL_CP,{default_travel_speed:.4f})
BAS (#ACC_CP,0.05)

$VEL.CP = {default_travel_speed:.4f}
$ACC.CP = 0.05
$VEL.ORI1 = 90
$VEL.ORI2 = 90
$ACC.ORI1 = 500
$ACC.ORI2 = 500
$ORI_TYPE = #VAR
$ADVANCE = 1
$APO.CDIS = {default_blend:.2f}

PTP HOME
CONTINUE
$OUT[{torch_output}] = FALSE
WAIT SEC 0.5

;===========
; BEGIN WAAM PROCESS
; Speed: 5mm/s weld, 10mm/s travel
; Arc length: 2mm TCP offset
; Dynamic torch orientation
;===========

"""

HEADER_DAT = """DEFDAT {program_name} PUBLIC
DECL E6POS HOME={{X -183.3,Y -17.4,Z 38.3,A 129.7,B -46.5,C 162.4,S 18,T 34}}
ENDDAT
"""

FOOTER_SRC = """
;===========
; WAAM PROCESS COMPLETE
;===========

$VEL.CP = {default_travel_speed:.4f}
LIN_REL {{Z 20}}
WAIT SEC 0
PTP HOME

END
"""


class WAAMTranspiler:
    """Enhanced WAAM transpiler with visualization"""
    
    def __init__(self):
        self.current_pos = [0.0, 0.0, 0.0]
        self.welding_active = False
        self.position_mode = PositionMode.ABSOLUTE
        self.warnings = []
        self.current_e = 0.0
        self.previous_e = 0.0
        
    def transform_to_base(self, x: float, y: float, z: float, welding: bool = False) -> Tuple[float, float, float]:
        """Transform G-code coordinates to BASE frame with arc length offset"""
        x_base = PRUSASLICER_ORIGIN['x'] + x
        y_base = PRUSASLICER_ORIGIN['y'] + y
        z_offset = WAAM_PARAMS['arc_length'] if welding else 0.0
        z_base = PRUSASLICER_ORIGIN['z'] + z + z_offset
        return (x_base, y_base, z_base)
    
    def calculate_orientation(self, points: List[Point], index: int) -> Tuple[float, float, float]:
        """Calculate torch orientation based on path direction (look-ahead method)"""
        if not points[index].welding:
            return (129.7, -46.5, 162.4)
        
        lookahead = WAAM_PARAMS['orientation_lookahead']
        direction_point = None
        
        # Look ahead
        for i in range(index + 1, min(index + lookahead + 1, len(points))):
            if points[i].welding:
                dist = points[index].distance_to(points[i])
                if dist > 1.0:
                    direction_point = points[i]
                    break
        
        # Look behind if no ahead
        if direction_point is None and index > 0:
            for i in range(index - 1, max(index - lookahead - 1, -1), -1):
                if points[i].welding:
                    dist = points[index].distance_to(points[i])
                    if dist > 1.0:
                        direction_point = points[i]
                        break
        
        if direction_point is None:
            return (129.7, -46.5, 162.4)
        
        # Direction vector
        dx = direction_point.x_base - points[index].x_base
        dy = direction_point.y_base - points[index].y_base
        dz = direction_point.z_base - points[index].z_base
        
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 0.001:
            return (129.7, -46.5, 162.4)
        
        dx /= mag
        dy /= mag
        dz /= mag
        
        # Calculate orientation
        c = math.degrees(math.atan2(dy, dx))
        b = -45.0  # Standard push angle
        a = 130.0
        
        return (a, b, c)
    
    def parse_line(self, line: str) -> Optional[Point]:
        """Parse G-code line with E-delta based welding detection"""
        line = line.strip()
        original_line = line
        line = line.upper()
        
        is_layer_change = ';LAYER_CHANGE' in original_line
        layer_num = None
        if is_layer_change:
            layer_match = re.search(r';LAYER:(\d+)', original_line)
            if layer_match:
                layer_num = int(layer_match.group(1))
        
        if not line or line.startswith(';') or line.startswith('('):
            return None
        
        if 'G90' in line:
            self.position_mode = PositionMode.ABSOLUTE
            return None
        elif 'G91' in line:
            self.position_mode = PositionMode.RELATIVE
            return None
        
        if 'G92' in line:
            e_match = re.search(r'E([-+]?[0-9]*\.?[0-9]+)', line)
            if e_match:
                self.current_e = float(e_match.group(1))
                self.previous_e = self.current_e
            return None
        
        move_type = None
        if 'G0' in line or 'G00' in line:
            move_type = MoveType.RAPID
        elif 'G1' in line or 'G01' in line:
            move_type = MoveType.LINEAR
        else:
            return None
        
        x_match = re.search(r'X([-+]?[0-9]*\.?[0-9]+)', line)
        y_match = re.search(r'Y([-+]?[0-9]*\.?[0-9]+)', line)
        z_match = re.search(r'Z([-+]?[0-9]*\.?[0-9]+)', line)
        e_match = re.search(r'E([-+]?[0-9]*\.?[0-9]+)', line)
        
        if self.position_mode == PositionMode.ABSOLUTE:
            x = float(x_match.group(1)) if x_match else self.current_pos[0]
            y = float(y_match.group(1)) if y_match else self.current_pos[1]
            z = float(z_match.group(1)) if z_match else self.current_pos[2]
        else:
            x = self.current_pos[0] + (float(x_match.group(1)) if x_match else 0.0)
            y = self.current_pos[1] + (float(y_match.group(1)) if y_match else 0.0)
            z = self.current_pos[2] + (float(z_match.group(1)) if z_match else 0.0)
        
        e_delta = 0.0
        if e_match:
            self.current_e = float(e_match.group(1))
            e_delta = self.current_e - self.previous_e
            self.previous_e = self.current_e
        
        weld_from_e = e_delta > WAAM_PARAMS['min_extrusion_threshold']
        weld_on_cmd = 'M3' in line or 'M03' in line
        weld_off_cmd = 'M5' in line or 'M05' in line or 'M107' in line
        
        if weld_on_cmd or weld_from_e:
            self.welding_active = True
        elif weld_off_cmd or e_delta < -0.01:
            self.welding_active = False
        
        x_base, y_base, z_base = self.transform_to_base(x, y, z, self.welding_active)
        self.current_pos = [x, y, z]
        
        return Point(
            x=x, y=y, z=z,
            x_base=x_base, y_base=y_base, z_base=z_base,
            move_type=move_type,
            welding=self.welding_active,
            e_delta=e_delta,
            is_layer_change=is_layer_change,
            layer_num=layer_num
        )
    
    def detect_layers(self, points: List[Point]) -> None:
        """Detect and mark layer boundaries"""
        if not points:
            return
        
        current_layer = 0
        last_weld_z = None
        
        for point in points:
            if not point.welding:
                continue
            
            current_z = point.z
            
            if last_weld_z is not None:
                z_diff = current_z - last_weld_z
                if z_diff >= WAAM_PARAMS['layer_height'] * 0.9:
                    current_layer += 1
                    point.is_layer_change = True
            
            point.layer_num = current_layer
            last_weld_z = current_z
    
    def calculate_blend_distance(self, points: List[Point], index: int) -> float:
        """Calculate blend distance using transformed coordinates"""
        if index == 0 or index >= len(points) - 1:
            return 0.0
        
        point = points[index]
        if not point.welding:
            return 0.0
        
        prev_point = points[index - 1]
        next_point = points[index + 1]
        
        dist_to_prev = point.distance_to(prev_point)
        dist_to_next = point.distance_to(next_point)
        
        if dist_to_prev < 0.1 or dist_to_next < 0.1:
            return 0.0
        
        v1 = [point.x_base - prev_point.x_base, 
              point.y_base - prev_point.y_base, 
              point.z_base - prev_point.z_base]
        v2 = [next_point.x_base - point.x_base, 
              next_point.y_base - point.y_base, 
              next_point.z_base - point.z_base]
        
        dot = sum(a*b for a, b in zip(v1, v2))
        mag1 = math.sqrt(sum(a*a for a in v1))
        mag2 = math.sqrt(sum(a*a for a in v2))
        
        if mag1 < 0.001 or mag2 < 0.001:
            return 0.0
        
        cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
        angle = math.degrees(math.acos(cos_angle))
        
        if angle < WAAM_PARAMS['corner_angle_threshold']:
            return 0.0
        
        max_blend = min(
            dist_to_prev * 0.15,
            dist_to_next * 0.15,
            WAAM_PARAMS['max_blend_distance']
        )
        
        angle_factor = (angle - WAAM_PARAMS['corner_angle_threshold']) / \
                      (180 - WAAM_PARAMS['corner_angle_threshold'])
        blend = max_blend * angle_factor
        
        return blend if blend > WAAM_PARAMS['min_blend_distance'] else 0.0
    
    def visualize_path(self, points: List[Point], output_path: str = None):
        """Create 3D visualization of the tool path with orientation vectors"""
        if not VISUALIZATION_AVAILABLE:
            print("\nVisualization skipped (matplotlib not available)")
            return
        
        # Separate weld and travel points
        weld_points = [p for p in points if p.welding]
        travel_points = [p for p in points if not p.welding]
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        
        # Main 3D view
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        
        # Plot travel moves (blue, thin)
        if travel_points:
            travel_x = [p.x_base for p in travel_points]
            travel_y = [p.y_base for p in travel_points]
            travel_z = [p.z_base for p in travel_points]
            ax1.plot(travel_x, travel_y, travel_z, 'b-', linewidth=0.5, alpha=0.3, label='Travel')
        
        # Plot weld moves by layer (different colors)
        layers = set(p.layer_num for p in weld_points if p.layer_num is not None)
        colors = plt.cm.viridis(np.linspace(0, 1, max(len(layers), 1)))
        
        for i, layer in enumerate(sorted(layers)):
            layer_points = [p for p in weld_points if p.layer_num == layer]
            if layer_points:
                lx = [p.x_base for p in layer_points]
                ly = [p.y_base for p in layer_points]
                lz = [p.z_base for p in layer_points]
                ax1.plot(lx, ly, lz, color=colors[i], linewidth=2, label=f'Layer {layer}')
        
        # Plot orientation vectors (every Nth weld point)
        sample_rate = max(1, len(weld_points) // 20)  # Show ~20 vectors
        for i in range(0, len(weld_points), sample_rate):
            p = weld_points[i]
            vx, vy, vz = p.get_orientation_vector(length=15.0)
            ax1.quiver(p.x_base, p.y_base, p.z_base, vx, vy, vz,
                      color='red', alpha=0.6, arrow_length_ratio=0.3, linewidth=1.5)
        
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.set_title('3D Tool Path with Torch Orientation')
        ax1.legend(loc='upper left', fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        # Top view (XY plane)
        ax2 = fig.add_subplot(2, 2, 2)
        if travel_points:
            ax2.plot([p.x_base for p in travel_points], 
                    [p.y_base for p in travel_points], 
                    'b-', linewidth=0.5, alpha=0.3)
        for i, layer in enumerate(sorted(layers)):
            layer_points = [p for p in weld_points if p.layer_num == layer]
            if layer_points:
                ax2.plot([p.x_base for p in layer_points],
                        [p.y_base for p in layer_points],
                        color=colors[i], linewidth=2, label=f'Layer {layer}')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_title('Top View (XY)')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # Side view (XZ plane)
        ax3 = fig.add_subplot(2, 2, 3)
        if travel_points:
            ax3.plot([p.x_base for p in travel_points],
                    [p.z_base for p in travel_points],
                    'b-', linewidth=0.5, alpha=0.3)
        for i, layer in enumerate(sorted(layers)):
            layer_points = [p for p in weld_points if p.layer_num == layer]
            if layer_points:
                ax3.plot([p.x_base for p in layer_points],
                        [p.z_base for p in layer_points],
                        color=colors[i], linewidth=2)
        ax3.set_xlabel('X (mm)')
        ax3.set_ylabel('Z (mm)')
        ax3.set_title('Side View (XZ)')
        ax3.grid(True, alpha=0.3)
        
        # Front view (YZ plane)
        ax4 = fig.add_subplot(2, 2, 4)
        if travel_points:
            ax4.plot([p.y_base for p in travel_points],
                    [p.z_base for p in travel_points],
                    'b-', linewidth=0.5, alpha=0.3)
        for i, layer in enumerate(sorted(layers)):
            layer_points = [p for p in weld_points if p.layer_num == layer]
            if layer_points:
                ax4.plot([p.y_base for p in layer_points],
                        [p.z_base for p in layer_points],
                        color=colors[i], linewidth=2)
        ax4.set_xlabel('Y (mm)')
        ax4.set_ylabel('Z (mm)')
        ax4.set_title('Front View (YZ)')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save if output path provided
        if output_path:
            viz_path = output_path.replace('.src', '_visualization.png')
            plt.savefig(viz_path, dpi=150, bbox_inches='tight')
            print(f"\n Visualization saved: {viz_path}")
        
        plt.show()
    
    def generate_krl(self, points: List[Point], program_name: str = "WAAM_PART") -> str:
        """Generate optimized KRL with dynamic orientation and speed ramping"""
        if not points:
            raise ValueError("No valid points")
        
        # Filter duplicates
        filtered = [points[0]]
        for i in range(1, len(points)):
            dist = points[i].distance_to(filtered[-1])
            if dist > 0.01 or points[i].welding != filtered[-1].welding:
                filtered.append(points[i])
        points = filtered
        
        self.detect_layers(points)
        
        # Calculate orientations
        for i in range(len(points)):
            if points[i].welding:
                a, b, c = self.calculate_orientation(points, i)
                points[i].a, points[i].b, points[i].c = a, b, c
        
        krl_lines = []
        krl_lines.append(HEADER_SRC.format(
            program_name=program_name,
            default_travel_speed=ROBOT_SPECS['default_travel_speed'],
            default_blend=WAAM_PARAMS['max_blend_distance'],
            torch_output=WAAM_PARAMS['torch_output']
        ))
        
        torch_on = False
        prev_welding = False
        prev_layer = None
        weld_segment_start = None
        cumulative_weld_dist = 0.0
        
        WELD_SPEED = ROBOT_SPECS['default_weld_speed']
        TRAVEL_SPEED = ROBOT_SPECS['default_travel_speed']
        
        for i, point in enumerate(points):
            if point.is_layer_change and point.layer_num != prev_layer and prev_layer is not None:
                if torch_on:
                    krl_lines.append("\n; Turn off torch for layer transition\n")
                    krl_lines.append(f"CONTINUE\n$OUT[{WAAM_PARAMS['torch_output']}] = FALSE\n")
                    krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_off_delay']:.2f}\n")
                    torch_on = False
                
                krl_lines.append(f"\n;========== LAYER {point.layer_num} ==========\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['inter_layer_delay']:.2f}\n\n")
            
            if point.welding and not torch_on:
                krl_lines.append("\n;--- TORCH ON ---\n")
                krl_lines.append(f"$VEL.CP = 0.001\nWAIT SEC 0\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['pre_weld_delay']:.2f}\n")
                krl_lines.append(f"CONTINUE\n$OUT[{WAAM_PARAMS['torch_output']}] = TRUE\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_strike_delay']:.2f}\n\n")
                torch_on = True
                weld_segment_start = i
                cumulative_weld_dist = 0.0
                
                start_speed = WELD_SPEED * WAAM_PARAMS['start_speed_factor']
                krl_lines.append(f"; Arc start at {start_speed*1000:.1f} mm/s\n")
                krl_lines.append(f"$VEL.CP = {start_speed:.4f}\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_stabilization']:.2f}\n\n")
                
            elif not point.welding and torch_on:
                crater_speed = WELD_SPEED * WAAM_PARAMS['end_speed_factor']
                krl_lines.append(f"\n; Crater fill at {crater_speed*1000:.1f} mm/s\n")
                krl_lines.append(f"$VEL.CP = {crater_speed:.4f}\nWAIT SEC 0.1\n\n")
                
                krl_lines.append(";--- TORCH OFF ---\n")
                krl_lines.append(f"CONTINUE\n$OUT[{WAAM_PARAMS['torch_output']}] = FALSE\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_off_delay']:.2f}\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['post_weld_delay']:.2f}\n\n")
                torch_on = False
                weld_segment_start = None
            
            if point.welding and weld_segment_start is not None and i > 0:
                cumulative_weld_dist += points[i-1].distance_to(point)
                
                if cumulative_weld_dist >= WAAM_PARAMS['ramp_distance']:
                    prev_dist = cumulative_weld_dist - points[i-1].distance_to(point)
                    if prev_dist < WAAM_PARAMS['ramp_distance']:
                        krl_lines.append(f"; Ramp to full weld speed {WELD_SPEED*1000:.1f} mm/s\n")
                        krl_lines.append(f"$VEL.CP = {WELD_SPEED:.4f}\n\n")
            
            if not point.welding and prev_welding:
                krl_lines.append(f"; Travel speed {TRAVEL_SPEED*1000:.1f} mm/s\n")
                krl_lines.append(f"$VEL.CP = {TRAVEL_SPEED:.4f}\n\n")
            
            pos_str = f"{{X {point.x_base:.3f}, Y {point.y_base:.3f}, Z {point.z_base:.3f}, " \
                     f"A {point.a:.1f}, B {point.b:.1f}, C {point.c:.1f}}}"
            
            if point.move_type == MoveType.RAPID:
                krl_lines.append(f"PTP {pos_str}\n")
            else:
                blend = 0.0
                if point.welding and prev_welding and not point.is_layer_change:
                    blend = self.calculate_blend_distance(points, i)
                
                if point.is_layer_change or blend < WAAM_PARAMS['min_blend_distance']:
                    krl_lines.append(f"LIN {pos_str}\n")
                    if point.is_layer_change:
                        krl_lines.append("WAIT SEC 0\n")
                else:
                    krl_lines.append(f"$APO.CDIS = {blend:.2f}\n")
                    krl_lines.append(f"LIN {pos_str} C_DIS\n")
            
            prev_welding = point.welding
            prev_layer = point.layer_num
        
        if torch_on:
            krl_lines.append("\n; Final torch off\n")
            krl_lines.append(f"CONTINUE\n$OUT[{WAAM_PARAMS['torch_output']}] = FALSE\n")
            krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_off_delay']:.2f}\n")
        
        krl_lines.append(FOOTER_SRC.format(
            default_travel_speed=ROBOT_SPECS['default_travel_speed']
        ))
        
        return "".join(krl_lines)
    
    def generate_statistics(self, points: List[Point]) -> dict:
        """Calculate statistics"""
        weld_points = [p for p in points if p.welding]
        travel_points = [p for p in points if not p.welding]
        
        total_length = sum(points[i].distance_to(points[i+1]) 
                          for i in range(len(points)-1))
        weld_length = sum(weld_points[i].distance_to(weld_points[i+1]) 
                         for i in range(len(weld_points)-1) if i < len(weld_points)-1)
        
        layers = set(p.layer_num for p in points if p.layer_num is not None)
        
        return {
            'total_points': len(points),
            'weld_points': len(weld_points),
            'travel_points': len(travel_points),
            'total_length_mm': total_length,
            'weld_length_mm': weld_length,
            'layer_count': len(layers),
            'warnings': len(self.warnings)
        }
    
    def process_file(self, input_path: str, output_path: str, visualize: bool = True):
        """Process G-code and generate KRL with optional visualization"""
        points = []
        
        print(f"\n{'='*70}")
        print(f"  WAAM Transpiler - Fixed & Optimized with Visualization")
        print(f"{'='*70}")
        print(f"\nInput:  {input_path}")
        
        program_name = os.path.splitext(os.path.basename(input_path))[0].upper()
        
        print("\nParsing G-code...")
        with open(input_path, 'r') as f:
            for line in f:
                point = self.parse_line(line)
                if point:
                    points.append(point)
        
        if not points:
            print("\n ERROR: No valid G-code found!")
            return
        
        print(f" Parsed {len(points)} points")
        
        # Check first layer Z
        first_weld = next((p for p in points if p.welding), None)
        if first_weld:
            print(f"\n First weld point:")
            print(f"   G-code Z:     {first_weld.z:.2f} mm")
            print(f"   TCP Z (BASE): {first_weld.z_base:.2f} mm")
            print(f"   (Arc offset: {WAAM_PARAMS['arc_length']:.1f} mm applied)")
        
        print("\nGenerating KRL...")
        try:
            krl_content = self.generate_krl(points, program_name)
        except Exception as e:
            print(f"\n ERROR: {e}")
            import traceback
            traceback.print_exc()
            return
        
        print("\nWriting files...")
        with open(output_path, 'w') as f:
            f.write(krl_content)
        
        dat_path = output_path.replace('.src', '.dat')
        with open(dat_path, 'w') as f:
            f.write(HEADER_DAT.format(program_name=program_name))
        
        stats = self.generate_statistics(points)
        
        print("\n TRANSPILATION COMPLETE")
        print(f"\n Output Files:")
        print(f"   • {output_path}")
        print(f"   • {dat_path}")
        print(f"\n Statistics:")
        print(f"   • Total points:  {stats['total_points']}")
        print(f"   • Weld points:   {stats['weld_points']}")
        print(f"   • Travel points: {stats['travel_points']}")
        print(f"   • Total path:    {stats['total_length_mm']:.1f} mm")
        print(f"   • Weld path:     {stats['weld_length_mm']:.1f} mm")
        print(f"   • Layers:        {stats['layer_count']}")
        
        if self.warnings:
            print(f"\n Warnings ({len(self.warnings)}):")
            for w in list(set(self.warnings))[:5]:
                print(f"   • {w}")
        
        # Generate visualization
        if visualize and VISUALIZATION_AVAILABLE:
            print("\nGenerating 3D visualization...")
            self.visualize_path(points, output_path)
        elif visualize and not VISUALIZATION_AVAILABLE:
            print("\n Note: Install matplotlib for visualization:")
            print("   pip install matplotlib numpy")


def main():
    if len(sys.argv) < 2:
        print("\n" + "="*70)
        print("  WAAM Transpiler - Fixed & Optimized with Visualization")
        print("\nUsage:")
        print("  python transpiler.py <gcode_file> [--no-viz]")
        print("\nOptions:")
        print("  --no-viz    Skip 3D visualization")
        print("\nOutput:")
        print("  • <name>.src              (KRL source)")
        print("  • <name>.dat              (KRL data)")
        print("  • <name>_visualization.png (3D path plot)")
        print("\nVisualization shows:")
        print("  • 3D tool path with layers in different colors")
        print("  • Red arrows showing torch orientation")
        print("  • Top, side, and front view projections")
        print("  • Travel moves (blue) vs weld moves (colored by layer)")
        print("\n" + "="*70 + "\n")
        sys.exit(1)
    
    gcode_file = sys.argv[1]
    if not os.path.isfile(gcode_file):
        print(f"\n ERROR: File not found: {gcode_file}\n")
        sys.exit(1)
    
    # Check for --no-viz flag
    visualize = '--no-viz' not in sys.argv
    
    base_name = os.path.splitext(os.path.basename(gcode_file))[0]
    src_file = base_name + ".src"
    
    transpiler = WAAMTranspiler()
    transpiler.process_file(gcode_file, src_file, visualize=visualize)


if __name__ == "__main__":
    main()