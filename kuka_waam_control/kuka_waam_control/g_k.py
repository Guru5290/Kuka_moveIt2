#!/usr/bin/env python3
"""
Key Features:
- PrusaSlicer G-code parsing
- E-value based welding detection (E>0 = weld, E≤0 = travel)
- Explicit torch ON/OFF control throughout program
- Layer boundary detection via Z height changes
- Speed ramping for arc stability
- Proper approximate positioning with calculated blend distances
- Complete velocity and orientation control
- Arc timing sequences for MIG welding
"""

import sys
import os
import re
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum

class MoveType(Enum):
    RAPID = "G0"
    LINEAR = "G1"

class PositionMode(Enum):
    ABSOLUTE = "G90"
    RELATIVE = "G91"

@dataclass
class Point:
    x: float
    y: float
    z: float
    move_type: MoveType = MoveType.LINEAR
    welding: bool = False
    feed_rate: Optional[float] = None  # in m/s
    e_value: float = 0.0  # Extrusion value
    is_layer_change: bool = False  # Layer boundary marker
    layer_z: Optional[float] = None  # Layer Z height from comment
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate Euclidean distance to another point"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )

# KUKA KR6 R900-2 Specifications
ROBOT_SPECS = {
    'reach': 900,  # mm
    'payload': 6,  # kg
    'max_tcp_speed': 0.3,  # m/s (safety limit)
    'default_travel_speed': 0.04,  # m/s (40 mm/s for travel)
    'default_weld_speed': 0.006,  # m/s (6 mm/s typical for WAAM)
    'min_weld_speed': 0.003,  # m/s (3 mm/s minimum)
    'max_weld_speed': 0.015,  # m/s (15 mm/s maximum for quality)
}

# PrusaSlicer origin in robot BASE frame
PRUSASLICER_ORIGIN = {
    'x': 0,
    'y': 0,
    'z': 0
}

# WAAM-specific parameters for MIG welding
WAAM_PARAMS = {
    # Arc timing (all in seconds)
    'pre_weld_delay': 0.2,        # Stabilization before arc strike
    'arc_strike_delay': 0.3,      # Wait for arc establishment
    'arc_stabilization': 0.2,     # Post-strike stabilization
    'arc_off_delay': 0.15,        # Arc termination time
    'post_weld_delay': 0.1,       # Post-weld stabilization
    
    # Layer management
    'inter_layer_delay': 1.0,     # Thermal management between layers
    'layer_height': 2.0,          # mm - Expected layer height
    
    # Speed ramping factors
    'start_speed_factor': 0.4,    # 40% speed for arc start
    'end_speed_factor': 0.5,      # 50% speed for crater fill
    'ramp_distance': 5.0,         # mm - distance for speed transitions
    
    # Blending
    'max_blend_distance': 2.0,    # mm - maximum approximate positioning
    'min_blend_distance': 0.1,    # mm - minimum to trigger blending
    'corner_angle_threshold': 45,  # degrees - sharp corner detection
    
    # Extrusion detection
    'min_extrusion_threshold': 0.01,  # Minimum E value to consider welding
    
    # I/O
    'torch_output': 1,            # Digital output for torch control
}

# KRL Program Templates
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


; Basic PTP parameters
BAS (#VEL_PTP,100)
BAS (#ACC_PTP,100)

; CP motion parameters (critical for WAAM quality)
BAS (#VEL_CP,{default_travel_speed:.4f})
BAS (#ACC_CP,0.05)

; Complete velocity and acceleration initialization
$VEL.CP = {default_travel_speed:.4f}        ; Cartesian path velocity (m/s)
$ACC.CP = 0.05                              ; Cartesian path acceleration (m/s²)
$VEL.ORI1 = 90                              ; Swivel velocity (°/s)
$VEL.ORI2 = 90                              ; Rotational velocity (°/s)
$ACC.ORI1 = 500                             ; Swivel acceleration (°/s²)
$ACC.ORI2 = 500                             ; Rotational acceleration (°/s²)

; Orientation control for consistent torch angle
$ORI_TYPE = #VAR                            ; Smooth orientation interpolation

; Advance run control - CRITICAL for welding
$ADVANCE = 1                                ; Reduced advance for precise I/O timing

; Default approximation distance
$APO.CDIS = {default_blend:.2f}             ; Default blending distance (mm)

; Move to safe start position
PTP HOME

; Initialize welding system
CONTINUE                                    ; Prevent advance run stop
$OUT[{torch_output}] = FALSE                ; Ensure torch is OFF
WAIT SEC 0.5                                ; System stabilization

;===========
; BEGIN WAAM PROCESS
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

; Return to home position
$VEL.CP = {default_travel_speed:.4f}
LIN_REL {{Z 20}}                            ; Retract 20mm first
WAIT SEC 0
PTP HOME                                    ; Return home

END
"""


class WAAMTranspiler:
    """Enhanced transpiler for WAAM with PrusaSlicer support"""
    
    def __init__(self):
        self.current_pos = [0.0, 0.0, 0.0]
        self.welding_active = False
        self.position_mode = PositionMode.ABSOLUTE
        self.feed_rate = ROBOT_SPECS['default_weld_speed']
        self.warnings = []
        self.layer_count = 0
        self.current_e = 0.0  # Track cumulative E value
        self.last_layer_z = None
        self.torch_state = False  # Track actual torch ON/OFF state
        
    def transform_gcode_to_base(self, x: float, y: float, z: float) -> Optional[tuple]:
        """Transform PrusaSlicer coordinates to robot BASE frame"""
        x_base = PRUSASLICER_ORIGIN['x'] + x
        y_base = PRUSASLICER_ORIGIN['y'] + y
        z_base = PRUSASLICER_ORIGIN['z'] + z
        
        if not self.validate_position(x_base, y_base, z_base):
            self.warnings.append(
                f"Invalid transformed position: X{x_base:.1f} Y{y_base:.1f} Z{z_base:.1f}"
            )
            return None
            
        return (x_base, y_base, z_base)
    
    def validate_position(self, x: float, y: float, z: float) -> bool:
        """Validate position values"""
        if not all(isinstance(v, (int, float)) for v in [x, y, z]):
            return False
        # Basic workspace check (adjust based on your setup)
        if abs(x) > ROBOT_SPECS['reach'] or abs(y) > ROBOT_SPECS['reach']:
            return False
        return True

    def format_pos(self, x: float, y: float, z: float) -> Optional[str]:
        """Format position for KRL with HOME orientation values"""
        transformed = self.transform_gcode_to_base(x, y, z)
        if transformed is None:
            return None
            
        x_base, y_base, z_base = transformed
        return f"{{X {x_base:.3f}, Y {y_base:.3f}, Z {z_base:.3f}, A 129.7, B -46.5, C 162.4}}"

    def parse_line(self, line: str) -> Optional[Point]:
        """Parse G-code line into Point object with E-value based welding detection"""
        line = line.strip()
        original_line = line
        line = line.upper()
        
        # Check for layer change comments
        is_layer_change = False
        layer_z = None
        if ';LAYER_CHANGE' in original_line:
            is_layer_change = True
        # Extract Z height from comment
        z_comment_match = re.search(r';Z:([\d.]+)', original_line)
        if z_comment_match:
            layer_z = float(z_comment_match.group(1))
        
        if not line or line.startswith(';') or line.startswith('('):
            return None
        
        # Handle positioning mode
        if 'G90' in line:
            self.position_mode = PositionMode.ABSOLUTE
            return None
        elif 'G91' in line:
            self.position_mode = PositionMode.RELATIVE
            return None
            
        # Determine move type
        move_type = None
        if 'G0' in line or 'G00' in line:
            move_type = MoveType.RAPID
        elif 'G1' in line or 'G01' in line:
            move_type = MoveType.LINEAR
        else:
            return None
            
        # Extract coordinates
        x_match = re.search(r'X([-+]?[0-9]*\.?[0-9]+)', line)
        y_match = re.search(r'Y([-+]?[0-9]*\.?[0-9]+)', line)
        z_match = re.search(r'Z([-+]?[0-9]*\.?[0-9]+)', line)
        f_match = re.search(r'F([-+]?[0-9]*\.?[0-9]+)', line)
        e_match = re.search(r'E([-+]?[0-9]*\.?[0-9]+)', line)
        
        # Parse coordinates
        if self.position_mode == PositionMode.ABSOLUTE:
            x = float(x_match.group(1)) if x_match else self.current_pos[0]
            y = float(y_match.group(1)) if y_match else self.current_pos[1]
            z = float(z_match.group(1)) if z_match else self.current_pos[2]
        else:
            x = self.current_pos[0] + (float(x_match.group(1)) if x_match else 0.0)
            y = self.current_pos[1] + (float(y_match.group(1)) if y_match else 0.0)
            z = self.current_pos[2] + (float(z_match.group(1)) if z_match else 0.0)
        
        # Convert feed rate: mm/min to m/s
        if f_match:
            feed_mm_min = float(f_match.group(1))
            self.feed_rate = feed_mm_min / 60000.0
            # Clamp to safe WAAM range
            self.feed_rate = max(
                ROBOT_SPECS['min_weld_speed'],
                min(self.feed_rate, ROBOT_SPECS['max_weld_speed'])
            )
        
        # Parse E value and detect welding state
        e_value = 0.0
        if e_match:
            e_value = float(e_match.group(1))
        
        # Handle G92 E0 (reset extruder position)
        if 'G92' in line and e_match:
            self.current_e = float(e_match.group(1))
            return None  # G92 doesn't produce a motion point
        
        # Detect welding state from E values AND M commands
        # Welding is active if:
        # 1. Positive E value (actual deposition) - most reliable indicator
        # 2. M3/M03 command (torch on)
        weld_on = (
            (e_value > WAAM_PARAMS['min_extrusion_threshold']) or
            'M3' in line or 'M03' in line
        )
        
        # Welding stops if:
        # 1. Negative E value (retraction)
        # 2. Zero or very small E value on a move
        # 3. M5/M05 command (torch off)
        # 4. M107 (fan/torch off)
        weld_off = (
            (e_value < -0.01) or  # Retraction
            (e_match and abs(e_value) < WAAM_PARAMS['min_extrusion_threshold']) or  # No extrusion
            'M5' in line or 'M05' in line or
            'M107' in line
        )
        
        if weld_on:
            self.welding_active = True
        elif weld_off:
            self.welding_active = False
            
        self.current_pos = [x, y, z]
        
        return Point(
            x=x, y=y, z=z,
            move_type=move_type,
            welding=self.welding_active,
            feed_rate=self.feed_rate,
            e_value=e_value,
            is_layer_change=is_layer_change,
            layer_z=layer_z
        )
    
    def detect_layer_boundaries(self, points: List[Point]) -> List[bool]:
        """Detect layer boundaries using Z height changes (2mm layers)"""
        boundaries = [False] * len(points)
        
        # Track last welding Z position to detect layer changes
        last_weld_z = None
        
        for i in range(len(points)):
            if points[i].welding:
                current_z = points[i].z
                
                # If we have a previous weld Z and it's different by layer height
                if last_weld_z is not None:
                    z_diff = abs(current_z - last_weld_z)
                    if z_diff >= WAAM_PARAMS['layer_height'] * 0.9:  # 90% threshold
                        # Mark this as start of new layer
                        boundaries[i] = True
                        # Mark previous weld end as boundary
                        for j in range(i-1, -1, -1):
                            if points[j].welding:
                                boundaries[j] = True
                                break
                
                last_weld_z = current_z
                    
        return boundaries
    
    def identify_weld_segments(self, points: List[Point]) -> List[str]:
        """Identify weld start, middle, and end points"""
        segments = ['middle'] * len(points)
        
        for i in range(len(points)):
            if i == 0:
                if points[i].welding:
                    segments[i] = 'start'
            else:
                if points[i].welding and not points[i-1].welding:
                    segments[i] = 'start'
                elif not points[i].welding and points[i-1].welding:
                    segments[i-1] = 'end'
        
        # Mark the last welding point as end if still welding
        if len(points) > 0 and points[-1].welding:
            segments[-1] = 'end'
                    
        return segments
    
    def calculate_blend_distance(self, points: List[Point], index: int) -> float:
        """Calculate optimal blend distance for point"""
        if index == 0 or index >= len(points) - 1:
            return 0.0  # No blending at endpoints
            
        point = points[index]
        if not point.welding:
            return 0.0  # No blending for travel moves
            
        prev_point = points[index - 1]
        next_point = points[index + 1]
        
        # Calculate segment lengths
        dist_to_prev = point.distance_to(prev_point)
        dist_to_next = point.distance_to(next_point)
        
        if dist_to_prev < 0.1 or dist_to_next < 0.1:
            return 0.0  # Too close for blending
        
        # Calculate angle at this point
        v1 = [point.x - prev_point.x, point.y - prev_point.y, point.z - prev_point.z]
        v2 = [next_point.x - point.x, next_point.y - point.y, next_point.z - point.z]
        
        dot = sum(a*b for a, b in zip(v1, v2))
        mag1 = math.sqrt(sum(a*a for a in v1))
        mag2 = math.sqrt(sum(a*a for a in v2))
        
        if mag1 < 0.001 or mag2 < 0.001:
            return 0.0
            
        cos_angle = dot / (mag1 * mag2)
        cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp
        angle = math.degrees(math.acos(cos_angle))
        
        # Sharp corners need exact positioning
        if angle < WAAM_PARAMS['corner_angle_threshold']:
            return 0.0
            
        # Calculate blend based on geometry
        max_blend = min(
            dist_to_prev * 0.15,
            dist_to_next * 0.15,
            WAAM_PARAMS['max_blend_distance']
        )
        
        # Reduce blend for sharper corners
        angle_factor = (angle - WAAM_PARAMS['corner_angle_threshold']) / (180 - WAAM_PARAMS['corner_angle_threshold'])
        blend = max_blend * angle_factor
        
        return blend if blend > WAAM_PARAMS['min_blend_distance'] else 0.0
    
    def generate_torch_on(self) -> str:
        """Generate torch ON sequence"""
        return f"""CONTINUE
$OUT[{WAAM_PARAMS['torch_output']}] = TRUE
WAIT SEC {WAAM_PARAMS['arc_strike_delay']:.2f}

"""
    
    def generate_torch_off(self) -> str:
        """Generate torch OFF sequence"""
        return f"""CONTINUE
$OUT[{WAAM_PARAMS['torch_output']}] = FALSE
WAIT SEC {WAAM_PARAMS['arc_off_delay']:.2f}

"""
    
    def generate_weld_end_sequence(self, current_feed_rate: float) -> str:
        """Generate arc termination sequence"""
        crater_fill_speed = current_feed_rate * WAAM_PARAMS['end_speed_factor']
        return f""";--- WELD END SEQUENCE ---
; Slow for crater fill
$VEL.CP = {crater_fill_speed:.4f}
WAIT SEC 0.1

; Arc termination with CONTINUE
CONTINUE
$OUT[{WAAM_PARAMS['torch_output']}] = FALSE
WAIT SEC {WAAM_PARAMS['arc_off_delay']:.2f}

; Post-weld stabilization
WAIT SEC {WAAM_PARAMS['post_weld_delay']:.2f}

"""
    
    def generate_layer_transition(self, layer_num: int) -> str:
        """Generate inter-layer operations"""
        return f""";===========
; LAYER {layer_num} COMPLETE - Thermal Management
;===========
WAIT SEC {WAAM_PARAMS['inter_layer_delay']:.2f}

"""
    
    def generate_speed_ramp_to_full(self) -> str:
        """Ramp to full welding speed"""
        return f"""; Ramp to full welding speed
$VEL.CP = {self.feed_rate:.4f}

"""
    
    def generate_motion_command(self, point: Point, blend_dist: float, 
                               is_boundary: bool, segment_type: str) -> Optional[str]:
        """Generate KRL motion command with proper blending"""
        pos_str = self.format_pos(point.x, point.y, point.z)
        if pos_str is None:
            return None
        
        commands = []
        
        # PTP for rapid moves
        if point.move_type == MoveType.RAPID:
            commands.append(f"PTP {pos_str}\n")
            return "".join(commands)
        
        # LIN moves with blending logic
        if is_boundary:
            # Exact positioning at layer boundaries
            commands.append(f"LIN {pos_str}\n")
            commands.append("WAIT SEC 0  ; Exact positioning\n")
        elif blend_dist > WAAM_PARAMS['min_blend_distance']:
            # Apply calculated blend distance
            commands.append(f"$APO.CDIS = {blend_dist:.2f}\n")
            commands.append(f"LIN {pos_str} C_DIS\n")
        else:
            # Exact positioning for corners
            commands.append(f"LIN {pos_str}\n")
            
        return "".join(commands)
    
    def generate_krl(self, points: List[Point], program_name: str = "WAAM_PART") -> str:
        """Generate complete KRL program with proper torch control"""
        if not points:
            raise ValueError("No valid points to generate KRL")
        
        # Filter out duplicate/very close points
        filtered_points = [points[0]]
        for i in range(1, len(points)):
            dist = points[i].distance_to(filtered_points[-1])
            # Keep point if it's far enough OR if welding state changes
            if dist > 0.01 or points[i].welding != filtered_points[-1].welding:
                filtered_points.append(points[i])
        
        points = filtered_points
        
        krl_lines = []
        
        # Header
        header = HEADER_SRC.format(
            program_name=program_name,
            default_travel_speed=ROBOT_SPECS['default_travel_speed'],
            default_blend=WAAM_PARAMS['max_blend_distance'],
            torch_output=WAAM_PARAMS['torch_output']
        )
        krl_lines.append(header)
        
        # Detect layers and weld segments
        layer_boundaries = self.detect_layer_boundaries(points)
        weld_segments = self.identify_weld_segments(points)
        
        # State tracking
        prev_welding = False
        current_speed = ROBOT_SPECS['default_travel_speed']
        torch_is_on = False  # Track actual torch state
        layer_num = 0
        
        for i, point in enumerate(points):
            is_boundary = layer_boundaries[i]
            segment = weld_segments[i]
            
            # Layer transition at start of new layer
            if is_boundary and segment == 'start' and i > 0:
                layer_num += 1
                # Ensure torch is OFF during layer transition
                if torch_is_on:
                    krl_lines.append(f"\n; Turn OFF torch for layer transition\n")
                    krl_lines.append(self.generate_torch_off())
                    torch_is_on = False
                
                krl_lines.append(self.generate_layer_transition(layer_num))
            
            # TORCH CONTROL - Manage torch state based on welding status
            if point.welding and not torch_is_on:
                # Need to turn torch ON
                krl_lines.append(f"\n;--- TORCH ON - Start Welding ---\n")
                krl_lines.append(f"; Stop and stabilize\n")
                krl_lines.append(f"$VEL.CP = 0.1\n")
                krl_lines.append(f"WAIT SEC 0\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['pre_weld_delay']:.2f}\n\n")
                krl_lines.append(self.generate_torch_on())
                torch_is_on = True
                
                # Set initial welding speed (reduced for arc start)
                krl_lines.append(f"; Arc stabilization at reduced speed\n")
                start_speed = point.feed_rate * WAAM_PARAMS['start_speed_factor']
                krl_lines.append(f"$VEL.CP = {start_speed:.4f}\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['arc_stabilization']:.2f}\n\n")
                current_speed = start_speed
                
            elif not point.welding and torch_is_on:
                # Need to turn torch OFF
                krl_lines.append(f"\n;--- TORCH OFF - Stop Welding ---\n")
                
                # Slow down for crater fill if this was the end of a weld
                if prev_welding:
                    crater_speed = current_speed * WAAM_PARAMS['end_speed_factor']
                    krl_lines.append(f"; Slow for crater fill\n")
                    krl_lines.append(f"$VEL.CP = {crater_speed:.4f}\n")
                    krl_lines.append(f"WAIT SEC 0.1\n\n")
                
                krl_lines.append(self.generate_torch_off())
                torch_is_on = False
                
                # Set to travel speed
                krl_lines.append(f"; Post-weld stabilization\n")
                krl_lines.append(f"WAIT SEC {WAAM_PARAMS['post_weld_delay']:.2f}\n")
                current_speed = ROBOT_SPECS['default_travel_speed']
            
            # SPEED MANAGEMENT
            target_speed = point.feed_rate if point.welding else ROBOT_SPECS['default_travel_speed']
            
            # Change speed if needed (and not during arc start/stop sequences)
            if target_speed and abs(target_speed - current_speed) > 0.001:
                # Only change speed during stable welding or travel
                if point.welding == prev_welding:
                    speed_mm_s = target_speed * 1000
                    krl_lines.append(f"\n; Set speed to {speed_mm_s:.1f} mm/s\n")
                    krl_lines.append(f"$VEL.CP = {target_speed:.4f}\n")
                    current_speed = target_speed
            
            # MOTION COMMAND
            # Calculate blending (no blending at layer boundaries or state changes)
            blend_dist = 0.0
            if not is_boundary and point.welding == prev_welding:
                blend_dist = self.calculate_blend_distance(points, i)
            
            # Generate motion
            motion_cmd = self.generate_motion_command(point, blend_dist, is_boundary, segment)
            if motion_cmd:
                krl_lines.append(motion_cmd)
            
            prev_welding = point.welding
        
        # Ensure torch is OFF at end
        if torch_is_on:
            krl_lines.append(f"\n; Final torch OFF\n")
            krl_lines.append(self.generate_torch_off())
        
        # Footer
        footer = FOOTER_SRC.format(
            default_travel_speed=ROBOT_SPECS['default_travel_speed']
        )
        krl_lines.append(footer)
        
        return "".join(krl_lines)
    
    def generate_statistics(self, points: List[Point]) -> dict:
        """Calculate program statistics"""
        weld_points = [p for p in points if p.welding]
        travel_points = [p for p in points if not p.welding]
        
        # Calculate path lengths
        total_length = sum(points[i].distance_to(points[i+1]) 
                          for i in range(len(points)-1))
        weld_length = sum(weld_points[i].distance_to(weld_points[i+1]) 
                         for i in range(len(weld_points)-1) if i < len(weld_points)-1)
        
        # Count unique layers from layer_z values
        layer_z_values = set(p.layer_z for p in points if p.layer_z is not None)
        layer_count = len(layer_z_values)
        
        return {
            'total_points': len(points),
            'weld_points': len(weld_points),
            'travel_points': len(travel_points),
            'total_length_mm': total_length,
            'weld_length_mm': weld_length,
            'layer_count': layer_count,
            'warnings': len(self.warnings)
        }
    
    def process_file(self, input_path: str, output_path: str):
        """Process G-code file and generate KRL output"""
        points = []
        
        print(f"\n{'='*70}")
        print(f"  WAAM G-code to KRL Transpiler for KUKA KR6 R900-2")
        print(f"{'='*70}")
        print(f"\nInput:  {input_path}")
        
        # Get program name
        program_name = os.path.splitext(os.path.basename(input_path))[0].upper()
        
        # Parse G-code
        print("\nParsing G-code...")
        with open(input_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                point = self.parse_line(line)
                if point is not None:
                    points.append(point)
        
        if not points:
            print("\n ERROR: No valid G-code commands found!")
            return
        
        print(f" Parsed {len(points)} points")
        
        # Generate KRL
        print("\nGenerating KRL...")
        try:
            krl_content = self.generate_krl(points, program_name)
        except Exception as e:
            print(f"\n ERROR during KRL generation: {e}")
            import traceback
            traceback.print_exc()
            return
        
        # Write files
        print("\nWriting output files...")
        with open(output_path, 'w') as f:
            f.write(krl_content)
        
        dat_path = output_path.replace('.src', '.dat')
        with open(dat_path, 'w') as f:
            dat_content = HEADER_DAT.format(program_name=program_name)
            f.write(dat_content)
        
        # Display results
        stats = self.generate_statistics(points)
        
       
        print("   TRANSPILATION COMPLETE")

        print(f"\n Generated Files:")
        print(f"   • {output_path}")
        print(f"   • {dat_path}")
        print(f"\n Statistics:")
        print(f"   • Total points:      {stats['total_points']}")
        print(f"   • Weld points:       {stats['weld_points']}")
        print(f"   • Travel points:     {stats['travel_points']}")
        print(f"   • Total path:        {stats['total_length_mm']:.1f} mm")
        print(f"   • Weld path:         {stats['weld_length_mm']:.1f} mm")
        
        # Warnings
        if self.warnings:
            print(f"\n  Warnings ({len(self.warnings)}):")
            for warning in list(set(self.warnings))[:5]:  # Show first 5 unique
                print(f"   • {warning}")
            if len(set(self.warnings)) > 5:
                print(f"   ... and {len(set(self.warnings)) - 5} more")
        
     


def main():
    if len(sys.argv) != 2:
        print("\n" + "="*70)
        print("  WAAM G-code to KRL Transpiler")
        print("\nUsage:")
        print("  python transpiler.py <gcode_file>\n")
        print("Example:")
        print("  python transpiler.py wall_part.gcode\n")
        print("Output:")
        print("  • wall_part.src  (KRL source file)")
        print("  • wall_part.dat  (KRL data file)")
        print("\n" + "="*70 + "\n")
        sys.exit(1)
    
    gcode_file = sys.argv[1]
    if not os.path.isfile(gcode_file):
        print(f"\n ERROR: File not found: {gcode_file}\n")
        sys.exit(1)
    
    # Generate output filename
    base_name = os.path.splitext(os.path.basename(gcode_file))[0]
    src_file = base_name + ".src"
    
    # Run transpiler
    transpiler = WAAMTranspiler()
    transpiler.process_file(gcode_file, src_file)


if __name__ == "__main__":
    main()