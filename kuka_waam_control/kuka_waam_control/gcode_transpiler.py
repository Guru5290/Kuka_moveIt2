#!/usr/bin/env python3
"""
Enhanced G-code to KRL Transpiler for WAAM
Supports advanced WAAM features and better MoveIt2 integration
"""

import sys
import os
import re
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum

class MoveType(Enum):
    RAPID = "G0"
    LINEAR = "G1"
    ARC_CW = "G2"
    ARC_CCW = "G3"

@dataclass
class WeldPoint:
    x: float
    y: float
    z: float
    move_type: MoveType = MoveType.LINEAR
    welding: bool = False
    feed_rate: Optional[float] = None
    wire_feed: Optional[float] = None

@dataclass
class WAAMConfig:
    """Configuration for WAAM process"""
    threshold: float = 3.0  # mm - minimum distance between points
    layer_height: float = 2.0  # mm
    arc_strike_delay: float = 0.2  # seconds
    arc_crater_fill: float = 0.15  # seconds
    cooling_time: float = 30.0  # seconds
    travel_speed: float = 0.5  # m/s
    weld_speed: float = 0.008  # m/s
    safe_height_offset: float = 50.0  # mm
    use_moveit_planning: bool = True  # Use MoveIt for collision-free paths

# KRL Header with MoveIt integration
HEADER_SRC = """DEF gcode_waam()
;FOLD INI
  ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3 
    BAS (#INITMOV,0 )
  ;ENDFOLD (BASISTECH INI)
  ;FOLD USER INI
    ;Make your modifications here
  ;ENDFOLD (USER INI)
;ENDFOLD (INI)

; ************
; WAAM Process Control Program
; Generated from G-code
; ************

; Move to home position
PTP {A1 -12.16, A2 -29.24, A3 103.79, A4 7.33, A5 71.20, A6 -102.38}

; Initialize system variables
$POS_ACT_MES = $POS_ACT
$POS_RET = $POS_ACT

; Declare variables
DECL INT layer_count
DECL INT segment_count
DECL REAL x_offset
DECL REAL y_offset  
DECL REAL z_offset
DECL E6POS current_pos
DECL E6POS target_pos
DECL E6POS safe_pos
DECL BOOL welding_active
DECL BOOL user_confirmed

; Initialize counters and states
layer_count = 0
segment_count = 0
welding_active = FALSE
user_confirmed = FALSE

; Configure I/O
$OUT[1] = FALSE  ; Welding arc
$OUT[2] = FALSE  ; Wire feed
$OUT[3] = FALSE  ; Gas flow

; Check safety conditions
IF $IN[12] == TRUE THEN  ; Fault signal
  MsgNotify("FAULT DETECTED - Check system before continuing")
  HALT
ENDIF

IF $IN[10] == FALSE THEN  ; Welding enable
  MsgNotify("WELDING NOT ENABLED - Enable and restart")
  HALT
ENDIF

MsgNotify("WAAM System Initialized")
WAIT SEC 1.0

"""

HEADER_DAT = """;FOLD EXTERNAL DECLARATIONS
;ENDFOLD (EXTERNAL DECLARATIONS)

;FOLD DECLARATION
DECL E6POS XHOME
DECL REAL Global_Speed
DECL REAL Weld_Speed
DECL REAL Travel_Speed
DECL REAL Layer_Height
DECL REAL Arc_Strike_Delay
DECL REAL Arc_Crater_Fill
;ENDFOLD (DECLARATION)

;FOLD INITIALIZATION
XHOME = {X 0.0, Y 0.0, Z 600.0, A 180.0, B 0.0, C 90.0}
Global_Speed = 0.5       ; m/s
Weld_Speed = 0.008       ; m/s (8 mm/s typical for WAAM)
Travel_Speed = 0.5       ; m/s
Layer_Height = 2.0       ; mm
Arc_Strike_Delay = 0.2   ; seconds
Arc_Crater_Fill = 0.15   ; seconds
;ENDFOLD (INITIALIZATION)
"""

FOOTER_SRC = """
; *****
; WAAM Job Completed
; *****

; Turn off all welding outputs
$OUT[1] = FALSE  ; Welding arc
$OUT[2] = FALSE  ; Wire feed
$OUT[3] = FALSE  ; Gas flow

; Move to safe position
current_pos = $POS_ACT
safe_pos = current_pos
safe_pos.Z = safe_pos.Z + 100.0
LIN safe_pos C_DIS

; Return to home
PTP XHOME

; Final status message
MsgNotify("WAAM Job Completed Successfully!")

END
"""

class EnhancedTranspiler:
    """Enhanced G-code to KRL transpiler with WAAM support"""
    
    def __init__(self, config: WAAMConfig):
        self.config = config
        self.points: List[WeldPoint] = []
        self.layers: List[List[WeldPoint]] = []
        self.current_layer: List[WeldPoint] = []
        self.last_z: Optional[float] = None
        self.origin_set = False
        self.current_pos = [0.0, 0.0, 0.0]
        self.welding_active = False
        
    def distance(self, p1: Tuple[float, float, float], 
                 p2: Tuple[float, float, float]) -> float:
        """Calculate Euclidean distance between two points"""
        return np.linalg.norm(np.array(p2) - np.array(p1))
    
    def format_kuka_pos(self, x: float, y: float, z: float, 
                       a: float = 180.0, b: float = 0.0, 
                       c: float = 90.0) -> str:
        """Format position in KUKA E6POS format"""
        return f"{{X {x:.3f}, Y {y:.3f}, Z {z:.3f}, A {a:.1f}, B {b:.1f}, C {c:.1f}}}"
    
    def filter_points(self, points: List[WeldPoint]) -> List[WeldPoint]:
        """Filter points by threshold distance"""
        if not points:
            return []
        
        filtered = [points[0]]
        for point in points[1:]:
            last = filtered[-1]
            dist = self.distance(
                (last.x, last.y, last.z),
                (point.x, point.y, point.z)
            )
            if dist > self.config.threshold:
                filtered.append(point)
        
        return filtered
    
    def parse_line(self, line: str) -> Optional[WeldPoint]:
        """Parse a single G-code line"""
        line = line.strip().upper()
        
        # Skip comments and empty lines
        if not line or line.startswith(';') or line.startswith('('):
            return None
        
        # Determine move type
        move_type = MoveType.LINEAR
        if line.startswith('G0'):
            move_type = MoveType.RAPID
        elif line.startswith('G1'):
            move_type = MoveType.LINEAR
        elif line.startswith('G2'):
            move_type = MoveType.ARC_CW
        elif line.startswith('G3'):
            move_type = MoveType.ARC_CCW
        else:
            return None
        
        # Extract coordinates
        x_match = re.search(r'X([-+]?[0-9]*\.?[0-9]+)', line)
        y_match = re.search(r'Y([-+]?[0-9]*\.?[0-9]+)', line)
        z_match = re.search(r'Z([-+]?[0-9]*\.?[0-9]+)', line)
        f_match = re.search(r'F([-+]?[0-9]*\.?[0-9]+)', line)
        
        # Welding control
        weld_on = 'E2' in line or 'M3' in line
        weld_off = 'E0' in line or 'M5' in line
        
        x = float(x_match.group(1)) if x_match else self.current_pos[0]
        y = float(y_match.group(1)) if y_match else self.current_pos[1]
        z = float(z_match.group(1)) if z_match else self.current_pos[2]
        feed_rate = float(f_match.group(1)) if f_match else None
        
        # Update welding state
        if weld_on:
            self.welding_active = True
        elif weld_off:
            self.welding_active = False
        
        # Update current position
        self.current_pos = [x, y, z]
        
        return WeldPoint(
            x=x, y=y, z=z,
            move_type=move_type,
            welding=self.welding_active,
            feed_rate=feed_rate
        )
    
    def generate_welding_section(self, points: List[WeldPoint]) -> str:
        """Generate KRL for a welding section"""
        if not points:
            return ""
        
        krl = ["\n; --- WELDING SECTION START ---\n"]
        
        # Filter points
        filtered_points = self.filter_points(points)
        
        # Set origin offset from first point if not set
        if not self.origin_set and filtered_points:
            p = filtered_points[0]
            krl.append("; Calculate offset from first weld point\n")
            krl.append(f"x_offset = $POS_ACT.X - {p.x:.3f}\n")
            krl.append(f"y_offset = $POS_ACT.Y - {p.y:.3f}\n")
            krl.append(f"z_offset = $POS_ACT.Z - {p.z:.3f}\n\n")
            self.origin_set = True
        
        # Pre-weld positioning
        if filtered_points and filtered_points[0].welding:
            krl.append("; Pre-weld setup\n")
            krl.append("$OUT[3] = TRUE  ; Gas pre-flow\n")
            krl.append("WAIT SEC 1.0\n\n")
        
        # Generate movement commands
        for i, point in enumerate(filtered_points):
            krl.append(f"; Point {i+1}/{len(filtered_points)}\n")
            krl.append(f"target_pos = {self.format_kuka_pos(point.x, point.y, point.z)}\n")
            
            if self.origin_set:
                krl.append("target_pos.X = target_pos.X + x_offset\n")
                krl.append("target_pos.Y = target_pos.Y + y_offset\n")
                krl.append("target_pos.Z = target_pos.Z + z_offset\n")
            
            # Welding control
            if point.welding and i == 0:
                krl.append("\n; Start welding\n")
                krl.append("IF $IN[10] == TRUE THEN  ; Check welding enable\n")
                krl.append("  $OUT[2] = TRUE  ; Wire feed ON\n")
                krl.append("  WAIT SEC 0.1\n")
                krl.append("  $OUT[1] = TRUE  ; Arc ON\n")
                krl.append(f"  WAIT SEC {self.config.arc_strike_delay}\n")
                krl.append("  welding_active = TRUE\n")
                krl.append("ENDIF\n\n")
            
            # Movement command
            if point.move_type == MoveType.RAPID:
                krl.append("PTP target_pos C_DIS\n")
            else:
                krl.append("LIN target_pos C_DIS\n")
            
            krl.append("\n")
        
        # Post-weld
        if filtered_points and filtered_points[-1].welding:
            krl.append("; Crater fill and arc off\n")
            krl.append(f"WAIT SEC {self.config.arc_crater_fill}\n")
            krl.append("$OUT[1] = FALSE  ; Arc OFF\n")
            krl.append("WAIT SEC 0.2\n")
            krl.append("$OUT[2] = FALSE  ; Wire feed OFF\n")
            krl.append("WAIT SEC 0.5\n")
            krl.append("$OUT[3] = FALSE  ; Gas OFF\n")
            krl.append("welding_active = FALSE\n\n")
        
        krl.append("; === WELDING SECTION END ===\n\n")
        
        return "".join(krl)
    
    def generate_layer_transition(self, layer_num: int) -> str:
        """Generate KRL for layer transition"""
        krl = [f"\n; ========================================\n"]
        krl.append(f"; LAYER {layer_num} COMPLETE\n")
        krl.append(f"; ========================================\n\n")
        
        # Retract for cooling
        krl.append("; Retract for interlayer cooling\n")
        krl.append("current_pos = $POS_ACT\n")
        krl.append("safe_pos = current_pos\n")
        krl.append(f"safe_pos.Z = safe_pos.Z + {self.config.safe_height_offset}\n")
        krl.append("LIN safe_pos C_DIS\n\n")
        
        # Wait for cooling
        krl.append(f'MsgNotify("Layer {layer_num} completed. Cooling initiated.")\n')
        krl.append(f"WAIT SEC {self.config.cooling_time}\n\n")
        
        # User confirmation
        krl.append(f'MsgNotify("Ready for layer {layer_num + 1}. Press green button to continue.")\n')
        krl.append("user_confirmed = FALSE\n")
        krl.append("WAIT FOR $IN[11] == TRUE  ; Wait for user confirmation\n")
        krl.append("user_confirmed = TRUE\n\n")
        
        krl.append("; Return to build height\n")
        krl.append("LIN current_pos C_DIS\n\n")
        
        return "".join(krl)
    
    def parse_gcode(self, gcode_path: str) -> List[str]:
        """Parse G-code file and generate KRL commands"""
        krl_commands = [HEADER_SRC]
        
        current_segment = []
        layer_count = 0
        
        print(f"Parsing G-code: {gcode_path}")
        
        with open(gcode_path, 'r') as file:
            for line_num, line in enumerate(file, 1):
                point = self.parse_line(line)
                
                if point is None:
                    # Check for welding control commands
                    if 'E0' in line.upper() or 'M5' in line.upper():
                        if current_segment:
                            krl_commands.append(self.generate_welding_section(current_segment))
                            current_segment = []
                    continue
                
                # Check for layer transition
                if self.last_z is not None and point.z != self.last_z:
                    if current_segment:
                        krl_commands.append(self.generate_welding_section(current_segment))
                        current_segment = []
                    
                    if self.current_layer:
                        self.layers.append(self.current_layer)
                        layer_count += 1
                        krl_commands.append(self.generate_layer_transition(layer_count))
                        self.current_layer = []
                
                self.last_z = point.z
                self.current_layer.append(point)
                current_segment.append(point)
        
        # Handle final segment and layer
        if current_segment:
            krl_commands.append(self.generate_welding_section(current_segment))
        
        if self.current_layer:
            self.layers.append(self.current_layer)
            layer_count += 1
            krl_commands.append(f"; Final layer: {layer_count}\n")
        
        # Add footer
        krl_commands.append(FOOTER_SRC)
        
        print(f"Parsing complete:")
        print(f"  Total layers: {len(self.layers)}")
        print(f"  Total points: {sum(len(layer) for layer in self.layers)}")
        
        return krl_commands
    
    def write_krl(self, output_path: str, commands: List[str]):
        """Write KRL source and data files"""
        # Write SRC file
        with open(output_path, 'w') as f:
            f.write("".join(commands))
        
        # Write DAT file
        dat_path = output_path.replace('.src', '.dat')
        with open(dat_path, 'w') as f:
            f.write(HEADER_DAT)
        
        print(f"\nKRL files generated:")
        print(f"  Source: {output_path}")
        print(f"  Data:   {dat_path}")


def main():
    """Main transpiler function"""
    if len(sys.argv) < 3:
        print("Usage: python gcode_transpiler.py <gcode_file> <threshold> [options]")
        print("\nExample: python gcode_transpiler.py part.gcode 3.0")
        print("\nParameters:")
        print("  gcode_file : Path to input G-code file")
        print("  threshold  : Minimum distance between points (mm)")
        print("\nOptional:")
        print("  --layer-height FLOAT    : Layer height in mm (default: 2.0)")
        print("  --weld-speed FLOAT      : Welding speed in m/s (default: 0.008)")
        print("  --cooling-time FLOAT    : Cooling time in seconds (default: 30.0)")
        sys.exit(1)
    
    gcode_file = sys.argv[1]
    if not os.path.isfile(gcode_file):
        print(f"Error: File not found: {gcode_file}")
        sys.exit(1)
    
    threshold = float(sys.argv[2])
    
    # Parse optional arguments
    config = WAAMConfig(threshold=threshold)
    for i, arg in enumerate(sys.argv[3:], 3):
        if arg == '--layer-height' and i+1 < len(sys.argv):
            config.layer_height = float(sys.argv[i+1])
        elif arg == '--weld-speed' and i+1 < len(sys.argv):
            config.weld_speed = float(sys.argv[i+1])
        elif arg == '--cooling-time' and i+1 < len(sys.argv):
            config.cooling_time = float(sys.argv[i+1])
    
    print("\n" + "="*50)
    print("Enhanced G-code to KRL Transpiler for WAAM")
    print("="*50)
    print(f"\nInput file:     {gcode_file}")
    print(f"Threshold:      {config.threshold} mm")
    print(f"Layer height:   {config.layer_height} mm")
    print(f"Weld speed:     {config.weld_speed} m/s")
    print(f"Cooling time:   {config.cooling_time} s")
    print()
    
    # Create transpiler and parse
    transpiler = EnhancedTranspiler(config)
    commands = transpiler.parse_gcode(gcode_file)
    
    # Generate output filename
    src_filename = os.path.splitext(gcode_file)[0] + ".src"
    transpiler.write_krl(src_filename, commands)
    
    print("\n" + "="*50)
    print("IMPORTANT SAFETY NOTES")
    print("="*50)
    print("Before running on robot:")
    print("  1. Verify I/O assignments in robot configuration")
    print("  2. Test in T1 mode at 10% speed override")
    print("  3. Keep emergency stop accessible")
    print("  4. Verify welding parameters are appropriate")
    print("  5. Check collision geometry includes deposited material")
    print("\n" + "="*50)


if __name__ == "__main__":
    main()