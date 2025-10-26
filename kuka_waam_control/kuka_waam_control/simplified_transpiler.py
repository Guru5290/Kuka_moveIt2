#!/usr/bin/env python3
"""
Simplified G-code to KRL Transpiler for WAAM Testing
Focuses on smooth motion and basic welding torch control
"""

import sys
import os
import re
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum

class MoveType(Enum):
    RAPID = "G0"
    LINEAR = "G1"

@dataclass
class Point:
    x: float
    y: float
    z: float
    move_type: MoveType = MoveType.LINEAR
    welding: bool = False
    feed_rate: Optional[float] = None

HEADER_SRC = """DEF gcode_test()
;FOLD INI
  ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3 
    BAS (#INITMOV,0 )
  ;ENDFOLD (BASISTECH INI)
;ENDFOLD (INI)

; Initialize system
BAS (#VEL_PTP,100)  ; Set PTP velocity
BAS (#ACC_PTP,100)  ; Set PTP acceleration
$APO.CDIS = 0.5  ; Set blending distance

; Set default speeds
$VEL.CP = 0.3  ; m/s for travel moves
BAS (#VEL_CP,0.3)  ; Set cartesian velocity
BAS (#ACC_CP,0.3)  ; Set cartesian acceleration

; Move to safe start position
PTP XHOME

; Initialize welding output
$OUT[1] = FALSE  ; Welding torch OFF

"""

HEADER_DAT = """;FOLD EXTERNAL DECLARATIONS
;ENDFOLD (EXTERNAL DECLARATIONS)

DEFDAT  gcode_test
; Initialization Data
DECL E6POS XHOME={X 0,Y 0,Z 300,A 180,B 0,C 90,S 2,T 34}  ; TBD
ENDDAT
"""

FOOTER_SRC = """
; Ensure welding is off
$OUT[1] = FALSE  ; Welding torch OFF

; Return to home position
$VEL.CP = 0.3  ; Reset to travel speed
PTP XHOME

END
"""

class SimplifiedTranspiler:
    def __init__(self):
        self.current_pos = [0.0, 0.0, 0.0]
        self.welding_active = False
        
    def format_pos(self, x: float, y: float, z: float) -> str:
        """Format position for KRL with fixed orientation"""
        return f"{{X {x:.3f},Y {y:.3f},Z {z:.3f},A 180,B 0,C 90}}"
    
    def parse_line(self, line: str) -> Optional[Point]:
        """Parse a single G-code line"""
        line = line.strip().upper()
        
        if not line or line.startswith(';') or line.startswith('('):
            return None
            
        # Determine move type
        move_type = MoveType.LINEAR
        if line.startswith('G0'):
            move_type = MoveType.RAPID
        elif line.startswith('G1'):
            move_type = MoveType.LINEAR
        else:
            return None
            
        # Extract coordinates
        x_match = re.search(r'X([-+]?[0-9]*\.?[0-9]+)', line)
        y_match = re.search(r'Y([-+]?[0-9]*\.?[0-9]+)', line)
        z_match = re.search(r'Z([-+]?[0-9]*\.?[0-9]+)', line)
        f_match = re.search(r'F([-+]?[0-9]*\.?[0-9]+)', line)
        
        # Check for welding commands (E or M codes)
        weld_on = 'M3' in line or 'E1' in line
        weld_off = 'M5' in line or 'E0' in line
        
        x = float(x_match.group(1)) if x_match else self.current_pos[0]
        y = float(y_match.group(1)) if y_match else self.current_pos[1]
        z = float(z_match.group(1)) if z_match else self.current_pos[2]
        feed_rate = float(f_match.group(1))/1000.0 if f_match else None  # Convert mm/min to m/s for feed rate
        
        # Update welding state
        if weld_on:
            self.welding_active = True
        elif weld_off:
            self.welding_active = False
            
        # Update current position
        self.current_pos = [x, y, z]
        
        return Point(
            x=x, y=y, z=z,
            move_type=move_type,
            welding=self.welding_active,
            feed_rate=feed_rate
        )
    
    def generate_krl(self, points: List[Point]) -> List[str]:
        """Generate KRL commands from parsed points"""
        krl = [HEADER_SRC]
        
        for i, point in enumerate(points):
            # Set speed before motion if specified
            if point.feed_rate is not None:
                krl.append(f"$VEL.CP = {point.feed_rate:.3f}  ; Adjusted feed rate\n")
            
            # Handle welding state changes
            if point.welding and not (i > 0 and points[i-1].welding):
                krl.append("\n; Enable welding torch\n")
                krl.append("$OUT[1] = TRUE\n")
                krl.append("WAIT SEC 0.2  ; Arc strike delay\n")
            elif not point.welding and (i > 0 and points[i-1].welding):
                krl.append("\n; Disable welding torch\n")
                krl.append("WAIT SEC 0.1  ; Short delay before arc off\n")
                krl.append("$OUT[1] = FALSE\n")
                krl.append("WAIT SEC 0.2  ; Arc off delay\n")
            
            # Generate motion command
            pos = self.format_pos(point.x, point.y, point.z)
            if point.move_type == MoveType.RAPID:
                krl.append(f"PTP {pos} CONT\n")
            else:
                krl.append(f"LIN {pos} CONT\n")
        
        krl.append(FOOTER_SRC)
        return krl
        
    def process_file(self, input_path: str, output_path: str):
        """Process G-code file and generate KRL output"""
        points = []
        
        print(f"Processing: {input_path}")
        
        with open(input_path, 'r') as f:
            for line in f:
                point = self.parse_line(line)
                if point is not None:
                    points.append(point)
        
        # Generate KRL
        krl_commands = self.generate_krl(points)
        
        # Write SRC file
        with open(output_path, 'w') as f:
            f.write("".join(krl_commands))
            
        # Write DAT file
        dat_path = output_path.replace('.src', '.dat')
        with open(dat_path, 'w') as f:
            f.write(HEADER_DAT)
            
        print(f"\nGenerated KRL files:")
        print(f"  Source: {output_path}")
        print(f"  Data:   {dat_path}")
        print(f"\nTotal points: {len(points)}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python simplified_transpiler.py <gcode_file>")
        sys.exit(1)
        
    gcode_file = sys.argv[1]
    if not os.path.isfile(gcode_file):
        print(f"Error: File not found: {gcode_file}")
        sys.exit(1)
        
    # Generate output filename
    src_file = os.path.splitext(gcode_file)[0] + ".src"
    
    print("\nSimplified G-code to KRL Transpiler")
    print("       ")
    
    transpiler = SimplifiedTranspiler()
    transpiler.process_file(gcode_file, src_file)
    
    print("\nIMPORTANT:")
    print("1. Test in T1 mode first at low speed")
    print("2. Verify welding torch control (OUT[1])")


if __name__ == "__main__":
    main()