; Simple square weld pattern
G90             ; Absolute mode
G0 X0 Y0 Z50    ; Move to start position (safe height)
G0 Z10          ; Lower to near work surface
M3              ; Arc ON
G1 X100 Y0 Z5 F600     ; First side
G1 X100 Y100 Z5        ; Second side
G1 X0 Y100 Z5          ; Third side
G1 X0 Y0 Z5            ; Fourth side
M5              ; Arc OFF
G0 Z50          ; Retract to safe height