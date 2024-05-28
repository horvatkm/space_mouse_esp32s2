#define DEBUG (0)

// Deadzone to filter out unintended movements. Increase if the mouse has small movements when it should be idle or the mouse is too senstive to subtle movements.
// Recommended to have this as small as possible for V2 to allow smaller knob range of motion.
#define DEADZONE 5 

// Axes are matched to pin order.
#define AX 0
#define AY 1
#define BX 2
#define BY 3
#define CX 4
#define CY 5
#define DX 6
#define DY 7

// Direction
// Modify the direction of translation/rotation depending on preference. This can also be done per application in the 3DConnexion software.
// Switch between true/false as desired.

// pan left/right
#define INVX false
// pan up/down
#define INVY false 
// zoom in/out
#define INVZ true
// Rotate around X axis (tilt front/back)
#define INVRX true
// Rotate around Y axis (tilt left/right)
#define INVRY false
// Rotate around Z axis (twist left/right) 
#define INVRZ true
