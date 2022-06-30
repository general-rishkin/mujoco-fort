module mod_glfw
  use iso_c_binding
  implicit none

  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ! ! GLFW API tokens
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  integer(c_int), parameter     :: GLFW_VERSION_MAJOR                = 3
  integer(c_int), parameter     :: GLFW_VERSION_MINOR                = 4
  integer(c_int), parameter     :: GLFW_VERSION_REVISION             = 0
  integer(c_int), parameter     :: GLFW_TRUE                         = 1
  integer(c_int), parameter     :: GLFW_FALSE                        = 0
  integer(c_int), parameter     :: GLFW_RELEASE                      = 0
  integer(c_int), parameter     :: GLFW_PRESS                        = 1
  integer(c_int), parameter     :: GLFW_REPEAT                       = 2
  integer(c_int), parameter     :: GLFW_HAT_CENTERED                 = 0
  integer(c_int), parameter     :: GLFW_HAT_UP                       = 1
  integer(c_int), parameter     :: GLFW_HAT_RIGHT                    = 2
  integer(c_int), parameter     :: GLFW_HAT_DOWN                     = 4
  integer(c_int), parameter     :: GLFW_HAT_LEFT                     = 8
  integer(c_int), parameter     :: GLFW_HAT_RIGHT_UP                 = ior(GLFW_HAT_RIGHT , GLFW_HAT_UP)
  integer(c_int), parameter     :: GLFW_HAT_RIGHT_DOWN               = ior(GLFW_HAT_RIGHT , GLFW_HAT_DOWN)
  integer(c_int), parameter     :: GLFW_HAT_LEFT_UP                  = ior(GLFW_HAT_LEFT  , GLFW_HAT_UP)
  integer(c_int), parameter     :: GLFW_HAT_LEFT_DOWN                = ior(GLFW_HAT_LEFT  , GLFW_HAT_DOWN)
  integer(c_int), parameter     :: GLFW_KEY_UNKNOWN                  = -1
  integer(c_int), parameter     :: GLFW_KEY_SPACE                    = 32
  integer(c_int), parameter     :: GLFW_KEY_APOSTROPHE               = 39  !! ' !!
  integer(c_int), parameter     :: GLFW_KEY_COMMA                    = 44  !! , !!
  integer(c_int), parameter     :: GLFW_KEY_MINUS                    = 45  !! - !!
  integer(c_int), parameter     :: GLFW_KEY_PERIOD                   = 46  !! . !!
  integer(c_int), parameter     :: GLFW_KEY_SLASH                    = 47  !! ! !!
  integer(c_int), parameter     :: GLFW_KEY_0                        = 48
  integer(c_int), parameter     :: GLFW_KEY_1                        = 49
  integer(c_int), parameter     :: GLFW_KEY_2                        = 50
  integer(c_int), parameter     :: GLFW_KEY_3                        = 51
  integer(c_int), parameter     :: GLFW_KEY_4                        = 52
  integer(c_int), parameter     :: GLFW_KEY_5                        = 53
  integer(c_int), parameter     :: GLFW_KEY_6                        = 54
  integer(c_int), parameter     :: GLFW_KEY_7                        = 55
  integer(c_int), parameter     :: GLFW_KEY_8                        = 56
  integer(c_int), parameter     :: GLFW_KEY_9                        = 57
  integer(c_int), parameter     :: GLFW_KEY_SEMICOLON                = 59  !! ; !!
  integer(c_int), parameter     :: GLFW_KEY_EQUAL                    = 61  !! = !!
  integer(c_int), parameter     :: GLFW_KEY_A                        = 65
  integer(c_int), parameter     :: GLFW_KEY_B                        = 66
  integer(c_int), parameter     :: GLFW_KEY_C                        = 67
  integer(c_int), parameter     :: GLFW_KEY_D                        = 68
  integer(c_int), parameter     :: GLFW_KEY_E                        = 69
  integer(c_int), parameter     :: GLFW_KEY_F                        = 70
  integer(c_int), parameter     :: GLFW_KEY_G                        = 71
  integer(c_int), parameter     :: GLFW_KEY_H                        = 72
  integer(c_int), parameter     :: GLFW_KEY_I                        = 73
  integer(c_int), parameter     :: GLFW_KEY_J                        = 74
  integer(c_int), parameter     :: GLFW_KEY_K                        = 75
  integer(c_int), parameter     :: GLFW_KEY_L                        = 76
  integer(c_int), parameter     :: GLFW_KEY_M                        = 77
  integer(c_int), parameter     :: GLFW_KEY_N                        = 78
  integer(c_int), parameter     :: GLFW_KEY_O                        = 79
  integer(c_int), parameter     :: GLFW_KEY_P                        = 80
  integer(c_int), parameter     :: GLFW_KEY_Q                        = 81
  integer(c_int), parameter     :: GLFW_KEY_R                        = 82
  integer(c_int), parameter     :: GLFW_KEY_S                        = 83
  integer(c_int), parameter     :: GLFW_KEY_T                        = 84
  integer(c_int), parameter     :: GLFW_KEY_U                        = 85
  integer(c_int), parameter     :: GLFW_KEY_V                        = 86
  integer(c_int), parameter     :: GLFW_KEY_W                        = 87
  integer(c_int), parameter     :: GLFW_KEY_X                        = 88
  integer(c_int), parameter     :: GLFW_KEY_Y                        = 89
  integer(c_int), parameter     :: GLFW_KEY_Z                        = 90
  integer(c_int), parameter     :: GLFW_KEY_LEFT_BRACKET             = 91  !! [ !!
  integer(c_int), parameter     :: GLFW_KEY_BACKSLASH                = 92  !! \ !!
  integer(c_int), parameter     :: GLFW_KEY_RIGHT_BRACKET            = 93  !! ] !!
  integer(c_int), parameter     :: GLFW_KEY_GRAVE_ACCENT             = 96  !! ` !!
  integer(c_int), parameter     :: GLFW_KEY_WORLD_1                  = 161 !! non-US #1 !!
  integer(c_int), parameter     :: GLFW_KEY_WORLD_2                  = 162 !! non-US #2 !!
  integer(c_int), parameter     :: GLFW_KEY_ESCAPE                   = 256
  integer(c_int), parameter     :: GLFW_KEY_ENTER                    = 257
  integer(c_int), parameter     :: GLFW_KEY_TAB                      = 258
  integer(c_int), parameter     :: GLFW_KEY_BACKSPACE                = 259
  integer(c_int), parameter     :: GLFW_KEY_INSERT                   = 260
  integer(c_int), parameter     :: GLFW_KEY_DELETE                   = 261
  integer(c_int), parameter     :: GLFW_KEY_RIGHT                    = 262
  integer(c_int), parameter     :: GLFW_KEY_LEFT                     = 263
  integer(c_int), parameter     :: GLFW_KEY_DOWN                     = 264
  integer(c_int), parameter     :: GLFW_KEY_UP                       = 265
  integer(c_int), parameter     :: GLFW_KEY_PAGE_UP                  = 266
  integer(c_int), parameter     :: GLFW_KEY_PAGE_DOWN                = 267
  integer(c_int), parameter     :: GLFW_KEY_HOME                     = 268
  integer(c_int), parameter     :: GLFW_KEY_END                      = 269
  integer(c_int), parameter     :: GLFW_KEY_CAPS_LOCK                = 280
  integer(c_int), parameter     :: GLFW_KEY_SCROLL_LOCK              = 281
  integer(c_int), parameter     :: GLFW_KEY_NUM_LOCK                 = 282
  integer(c_int), parameter     :: GLFW_KEY_PRINT_SCREEN             = 283
  integer(c_int), parameter     :: GLFW_KEY_PAUSE                    = 284
  integer(c_int), parameter     :: GLFW_KEY_F1                       = 290
  integer(c_int), parameter     :: GLFW_KEY_F2                       = 291
  integer(c_int), parameter     :: GLFW_KEY_F3                       = 292
  integer(c_int), parameter     :: GLFW_KEY_F4                       = 293
  integer(c_int), parameter     :: GLFW_KEY_F5                       = 294
  integer(c_int), parameter     :: GLFW_KEY_F6                       = 295
  integer(c_int), parameter     :: GLFW_KEY_F7                       = 296
  integer(c_int), parameter     :: GLFW_KEY_F8                       = 297
  integer(c_int), parameter     :: GLFW_KEY_F9                       = 298
  integer(c_int), parameter     :: GLFW_KEY_F10                      = 299
  integer(c_int), parameter     :: GLFW_KEY_F11                      = 300
  integer(c_int), parameter     :: GLFW_KEY_F12                      = 301
  integer(c_int), parameter     :: GLFW_KEY_F13                      = 302
  integer(c_int), parameter     :: GLFW_KEY_F14                      = 303
  integer(c_int), parameter     :: GLFW_KEY_F15                      = 304
  integer(c_int), parameter     :: GLFW_KEY_F16                      = 305
  integer(c_int), parameter     :: GLFW_KEY_F17                      = 306
  integer(c_int), parameter     :: GLFW_KEY_F18                      = 307
  integer(c_int), parameter     :: GLFW_KEY_F19                      = 308
  integer(c_int), parameter     :: GLFW_KEY_F20                      = 309
  integer(c_int), parameter     :: GLFW_KEY_F21                      = 310
  integer(c_int), parameter     :: GLFW_KEY_F22                      = 311
  integer(c_int), parameter     :: GLFW_KEY_F23                      = 312
  integer(c_int), parameter     :: GLFW_KEY_F24                      = 313
  integer(c_int), parameter     :: GLFW_KEY_F25                      = 314
  integer(c_int), parameter     :: GLFW_KEY_KP_0                     = 320
  integer(c_int), parameter     :: GLFW_KEY_KP_1                     = 321
  integer(c_int), parameter     :: GLFW_KEY_KP_2                     = 322
  integer(c_int), parameter     :: GLFW_KEY_KP_3                     = 323
  integer(c_int), parameter     :: GLFW_KEY_KP_4                     = 324
  integer(c_int), parameter     :: GLFW_KEY_KP_5                     = 325
  integer(c_int), parameter     :: GLFW_KEY_KP_6                     = 326
  integer(c_int), parameter     :: GLFW_KEY_KP_7                     = 327
  integer(c_int), parameter     :: GLFW_KEY_KP_8                     = 328
  integer(c_int), parameter     :: GLFW_KEY_KP_9                     = 329
  integer(c_int), parameter     :: GLFW_KEY_KP_DECIMAL               = 330
  integer(c_int), parameter     :: GLFW_KEY_KP_DIVIDE                = 331
  integer(c_int), parameter     :: GLFW_KEY_KP_MULTIPLY              = 332
  integer(c_int), parameter     :: GLFW_KEY_KP_SUBTRACT              = 333
  integer(c_int), parameter     :: GLFW_KEY_KP_ADD                   = 334
  integer(c_int), parameter     :: GLFW_KEY_KP_ENTER                 = 335
  integer(c_int), parameter     :: GLFW_KEY_KP_EQUAL                 = 336
  integer(c_int), parameter     :: GLFW_KEY_LEFT_SHIFT               = 340
  integer(c_int), parameter     :: GLFW_KEY_LEFT_CONTROL             = 341
  integer(c_int), parameter     :: GLFW_KEY_LEFT_ALT                 = 342
  integer(c_int), parameter     :: GLFW_KEY_LEFT_SUPER               = 343
  integer(c_int), parameter     :: GLFW_KEY_RIGHT_SHIFT              = 344
  integer(c_int), parameter     :: GLFW_KEY_RIGHT_CONTROL            = 345
  integer(c_int), parameter     :: GLFW_KEY_RIGHT_ALT                = 346
  integer(c_int), parameter     :: GLFW_KEY_RIGHT_SUPER              = 347
  integer(c_int), parameter     :: GLFW_KEY_MENU                     = 348
  integer(c_int), parameter     :: GLFW_KEY_LAST                     = GLFW_KEY_MENU
  integer(c_int), parameter     :: GLFW_MOD_SHIFT                    = int(z'0001', c_int)
  integer(c_int), parameter     :: GLFW_MOD_CONTROL                  = int(z'0002', c_int)
  integer(c_int), parameter     :: GLFW_MOD_ALT                      = int(z'0004', c_int)
  integer(c_int), parameter     :: GLFW_MOD_SUPER                    = int(z'0008', c_int)
  integer(c_int), parameter     :: GLFW_MOD_CAPS_LOCK                = int(z'0010', c_int)
  integer(c_int), parameter     :: GLFW_MOD_NUM_LOCK                 = int(z'0020', c_int)
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_1               = 0
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_2               = 1
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_3               = 2
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_4               = 3
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_5               = 4
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_6               = 5
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_7               = 6
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_8               = 7
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_LAST            = GLFW_MOUSE_BUTTON_8
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_LEFT            = GLFW_MOUSE_BUTTON_1
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_RIGHT           = GLFW_MOUSE_BUTTON_2
  integer(c_int), parameter     :: GLFW_MOUSE_BUTTON_MIDDLE          = GLFW_MOUSE_BUTTON_3
  integer(c_int), parameter     :: GLFW_JOYSTICK_1                   = 0
  integer(c_int), parameter     :: GLFW_JOYSTICK_2                   = 1
  integer(c_int), parameter     :: GLFW_JOYSTICK_3                   = 2
  integer(c_int), parameter     :: GLFW_JOYSTICK_4                   = 3
  integer(c_int), parameter     :: GLFW_JOYSTICK_5                   = 4
  integer(c_int), parameter     :: GLFW_JOYSTICK_6                   = 5
  integer(c_int), parameter     :: GLFW_JOYSTICK_7                   = 6
  integer(c_int), parameter     :: GLFW_JOYSTICK_8                   = 7
  integer(c_int), parameter     :: GLFW_JOYSTICK_9                   = 8
  integer(c_int), parameter     :: GLFW_JOYSTICK_10                  = 9
  integer(c_int), parameter     :: GLFW_JOYSTICK_11                  = 10
  integer(c_int), parameter     :: GLFW_JOYSTICK_12                  = 11
  integer(c_int), parameter     :: GLFW_JOYSTICK_13                  = 12
  integer(c_int), parameter     :: GLFW_JOYSTICK_14                  = 13
  integer(c_int), parameter     :: GLFW_JOYSTICK_15                  = 14
  integer(c_int), parameter     :: GLFW_JOYSTICK_16                  = 15
  integer(c_int), parameter     :: GLFW_JOYSTICK_LAST                = GLFW_JOYSTICK_16
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_A             = 0
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_B             = 1
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_X             = 2
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_Y             = 3
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_LEFT_BUMPER   = 4
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER  = 5
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_BACK          = 6
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_START         = 7
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_GUIDE         = 8
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_LEFT_THUMB    = 9
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_RIGHT_THUMB   = 10
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_DPAD_UP       = 11
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_DPAD_RIGHT    = 12
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_DPAD_DOWN     = 13
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_DPAD_LEFT     = 14
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_LAST          = GLFW_GAMEPAD_BUTTON_DPAD_LEFT
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_CROSS         = GLFW_GAMEPAD_BUTTON_A
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_CIRCLE        = GLFW_GAMEPAD_BUTTON_B
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_SQUARE        = GLFW_GAMEPAD_BUTTON_X
  integer(c_int), parameter     :: GLFW_GAMEPAD_BUTTON_TRIANGLE      = GLFW_GAMEPAD_BUTTON_Y
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_LEFT_X          = 0
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_LEFT_Y          = 1
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_RIGHT_X         = 2
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_RIGHT_Y         = 3
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_LEFT_TRIGGER    = 4
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER   = 5
  integer(c_int), parameter     :: GLFW_GAMEPAD_AXIS_LAST            = GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER
  integer(c_int), parameter     :: GLFW_NO_ERROR                     = 0
  integer(c_int), parameter     :: GLFW_NOT_INITIALIZED              = int(z'00010001', c_int)
  integer(c_int), parameter     :: GLFW_NO_CURRENT_CONTEXT           = int(z'00010002', c_int)
  integer(c_int), parameter     :: GLFW_INVALID_ENUM                 = int(z'00010003', c_int)
  integer(c_int), parameter     :: GLFW_INVALID_VALUE                = int(z'00010004', c_int)
  integer(c_int), parameter     :: GLFW_OUT_OF_MEMORY                = int(z'00010005', c_int)
  integer(c_int), parameter     :: GLFW_API_UNAVAILABLE              = int(z'00010006', c_int)
  integer(c_int), parameter     :: GLFW_VERSION_UNAVAILABLE          = int(z'00010007', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_ERROR               = int(z'00010008', c_int)
  integer(c_int), parameter     :: GLFW_FORMAT_UNAVAILABLE           = int(z'00010009', c_int)
  integer(c_int), parameter     :: GLFW_NO_WINDOW_CONTEXT            = int(z'0001000A', c_int)
  integer(c_int), parameter     :: GLFW_CURSOR_UNAVAILABLE           = int(z'0001000B', c_int)
  integer(c_int), parameter     :: GLFW_FEATURE_UNAVAILABLE          = int(z'0001000C', c_int)
  integer(c_int), parameter     :: GLFW_FEATURE_UNIMPLEMENTED        = int(z'0001000D', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_UNAVAILABLE         = int(z'0001000E', c_int)
  integer(c_int), parameter     :: GLFW_RESIZABLE                    = int(z'00020003', c_int)
  integer(c_int), parameter     :: GLFW_AUTO_ICONIFY                 = int(z'00020006', c_int)
  integer(c_int), parameter     :: GLFW_FLOATING                     = int(z'00020007', c_int)
  integer(c_int), parameter     :: GLFW_MAXIMIZED                    = int(z'00020008', c_int)
  integer(c_int), parameter     :: GLFW_CENTER_CURSOR                = int(z'00020009', c_int)
  integer(c_int), parameter     :: GLFW_TRANSPARENT_FRAMEBUFFER      = int(z'0002000A', c_int)
  integer(c_int), parameter     :: GLFW_HOVERED                      = int(z'0002000B', c_int)
  integer(c_int), parameter     :: GLFW_FOCUS_ON_SHOW                = int(z'0002000C', c_int)
  integer(c_int), parameter     :: GLFW_MOUSE_PASSTHROUGH            = int(z'0002000D', c_int)
  integer(c_int), parameter     :: GLFW_RED_BITS                     = int(z'00021001', c_int)
  integer(c_int), parameter     :: GLFW_GREEN_BITS                   = int(z'00021002', c_int)
  integer(c_int), parameter     :: GLFW_BLUE_BITS                    = int(z'00021003', c_int)
  integer(c_int), parameter     :: GLFW_ALPHA_BITS                   = int(z'00021004', c_int)
  integer(c_int), parameter     :: GLFW_DEPTH_BITS                   = int(z'00021005', c_int)
  integer(c_int), parameter     :: GLFW_STENCIL_BITS                 = int(z'00021006', c_int)
  integer(c_int), parameter     :: GLFW_ACCUM_RED_BITS               = int(z'00021007', c_int)
  integer(c_int), parameter     :: GLFW_ACCUM_GREEN_BITS             = int(z'00021008', c_int)
  integer(c_int), parameter     :: GLFW_ACCUM_BLUE_BITS              = int(z'00021009', c_int)
  integer(c_int), parameter     :: GLFW_ACCUM_ALPHA_BITS             = int(z'0002100A', c_int)
  integer(c_int), parameter     :: GLFW_AUX_BUFFERS                  = int(z'0002100B', c_int)
  integer(c_int), parameter     :: GLFW_STEREO                       = int(z'0002100C', c_int)
  integer(c_int), parameter     :: GLFW_SAMPLES                      = int(z'0002100D', c_int)
  integer(c_int), parameter     :: GLFW_SRGB_CAPABLE                 = int(z'0002100E', c_int)
  integer(c_int), parameter     :: GLFW_REFRESH_RATE                 = int(z'0002100F', c_int)
  integer(c_int), parameter     :: GLFW_DOUBLEBUFFER                 = int(z'00021010', c_int)
  integer(c_int), parameter     :: GLFW_CLIENT_API                   = int(z'00022001', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_VERSION_MAJOR        = int(z'00022002', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_VERSION_MINOR        = int(z'00022003', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_REVISION             = int(z'00022004', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_ROBUSTNESS           = int(z'00022005', c_int)
  integer(c_int), parameter     :: GLFW_OPENGL_FORWARD_COMPAT        = int(z'00022006', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_DEBUG                = int(z'00022007', c_int)
  integer(c_int), parameter     :: GLFW_OPENGL_DEBUG_CONTEXT         = GLFW_CONTEXT_DEBUG
  integer(c_int), parameter     :: GLFW_OPENGL_PROFILE               = int(z'00022008', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_RELEASE_BEHAVIOR     = int(z'00022009', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_NO_ERROR             = int(z'0002200A', c_int)
  integer(c_int), parameter     :: GLFW_CONTEXT_CREATION_API         = int(z'0002200B', c_int)
  integer(c_int), parameter     :: GLFW_SCALE_TO_MONITOR             = int(z'0002200C', c_int)
  integer(c_int), parameter     :: GLFW_COCOA_RETINA_FRAMEBUFFER     = int(z'00023001', c_int)
  integer(c_int), parameter     :: GLFW_COCOA_FRAME_NAME             = int(z'00023002', c_int)
  integer(c_int), parameter     :: GLFW_COCOA_GRAPHICS_SWITCHING     = int(z'00023003', c_int)
  integer(c_int), parameter     :: GLFW_X11_CLASS_NAME               = int(z'00024001', c_int)
  integer(c_int), parameter     :: GLFW_X11_INSTANCE_NAME            = int(z'00024002', c_int)
  integer(c_int), parameter     :: GLFW_WIN32_KEYBOARD_MENU          = int(z'00025001', c_int)
  integer(c_int), parameter     :: GLFW_NO_API                       =          0
  integer(c_int), parameter     :: GLFW_OPENGL_API                   = int(z'00030001', c_int)
  integer(c_int), parameter     :: GLFW_OPENGL_ES_API                = int(z'00030002', c_int)
  integer(c_int), parameter     :: GLFW_NO_ROBUSTNESS                =          0
  integer(c_int), parameter     :: GLFW_NO_RESET_NOTIFICATION        = int(z'00031001', c_int)
  integer(c_int), parameter     :: GLFW_LOSE_CONTEXT_ON_RESET        = int(z'00031002', c_int)
  integer(c_int), parameter     :: GLFW_OPENGL_ANY_PROFILE           =          0
  integer(c_int), parameter     :: GLFW_OPENGL_CORE_PROFILE          = int(z'00032001', c_int)
  integer(c_int), parameter     :: GLFW_OPENGL_COMPAT_PROFILE        = int(z'00032002', c_int)
  integer(c_int), parameter     :: GLFW_CURSOR                       = int(z'00033001', c_int)
  integer(c_int), parameter     :: GLFW_STICKY_KEYS                  = int(z'00033002', c_int)
  integer(c_int), parameter     :: GLFW_STICKY_MOUSE_BUTTONS         = int(z'00033003', c_int)
  integer(c_int), parameter     :: GLFW_LOCK_KEY_MODS                = int(z'00033004', c_int)
  integer(c_int), parameter     :: GLFW_RAW_MOUSE_MOTION             = int(z'00033005', c_int)
  integer(c_int), parameter     :: GLFW_CURSOR_NORMAL                = int(z'00034001', c_int)
  integer(c_int), parameter     :: GLFW_CURSOR_HIDDEN                = int(z'00034002', c_int)
  integer(c_int), parameter     :: GLFW_CURSOR_DISABLED              = int(z'00034003', c_int)
  integer(c_int), parameter     :: GLFW_ANY_RELEASE_BEHAVIOR         =          0
  integer(c_int), parameter     :: GLFW_RELEASE_BEHAVIOR_FLUSH       = int(z'00035001', c_int)
  integer(c_int), parameter     :: GLFW_RELEASE_BEHAVIOR_NONE        = int(z'00035002', c_int)
  integer(c_int), parameter     :: GLFW_NATIVE_CONTEXT_API           = int(z'00036001', c_int)
  integer(c_int), parameter     :: GLFW_EGL_CONTEXT_API              = int(z'00036002', c_int)
  integer(c_int), parameter     :: GLFW_OSMESA_CONTEXT_API           = int(z'00036003', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_NONE     = int(z'00037001', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_OPENGL   = int(z'00037002', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_OPENGLES = int(z'00037003', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_D3D9     = int(z'00037004', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_D3D11    = int(z'00037005', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_VULKAN   = int(z'00037007', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE_METAL    = int(z'00037008', c_int)
  integer(c_int), parameter     :: GLFW_ARROW_CURSOR                 = int(z'00036001', c_int)
  integer(c_int), parameter     :: GLFW_IBEAM_CURSOR                 = int(z'00036002', c_int)
  integer(c_int), parameter     :: GLFW_CROSSHAIR_CURSOR             = int(z'00036003', c_int)
  integer(c_int), parameter     :: GLFW_POINTING_HAND_CURSOR         = int(z'00036004', c_int)
  integer(c_int), parameter     :: GLFW_RESIZE_EW_CURSOR             = int(z'00036005', c_int)
  integer(c_int), parameter     :: GLFW_RESIZE_NS_CURSOR             = int(z'00036006', c_int)
  integer(c_int), parameter     :: GLFW_RESIZE_NWSE_CURSOR           = int(z'00036007', c_int)
  integer(c_int), parameter     :: GLFW_RESIZE_NESW_CURSOR           = int(z'00036008', c_int)
  integer(c_int), parameter     :: GLFW_RESIZE_ALL_CURSOR            = int(z'00036009', c_int)
  integer(c_int), parameter     :: GLFW_NOT_ALLOWED_CURSOR           = int(z'0003600A', c_int)
  integer(c_int), parameter     :: GLFW_HRESIZE_CURSOR               = GLFW_RESIZE_EW_CURSOR
  integer(c_int), parameter     :: GLFW_VRESIZE_CURSOR               = GLFW_RESIZE_NS_CURSOR
  integer(c_int), parameter     :: GLFW_HAND_CURSOR                  = GLFW_POINTING_HAND_CURSOR  
  integer(c_int), parameter     :: GLFW_CONNECTED                    = int(z'00040001', c_int)
  integer(c_int), parameter     :: GLFW_DISCONNECTED                 = int(z'00040002', c_int)
  integer(c_int), parameter     :: GLFW_JOYSTICK_HAT_BUTTONS         = int(z'00050001', c_int)
  integer(c_int), parameter     :: GLFW_ANGLE_PLATFORM_TYPE          = int(z'00050002', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM                     = int(z'00050003', c_int)
  integer(c_int), parameter     :: GLFW_COCOA_CHDIR_RESOURCES        = int(z'00051001', c_int)
  integer(c_int), parameter     :: GLFW_COCOA_MENUBAR                = int(z'00051002', c_int)
  integer(c_int), parameter     :: GLFW_X11_XCB_VULKAN_SURFACE       = int(z'00052001', c_int)
  integer(c_int), parameter     :: GLFW_ANY_PLATFORM                 = int(z'00060000', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_WIN32               = int(z'00060001', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_COCOA               = int(z'00060002', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_WAYLAND             = int(z'00060003', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_X11                 = int(z'00060004', c_int)
  integer(c_int), parameter     :: GLFW_PLATFORM_NULL                = int(z'00060005', c_int)  
  integer(c_int), parameter     :: GLFW_DONT_CARE                    = -1
  

  ! typedef struct GLFWwindow GLFWwindow;
  type, bind(c) :: GLFWwindow
  end type GLFWwindow

  ! typedef struct GLFWmonitor GLFWmonitor;
  type, bind(c) :: GLFWmonitor
  end type GLFWmonitor

  abstract interface
    ! typedef void (* GLFWkeyfun)(GLFWwindow* window, int key, int scancode, int action, int mods);
    subroutine GLFWkeyfun(window, key, scancode, action, mods) bind(c)!, name="GLFWkeyfun")
      import :: GLFWwindow, c_int
      type(GLFWwindow), intent(inout)   :: window
      integer(c_int), value, intent(in) :: key, scancode, action, mods
    end subroutine GLFWkeyfun

    ! typedef void (* GLFWcursorposfun)(GLFWwindow* window, double xpos, double ypos);
    subroutine GLFWcursorposfun(window, xpos, ypos) bind(c)!, name="GLFWcursorposfun")
      import :: GLFWwindow, c_double
      type(GLFWwindow), intent(inout)   :: window
      real(c_double), value, intent(in) :: xpos, ypos
    end subroutine GLFWcursorposfun

    ! typedef void (* GLFWmousebuttonfun)(GLFWwindow* window, int button, int action, int mods);
    subroutine GLFWmousebuttonfun(window, button, action, mods) bind(c)!, name="GLFWmousebuttonfun")
      import :: GLFWwindow, c_int
      type(GLFWwindow), intent(inout)   :: window
      integer(c_int), value, intent(in) :: button, action, mods
    end subroutine GLFWmousebuttonfun

    ! typedef void (* GLFWscrollfun)(GLFWwindow* window, double xoffset, double yoffset);
    subroutine GLFWscrollfun(window, xoffset, yoffset) bind(c)!, name="GLFWscrollfun")
      import :: GLFWwindow, c_double
      type(GLFWwindow), intent(inout)   :: window
      real(c_double), value, intent(in) :: xoffset, yoffset
    end subroutine GLFWscrollfun
  end interface

  interface
    ! GLFWAPI int glfwInit(void);
    integer(c_int) function glfwInit() bind(c, name="glfwInit")
      import :: c_int
    end function glfwInit

    ! GLFWAPI GLFWwindow* glfwCreateWindow(int width, int height, const char* title, GLFWmonitor* monitor, GLFWwindow* share);
    type(c_ptr) function glfwCreateWindow(width, height, title, monitor, share) bind(c, name="glfwCreateWindow")
      import :: c_ptr, c_int, c_char, GLFWmonitor, GLFWwindow
      integer(c_int), value, intent(in)           :: width, height
      character(c_char), intent(in)               :: title(*)
      type(c_ptr), value                          :: monitor
      type(c_ptr), value                          :: share
    end function glfwCreateWindow

    ! GLFWAPI void glfwMakeContextCurrent(GLFWwindow* window);
    subroutine glfwMakeContextCurrent(window) bind(c, name="glfwMakeContextCurrent")
      import :: c_ptr
      type(c_ptr), value             :: window
    end subroutine glfwMakeContextCurrent

    ! GLFWAPI void glfwSwapInterval(int interval);
    subroutine glfwSwapInterval(interval) bind(c, name="glfwSwapInterval")
      import :: c_int
      integer(c_int), value, intent(in)           :: interval
    end subroutine glfwSwapInterval

    ! GLFWAPI GLFWkeyfun glfwSetKeyCallback(GLFWwindow* window, GLFWkeyfun callback);
    type(c_funptr) function glfwSetKeyCallback(window, callback) bind(c, name="glfwSetKeyCallback")
      import :: c_funptr, GLFWwindow
      type(GLFWwindow), intent(inout)             :: window
      type(c_funptr), intent(in)                  :: callback
    end function glfwSetKeyCallback

    ! GLFWAPI GLFWcursorposfun glfwSetCursorPosCallback(GLFWwindow* window, GLFWcursorposfun callback);
    type(c_funptr) function glfwSetCursorPosCallback(window, callback) bind(c, name="glfwSetCursorPosCallback")
      import :: c_funptr, GLFWwindow
      type(GLFWwindow), intent(inout)             :: window
      type(c_funptr), intent(in)                  :: callback
    end function glfwSetCursorPosCallback

    ! GLFWAPI void glfwGetWindowSize(GLFWwindow* window, int* width, int* height);
    subroutine glfwGetWindowSize(window, width, height) bind(c, name="glfwGetWindowSize")
      import :: GLFWwindow, c_int
      type(GLFWwindow), intent(inout)     :: window
      integer(c_int), intent(out)         :: width, height
    end subroutine glfwGetWindowSize

    ! GLFWAPI int glfwGetKey(GLFWwindow* window, int key);
    integer(c_int) function glfwGetKey(window, key) bind(c, name="glfwGetKey")
      import :: c_int, GLFWwindow
      type(GLFWwindow), intent(inout)     :: window
      integer(c_int), value, intent(in)   :: key
    end function glfwGetKey

    ! GLFWAPI GLFWmousebuttonfun glfwSetMouseButtonCallback(GLFWwindow* window, GLFWmousebuttonfun callback);
    type(c_funptr) function glfwSetMouseButtonCallback(window, callback) bind(c, name="glfwSetMouseButtonCallback")
      import :: c_funptr, GLFWwindow
      type(GLFWwindow), intent(inout)     :: window
      type(c_funptr), intent(in)          :: callback
    end function glfwSetMouseButtonCallback


    ! GLFWAPI int glfwGetMouseButton(GLFWwindow* window, int button);
    integer(c_int) function glfwGetMouseButton(window, button) bind(c, name="glfwGetMouseButton")
      import :: c_int, GLFWwindow
      type(GLFWwindow), intent(inout)   :: window
      integer(c_int), value, intent(in) :: button
    end function glfwGetMouseButton

    ! GLFWAPI void glfwGetCursorPos(GLFWwindow* window, double* xpos, double* ypos);
    subroutine glfwGetCursorPos(window, xpos, ypos) bind(c, name="glfwGetCursorPos")
      import :: c_double, GLFWwindow
      type(GLFWwindow), intent(inout)   :: window
      real(c_double), intent(out)       :: xpos, ypos
    end subroutine glfwGetCursorPos

    ! GLFWAPI GLFWscrollfun glfwSetScrollCallback(GLFWwindow* window, GLFWscrollfun callback);
    type(c_funptr) function glfwSetScrollCallback(window, callback) bind(c, name="glfwSetScrollCallback")
      import :: c_funptr, GLFWwindow
      type(GLFWwindow), intent(inout)   :: window
      type(c_funptr), intent(in)        :: callback
    end function glfwSetScrollCallback

    ! GLFWAPI int glfwWindowShouldClose(GLFWwindow* window);
    integer(c_int) function glfwWindowShouldClose(window) bind(c, name="glfwWindowShouldClose")
      import :: c_int, c_ptr
      type(c_ptr), value                :: window
    end function glfwWindowShouldClose

    ! GLFWAPI void glfwGetFramebufferSize(GLFWwindow* window, int* width, int* height);
    subroutine glfwGetFramebufferSize(window, width, height) bind(c, name="glfwGetFramebufferSize")
      import :: c_ptr, c_int
      type(c_ptr), value                :: window
      integer(c_int), intent(inout)     :: width, height
    end subroutine glfwGetFramebufferSize
    ! subroutine glfwGetFramebufferSize(window, width, height) bind(c, name="glfwGetFramebufferSize")
    !   import :: c_ptr, c_int
    !   type(c_ptr), value           :: window
    !   type(c_ptr), value     :: width, height
    ! end subroutine glfwGetFramebufferSize

    ! GLFWAPI void glfwSwapBuffers(GLFWwindow* window);
    subroutine glfwSwapBuffers(window) bind(c, name="glfwSwapBuffers")
      import :: c_ptr
      type(c_ptr), value   :: window
    end subroutine glfwSwapBuffers

    ! GLFWAPI void glfwPollEvents(void);
    subroutine glfwPollEvents() bind(c, name="glfwPollEvents")
    end subroutine glfwPollEvents

  end interface
contains
  
end module mod_glfw