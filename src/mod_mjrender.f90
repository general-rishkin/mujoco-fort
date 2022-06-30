module mod_mjrender
  use iso_c_binding
  use mod_mjmodel
  implicit none
  
  integer(c_int), parameter        :: mjNAUX       = 10        !! number of auxiliary buffers
  integer(c_int), parameter        :: mjMAXTEXTURE = 1000      !! maximum number of textures

  !---------------------------------- primitive types (mjt) -----------------------------------------

  enum, bind(c) !! mjtGridPos  !!        !! grid position for overlay
    enumerator :: mjGRID_TOPLEFT      = 0        !! top left
    enumerator :: mjGRID_TOPRIGHT                !! top right
    enumerator :: mjGRID_BOTTOMLEFT              !! bottom left
    enumerator :: mjGRID_BOTTOMRIGHT              !! bottom right
  end enum !! mjtGridPos


  enum, bind(c) !! mjtFramebuffer  !!      !! OpenGL framebuffer option
    enumerator :: mjFB_WINDOW         = 0        !! default/window buffer
    enumerator :: mjFB_OFFSCREEN                  !! offscreen buffer
  end enum !! mjtFramebuffer


  enum, bind(c) !! mjtFontScale  !!        !! font scale used at context creation
    enumerator :: mjFONTSCALE_50      = 50       !! 50% scale suitable for low-res rendering
    enumerator :: mjFONTSCALE_100     = 100      !! normal scale suitable in the absence of DPI scaling
    enumerator :: mjFONTSCALE_150     = 150      !! 150% scale
    enumerator :: mjFONTSCALE_200     = 200      !! 200% scale
    enumerator :: mjFONTSCALE_250     = 250      !! 250% scale
    enumerator :: mjFONTSCALE_300     = 300       !! 300% scale
  end enum !! mjtFontScale


  enum, bind(c) !! mjtFont  !!           !! font type used at each text operation
    enumerator :: mjFONT_NORMAL       = 0        !! normal font
    enumerator :: mjFONT_SHADOW                  !! normal font with shadow (for higher contrast)
    enumerator :: mjFONT_BIG                      !! big font (for user alerts)
  end enum !! mjtFont


  type, bind(c) :: mjrRect  !!                 !! OpenGL rectangle
    integer(c_int)                      :: left                       !! left (usually 0)
    integer(c_int)                      :: bottom                     !! bottom (usually 0)
    integer(c_int)                      :: width                      !! width (usually buffer width)
    integer(c_int)                      :: height                     !! height (usually buffer height)
  end type mjrRect


  !---------------------------------- mjrContext ----------------------------------------------------

  type, bind(c) :: mjrContext  !!              !! custom OpenGL context
    !! parameters copied from mjVisual
    real(c_float)                       :: lineWidth                !! line width for wireframe rendering
    real(c_float)                       :: shadowClip               !! clipping radius for directional lights
    real(c_float)                       :: shadowScale              !! fraction of light cutoff for spot lights
    real(c_float)                       :: fogStart                 !! fog start = stat.extent * vis.map.fogstart
    real(c_float)                       :: fogEnd                   !! fog end = stat.extent * vis.map.fogend
    real(c_float)                       :: fogRGBA(4)               !! fog rgba
    integer(c_int)                      :: shadowSize                 !! size of shadow map texture
    integer(c_int)                      :: offWidth                   !! width of offscreen buffer
    integer(c_int)                      :: offHeight                  !! height of offscreen buffer
    integer(c_int)                      :: offSamples                 !! number of offscreen buffer multisamples

    !! parameters specified at creation
    integer(c_int)                      :: fontScale                  !! font scale
    integer(c_int)                      :: auxWidth(mjNAUX)           !! auxiliary buffer width
    integer(c_int)                      :: auxHeight(mjNAUX)          !! auxiliary buffer height
    integer(c_int)                      :: auxSamples(mjNAUX)         !! auxiliary buffer multisamples

    !! offscreen rendering objects
    integer(c_int)                      :: offFBO            !! offscreen framebuffer object
    integer(c_int)                      :: offFBO_r          !! offscreen framebuffer for resolving multisamples
    integer(c_int)                      :: offColor          !! offscreen color buffer
    integer(c_int)                      :: offColor_r        !! offscreen color buffer for resolving multisamples
    integer(c_int)                      :: offDepthStencil   !! offscreen depth and stencil buffer
    integer(c_int)                      :: offDepthStencil_r !! offscreen depth and stencil buffer for resolving multisamples

    !! shadow rendering objects
    integer(c_int)                      :: shadowFBO         !! shadow map framebuffer object
    integer(c_int)                      :: shadowTex         !! shadow map texture

    !! auxiliary buffers
    integer(c_int)                      :: auxFBO(mjNAUX)    !! auxiliary framebuffer object
    integer(c_int)                      :: auxFBO_r(mjNAUX)  !! auxiliary framebuffer object for resolving
    integer(c_int)                      :: auxColor(mjNAUX)  !! auxiliary color buffer
    integer(c_int)                      :: auxColor_r(mjNAUX)!! auxiliary color buffer for resolving

    !! texture objects and info
    integer(c_int)                      :: ntexture                   !! number of allocated textures
    integer(c_int)                      :: textureType(100)           !! type of texture (mjtTexture)
    integer(c_int)                      :: texture(100)      !! texture names

    !! displaylist starting positions
    integer(c_int)                      :: basePlane         !! all planes from model
    integer(c_int)                      :: baseMesh          !! all meshes from model
    integer(c_int)                      :: baseHField        !! all hfields from model
    integer(c_int)                      :: baseBuiltin       !! all buildin geoms with quality from model
    integer(c_int)                      :: baseFontNormal    !! normal font
    integer(c_int)                      :: baseFontShadow    !! shadow font
    integer(c_int)                      :: baseFontBig       !! big font

    !! displaylist ranges
    integer(c_int)                      :: rangePlane                 !! all planes from model
    integer(c_int)                      :: rangeMesh                  !! all meshes from model
    integer(c_int)                      :: rangeHField                !! all hfields from model
    integer(c_int)                      :: rangeBuiltin               !! all builtin geoms with quality from model
    integer(c_int)                      :: rangeFont                  !! all characters in font

    !! skin VBOs
    integer(c_int)                      :: nskin                      !! number of skins
    type(c_ptr)                         :: skinvertVBO      !! skin vertex position VBOs  //nskin floats - my addition
    type(c_ptr)                         :: skinnormalVBO    !! skin vertex normal VBOs
    type(c_ptr)                         :: skintexcoordVBO  !! skin vertex texture coordinate VBOs
    type(c_ptr)                         :: skinfaceVBO      !! skin face index VBOs

    !! character info
    integer(c_int)                      :: charWidth(127)             !! character widths: normal and shadow
    integer(c_int)                      :: charWidthBig(127)          !! chacarter widths: big
    integer(c_int)                      :: charHeight                 !! character heights: normal and shadow
    integer(c_int)                      :: charHeightBig              !! character heights: big

    !! capabilities
    integer(c_int)                      :: glewInitialized            !! is glew initialized
    integer(c_int)                      :: windowAvailable            !! is default/window framebuffer available
    integer(c_int)                      :: windowSamples              !! number of samples for default/window framebuffer
    integer(c_int)                      :: windowStereo               !! is stereo available for default/window framebuffer
    integer(c_int)                      :: windowDoublebuffer         !! is default/window framebuffer double buffered

    !! framebuffer
    integer(c_int)                      :: currentBuffer          !! currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN
  end type mjrContext

end module mod_mjrender