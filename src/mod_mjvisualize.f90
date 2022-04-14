module mod_mjvisualize
  use iso_c_binding
  use mod_mjtnum
  use mod_mjmodel
  implicit none

  integer(c_int), parameter             :: mjNGROUP       = 6         !! number of geom site joint groups with visflags
  integer(c_int), parameter             :: mjMAXLIGHT     = 100       !! maximum number of lights in a scene
  integer(c_int), parameter             :: mjMAXOVERLAY   = 500       !! maximum number of characters in overlay text
  integer(c_int), parameter             :: mjMAXLINE      = 100       !! maximum number of lines per plot
  integer(c_int), parameter             :: mjMAXLINEPNT   = 1000      !! maximum number points per line
  integer(c_int), parameter             :: mjMAXPLANEGRID = 200       !! maximum number of grid divisions for plane

  !---------------------------------- primitive types (mjt) -----------------------------------------

  enum, bind(c) !! mjtCatBit         !! bitflags for mjvGeom category
    enumerator :: mjCAT_STATIC        = 1        !! model elements in body 0
    enumerator :: mjCAT_DYNAMIC       = 2        !! model elements in all other bodies
    enumerator :: mjCAT_DECOR         = 4        !! decorative geoms
    enumerator :: mjCAT_ALL           = 7         !! select all categories
  end enum  !! mjtCatBit


  enum, bind(c) !! mjtMouse          !! mouse interaction mode
    enumerator :: mjMOUSE_NONE        = 0        !! no action
    enumerator :: mjMOUSE_ROTATE_V               !! rotate vertical plane
    enumerator :: mjMOUSE_ROTATE_H               !! rotate horizontal plane
    enumerator :: mjMOUSE_MOVE_V                 !! move vertical plane
    enumerator :: mjMOUSE_MOVE_H                 !! move horizontal plane
    enumerator :: mjMOUSE_ZOOM                   !! zoom
    enumerator :: mjMOUSE_SELECT                  !! selection
  end enum  !! mjtMouse


  enum, bind(c) !! mjtPertBit        !! mouse perturbations
    enumerator :: mjPERT_TRANSLATE    = 1        !! translation
    enumerator :: mjPERT_ROTATE       = 2         !! rotation
  end enum  !! mjtPertBit


  enum, bind(c) !! mjtCamera         !! abstract camera type
    enumerator :: mjCAMERA_FREE       = 0        !! free camera
    enumerator :: mjCAMERA_TRACKING              !! tracking camera uses trackbodyid
    enumerator :: mjCAMERA_FIXED                 !! fixed camera uses fixedcamid
    enumerator :: mjCAMERA_USER                   !! user is responsible for setting OpenGL camera
  end enum  !! mjtCamera


  enum, bind(c) !! mjtLabel          !! object labeling
    enumerator :: mjLABEL_NONE        = 0        !! nothing
    enumerator :: mjLABEL_BODY                   !! body labels
    enumerator :: mjLABEL_JOINT                  !! joint labels
    enumerator :: mjLABEL_GEOM                   !! geom labels
    enumerator :: mjLABEL_SITE                   !! site labels
    enumerator :: mjLABEL_CAMERA                 !! camera labels
    enumerator :: mjLABEL_LIGHT                  !! light labels
    enumerator :: mjLABEL_TENDON                 !! tendon labels
    enumerator :: mjLABEL_ACTUATOR               !! actuator labels
    enumerator :: mjLABEL_CONSTRAINT             !! constraint labels
    enumerator :: mjLABEL_SKIN                   !! skin labels
    enumerator :: mjLABEL_SELECTION              !! selected object
    enumerator :: mjLABEL_SELPNT                 !! coordinates of selection point
    enumerator :: mjLABEL_CONTACTFORCE           !! magnitude of contact force

    enumerator :: mjNLABEL                        !! number of label types
  end enum  !! mjtLabel


  enum, bind(c) !! mjtFrame          !! frame visualization
    enumerator :: mjFRAME_NONE        = 0        !! no frames
    enumerator :: mjFRAME_BODY                   !! body frames
    enumerator :: mjFRAME_GEOM                   !! geom frames
    enumerator :: mjFRAME_SITE                   !! site frames
    enumerator :: mjFRAME_CAMERA                 !! camera frames
    enumerator :: mjFRAME_LIGHT                  !! light frames
    enumerator :: mjFRAME_WORLD                  !! world frame

    enumerator :: mjNFRAME                        !! number of visualization frames
  end enum  !! mjtFrame


  enum, bind(c) !! mjtVisFlag        !! flags enabling model element visualization
    enumerator :: mjVIS_CONVEXHULL    = 0        !! mesh convex hull
    enumerator :: mjVIS_TEXTURE                  !! textures
    enumerator :: mjVIS_JOINT                    !! joints
    enumerator :: mjVIS_ACTUATOR                 !! actuators
    enumerator :: mjVIS_CAMERA                   !! cameras
    enumerator :: mjVIS_LIGHT                    !! lights
    enumerator :: mjVIS_TENDON                   !! tendons
    enumerator :: mjVIS_RANGEFINDER              !! rangefinder sensors
    enumerator :: mjVIS_CONSTRAINT               !! point constraints
    enumerator :: mjVIS_INERTIA                  !! equivalent inertia boxes
    enumerator :: mjVIS_SCLINERTIA               !! scale equivalent inertia boxes with mass
    enumerator :: mjVIS_PERTFORCE                !! perturbation force
    enumerator :: mjVIS_PERTOBJ                  !! perturbation object
    enumerator :: mjVIS_CONTACTPOINT             !! contact points
    enumerator :: mjVIS_CONTACTFORCE             !! contact force
    enumerator :: mjVIS_CONTACTSPLIT             !! split contact force into normal and tanget
    enumerator :: mjVIS_TRANSPARENT              !! make dynamic geoms more transparent
    enumerator :: mjVIS_AUTOCONNECT              !! auto connect joints and body coms
    enumerator :: mjVIS_COM                      !! center of mass
    enumerator :: mjVIS_SELECT                   !! selection point
    enumerator :: mjVIS_STATIC                   !! static bodies
    enumerator :: mjVIS_SKIN                     !! skin

    enumerator :: mjNVISFLAG                      !! number of visualization flags
  end enum  !! mjtVisFlag


  enum, bind(c) !! mjtRndFlag        !! flags enabling rendering effects
    enumerator :: mjRND_SHADOW        = 0        !! shadows
    enumerator :: mjRND_WIREFRAME                !! wireframe
    enumerator :: mjRND_REFLECTION               !! reflections
    enumerator :: mjRND_ADDITIVE                 !! additive transparency
    enumerator :: mjRND_SKYBOX                   !! skybox
    enumerator :: mjRND_FOG                      !! fog
    enumerator :: mjRND_HAZE                     !! haze
    enumerator :: mjRND_SEGMENT                  !! segmentation with random color
    enumerator :: mjRND_IDCOLOR                  !! segmentation with segid color

    enumerator :: mjNRNDFLAG                      !! number of rendering flags
  end enum  !! mjtRndFlag


  enum, bind(c) !! mjtStereo         !! type of stereo rendering
    enumerator :: mjSTEREO_NONE       = 0        !! no stereo use left eye only
    enumerator :: mjSTEREO_QUADBUFFERED          !! quad buffered revert to side-by-side if no hardware support
    enumerator :: mjSTEREO_SIDEBYSIDE             !! side-by-side
  end enum  !! mjtStereo


  !---------------------------------- mjvPerturb ----------------------------------------------------

  type, bind(c) :: mjvPerturb              !! object selection and perturbation
    integer(c_int)                  :: select                !! selected body id non-positive: none
    integer(c_int)                  :: skinselect            !! selected skin id negative: none
    integer(c_int)                  :: active                !! perturbation bitmask (mjtPertBit)
    integer(c_int)                  :: active2               !! secondary perturbation bitmask (mjtPertBit)
    real(mjtNum)                    :: refpos(3)             !! desired position for selected object
    real(mjtNum)                    :: refquat(4)            !! desired orientation for selected object
    real(mjtNum)                    :: localpos(3)           !! selection point in object coordinates
    real(mjtNum)                    :: scale                 !! relative mouse motion-to-space scaling (set by initPerturb)
  end type mjvPerturb


  !---------------------------------- mjvCamera -----------------------------------------------------

  type, bind(c) :: mjvCamera               !! abstract camera
    !! type and ids
    integer(c_int)                  :: type                  !! camera type (mjtCamera)
    integer(c_int)                  :: fixedcamid            !! fixed camera id
    integer(c_int)                  :: trackbodyid           !! body id to track

    !! abstract camera pose specification
    real(mjtNum)                    :: lookat(3)             !! lookat point
    real(mjtNum)                    :: distance              !! distance to lookat point or tracked body
    real(mjtNum)                    :: azimuth               !! camera azimuth (deg)
    real(mjtNum)                    :: elevation             !! camera elevation (deg)
  end type mjvCamera


  !---------------------------------- mjvGLCamera ---------------------------------------------------

  type, bind(c) :: mjvGLCamera             !! OpenGL camera
    !! camera frame
    real(c_float)                   :: pos(3)                !! position
    real(c_float)                   :: forward(3)            !! forward direction
    real(c_float)                   :: up(3)                 !! up direction

    !! camera projection
    real(c_float)                   :: frustum_center        !! hor. center (leftright set to match aspect)
    real(c_float)                   :: frustum_bottom        !! bottom
    real(c_float)                   :: frustum_top           !! top
    real(c_float)                   :: frustum_near          !! near
    real(c_float)                   :: frustum_far           !! far
  end type mjvGLCamera


  !---------------------------------- mjvGeom -------------------------------------------------------

  type, bind(c) :: mjvGeom                 !! abstract geom
    !! type info
    integer(c_int)                  :: type                  !! geom type (mjtGeom)
    integer(c_int)                  :: dataid                !! mesh hfield or plane id -1: none
    integer(c_int)                  :: objtype               !! mujoco object type mjOBJ_UNKNOWN for decor
    integer(c_int)                  :: objid                 !! mujoco object id -1 for decor
    integer(c_int)                  :: category              !! visual category
    integer(c_int)                  :: texid                 !! texture id -1: no texture
    integer(c_int)                  :: texuniform            !! uniform cube mapping
    integer(c_int)                  :: texcoord              !! mesh geom has texture coordinates
    integer(c_int)                  :: segid                 !! segmentation id -1: not shown

    !! OpenGL info
    real(c_float)                   :: texrepeat(2)          !! texture repetition for 2D mapping
    real(c_float)                   :: size(3)               !! size parameters
    real(c_float)                   :: pos(3)                !! Cartesian position
    real(c_float)                   :: mat(9)                !! Cartesian orientation
    real(c_float)                   :: rgba(4)               !! color and transparency
    real(c_float)                   :: emission              !! emission coef
    real(c_float)                   :: specular              !! specular coef
    real(c_float)                   :: shininess             !! shininess coef
    real(c_float)                   :: reflectance           !! reflectance coef
    character(len=1, kind=c_char), dimension(100) :: label !! text label

    !! transparency rendering (set internally)
    real(c_float)                   :: camdist               !! distance to camera (used by sorter)
    real(c_float)                   :: modelrbound           !! geom rbound from model 0 if not model geom
    integer(mjtByte)                ::  transparent           !! treat geom as transparent
  end type mjvGeom


  !---------------------------------- mjvLight ------------------------------------------------------

  type, bind(c) :: mjvLight                !! OpenGL light
    real(c_float)                   :: pos(3)                !! position rel. to body frame
    real(c_float)                   :: dir(3)                !! direction rel. to body frame
    real(c_float)                   :: attenuation(3)        !! OpenGL attenuation (quadratic model)
    real(c_float)                   :: cutoff                !! OpenGL cutoff
    real(c_float)                   :: exponent              !! OpenGL exponent
    real(c_float)                   :: ambient(3)            !! ambient rgb (alpha=1)
    real(c_float)                   :: diffuse(3)            !! diffuse rgb (alpha=1)
    real(c_float)                   :: specular(3)           !! specular rgb (alpha=1)
    integer(mjtByte)                :: headlight             !! headlight
    integer(mjtByte)                :: directional           !! directional light
    integer(mjtByte)                :: castshadow            !! does light cast shadows
  end type mjvLight


  !---------------------------------- mjvOption -----------------------------------------------------

  type, bind(c) :: mjvOption               !! abstract visualization options
    integer(c_int)                  :: label                 !! what objects to label (mjtLabel)
    integer(c_int)                  :: frame                 !! which frame to show (mjtFrame)
    integer(mjtByte)                :: geomgroup(mjNGROUP)   !! geom visualization by group
    integer(mjtByte)                :: sitegroup(mjNGROUP)   !! site visualization by group
    integer(mjtByte)                :: jointgroup(mjNGROUP)  !! joint visualization by group
    integer(mjtByte)                :: tendongroup(mjNGROUP) !! tendon visualization by group
    integer(mjtByte)                :: actuatorgroup(mjNGROUP)  !! actuator visualization by group
    integer(mjtByte)                :: flags(mjNVISFLAG)     !! visualization flags (indexed by mjtVisFlag)
  end type mjvOption


  !---------------------------------- mjvScene ------------------------------------------------------

  type, bind(c) :: mjvScene                !! abstract scene passed to OpenGL renderer
    !! abstract geoms
    integer(c_int)                  :: maxgeom               !! size of allocated geom buffer
    integer(c_int)                  :: ngeom                 !! number of geoms currently in buffer
    type(c_ptr)                     :: geoms                 !! buffer for geoms
    type(c_ptr)                     :: geomorder             !! buffer for ordering geoms by distance to camera

    !! skin data
    integer(c_int)                  :: nskin                 !! number of skins
    type(c_ptr)                     :: skinfacenum           !! number of faces in skin
    type(c_ptr)                     :: skinvertadr           !! address of skin vertices
    type(c_ptr)                     :: skinvertnum           !! number of vertices in skin
    type(c_ptr)                     :: skinvert              !! skin vertex data
    type(c_ptr)                     :: skinnormal            !! skin normal data

    !! OpenGL lights
    integer(c_int)                  :: nlight                !! number of lights currently in buffer
    type(mjvLight)                  :: lights(mjMAXLIGHT)    !! buffer for lights

    !! OpenGL cameras
    type(mjvGLCamera)               :: camera(2)          !! left and right camera

    !! OpenGL model transformation
    integer(mjtByte)                :: enabletransform       !! enable model transformation
    real(c_float)                   :: translate(3)          !! model translation
    real(c_float)                   :: rotate(4)             !! model quaternion rotation
    real(c_float)                   :: scale                 !! model scaling

    !! OpenGL rendering effects
    integer(c_int)                  :: stereo                !! stereoscopic rendering (mjtStereo)
    integer(mjtByte)                :: flags(mjNRNDFLAG)     !! rendering flags (indexed by mjtRndFlag)

    !! framing
    integer(c_int)                  :: framewidth            !! frame pixel width 0: disable framing
    real(c_float)                   :: framergb(3)           !! frame color
  end type mjvScene


  !---------------------------------- mjvFigure -----------------------------------------------------

  type, bind(c) :: mjvFigure               !! abstract 2D figure passed to OpenGL renderer
    !! enable flags
    integer(c_int)                  :: flg_legend             !! show legend
    integer(c_int)                  :: flg_ticklabel(2)       !! show grid tick labels (xy)
    integer(c_int)                  :: flg_extend             !! automatically extend axis ranges to fit data
    integer(c_int)                  :: flg_barplot            !! isolated line segments (i.e. GL_LINES)
    integer(c_int)                  :: flg_selection          !! vertical selection line
    integer(c_int)                  :: flg_symmetric          !! symmetric y-axis

    !! style settings
    real(c_float)                   :: linewidth              !! line width
    real(c_float)                   :: gridwidth              !! grid line width
    integer(c_int)                  :: gridsize(2)            !! number of grid points in (xy)
    real(c_float)                   :: gridrgb(3)             !! grid line rgb
    real(c_float)                   :: figurergba(4)          !! figure color and alpha
    real(c_float)                   :: panergba(4)            !! pane color and alpha
    real(c_float)                   :: legendrgba(4)          !! legend color and alpha
    real(c_float)                   :: textrgb(3)             !! text color
    real(c_float)                   :: linergb(3, mjMAXLINE)  !! line colors
    real(c_float)                   :: range(2, 2)            !! axis ranges (min>=max) automatic
    character(c_char)               :: xformat(20)            !! x-tick label format for sprintf
    character(c_char)               :: yformat(20)            !! y-tick label format for sprintf
    character(c_char)               :: minwidth(20)           !! string used to determine min y-tick width

    !! text labels
    character(len=1, kind=c_char), dimension(1000) :: title   !! figure title subplots separated with 2+ spaces
    character(len=1, kind=c_char), dimension(100) :: xlabel   !! x-axis label
    character(len=1, kind=c_char), dimension(100, mjMAXLINE) :: linename !! line names for legend

    !! dynamic settings
    integer(c_int)                  :: legendoffset           !! number of lines to offset legend
    integer(c_int)                  :: subplot                !! selected subplot (for title rendering)
    integer(c_int)                  :: highlight(2)           !! if point is in legend rect highlight line
    integer(c_int)                  :: highlightid            !! if id>=0 and no point highlight id
    real(c_float)                   :: selection              !! selection line x-value

    !! line data
    integer(c_int)                  :: linepnt(mjMAXLINE)     !! number of points in line (0) disable
    real(c_float)                   :: linedata(2*mjMAXLINEPNT, mjMAXLINE) !! line data (xy)

    !! output from renderer
    integer(c_int)                  :: xaxispixel(2)          !! range of x-axis in pixels
    integer(c_int)                  :: yaxispixel(2)          !! range of y-axis in pixels
    real(c_float)                   :: xaxisdata(2)           !! range of x-axis in data units
    real(c_float)                   :: yaxisdata(2)           !! range of y-axis in data units
  end type mjvFigure
  
end module mod_mjvisualize