module mod_mjmodel
  use iso_c_binding
  use mod_mjtnum
  implicit none
  
  ! global constants
  real(c_double), parameter, public :: mjPI         = 3.14159265358979323846
  real(c_double), parameter, public :: mjMAXVAL     = 1E+10         !! maximum value in qpos, qvel, qacc
  real(c_double), parameter, public :: mjMINMU      = 1E-5          !! minimum friction coefficient
  real(c_double), parameter, public :: mjMINIMP     = 0.0001        !! minimum constraint impedance
  real(c_double), parameter, public :: mjMAXIMP     = 0.9999        !! maximum constraint impedance
  integer(c_int), parameter, public :: mjMAXCONPAIR = 50            !! maximum number of contacts per geom pair
  integer(c_int), parameter, public :: mjMAXVFS     = 2000          !! maximum number of files in virtual file system
  integer(c_int), parameter, public :: mjMAXVFSNAME = 1000          !! maximum filename size in virtual file system


  !---------------------------------- sizes ---------------------------------------------------------

  integer(c_int), parameter, public :: mjNEQDATA    = 7             !! number of eq_data fields
  integer(c_int), parameter, public :: mjNDYN       = 10            !! number of actuator dynamics parameters
  integer(c_int), parameter, public :: mjNGAIN      = 10            !! number of actuator gain parameters
  integer(c_int), parameter, public :: mjNBIAS      = 10            !! number of actuator bias parameters
  integer(c_int), parameter, public :: mjNFLUID     = 12            !! number of fluid interaction parameters
  integer(c_int), parameter, public :: mjNREF       = 2             !! number of solver reference parameters
  integer(c_int), parameter, public :: mjNIMP       = 5             !! number of solver impedance parameters
  integer(c_int), parameter, public :: mjNSOLVER    = 1000          !! size of mjData.solver_XXX arrays


  !---------------------------------- primitive types (mjt) -----------------------------------------

  integer, parameter                    :: mjtByte = c_signed_char  !! used for true/false

  enum, bind(c) !! mjtDisableBit_                                   !! disable default feature bitflags
    enumerator :: mjDSBL_CONSTRAINT   = 1    !lshift(1, 0)                !! entire constraint solver
    enumerator :: mjDSBL_EQUALITY     = 2    !lshift(1, 1)                !! equality constraints
    enumerator :: mjDSBL_FRICTIONLOSS = 4    !lshift(1, 2)                !! joint and tendon frictionloss constraints
    enumerator :: mjDSBL_LIMIT        = 8    !lshift(1, 3)                !! joint and tendon limit constraints
    enumerator :: mjDSBL_CONTACT      = 16   !lshift(1, 4)                !! contact constraints
    enumerator :: mjDSBL_PASSIVE      = 32   !lshift(1, 5)                !! passive forces
    enumerator :: mjDSBL_GRAVITY      = 64   !lshift(1, 6)                !! gravitational forces
    enumerator :: mjDSBL_CLAMPCTRL    = 128  !lshift(1, 7)                !! clamp control to specified range
    enumerator :: mjDSBL_WARMSTART    = 256  !lshift(1, 8)                !! warmstart constraint solver
    enumerator :: mjDSBL_FILTERPARENT = 512  !lshift(1, 9)                !! remove collisions with parent body
    enumerator :: mjDSBL_ACTUATION    = 1024 !lshift(1, 10)               !! apply actuation forces
    enumerator :: mjDSBL_REFSAFE      = 2048 !lshift(1, 11)               !! integrator safety: make ref[0]>=2*timestep

    enumerator :: mjNDISABLE          = 12                          !! number of disable flags
  end enum

  enum, bind(c) !!mjtEnableBit_ {      !! enable optional feature bitflags
    enumerator :: mjENBL_OVERRIDE     = 1 !lshift(1, 0)                !! override contact parameters
    enumerator :: mjENBL_ENERGY       = 2 !lshift(1, 1)                !! energy computation
    enumerator :: mjENBL_FWDINV       = 4 !lshift(1, 2)                !! record solver statistics
    enumerator :: mjENBL_SENSORNOISE  = 8 !lshift(1, 3)                !! add noise to sensor data
    enumerator :: mjENBL_MULTICCD     = 16                             !! multi-point convex collison detection 
    enumerator :: mjNENABLE           = 5                              !! number of enable flags
  end enum

  enum, bind(c) !! mjtJoint_ {          !! type of degree of freedom
    enumerator :: mjJNT_FREE          = 0                           !! global position and orientation (quat)       (7)
    enumerator :: mjJNT_BALL                                        !! orientation (quat) relative to parent        (4)
    enumerator :: mjJNT_SLIDE                                       !! sliding distance along body-fixed axis       (1)
    enumerator :: mjJNT_HINGE                                       !! rotation angle (rad) around body-fixed axis  (1)
  end enum

  enum, bind(c) !!mjtGeom_ {           !! type of geometric shape
    !! regular geom types
    enumerator :: mjGEOM_PLANE        = 0                           !! plane
    enumerator :: mjGEOM_HFIELD                                     !! height field
    enumerator :: mjGEOM_SPHERE                                     !! sphere
    enumerator :: mjGEOM_CAPSULE                                    !! capsule
    enumerator :: mjGEOM_ELLIPSOID                                  !! ellipsoid
    enumerator :: mjGEOM_CYLINDER                                   !! cylinder
    enumerator :: mjGEOM_BOX                                        !! box
    enumerator :: mjGEOM_MESH                                       !! mesh

    enumerator :: mjNGEOMTYPES                                      !! number of regular geom types

    !! rendering-only geom types: not used in mjModel, not counted in mjNGEOMTYPES
    enumerator :: mjGEOM_ARROW        = 100                         !! arrow
    enumerator :: mjGEOM_ARROW1                                     !! arrow without wedges
    enumerator :: mjGEOM_ARROW2                                     !! arrow in both directions
    enumerator :: mjGEOM_LINE                                       !! line
    enumerator :: mjGEOM_SKIN                                       !! skin
    enumerator :: mjGEOM_LABEL                                      !! text label
    enumerator :: mjGEOM_NONE         = 1001                        !! missing geom type
  end enum

  enum, bind(c) !! mjtCamLight_ {       !! tracking mode for camera and light
    enumerator :: mjCAMLIGHT_FIXED    = 0                           !! pos and rot fixed in body
    enumerator :: mjCAMLIGHT_TRACK                                  !! pos tracks body, rot fixed in global
    enumerator :: mjCAMLIGHT_TRACKCOM                               !! pos tracks subtree com, rot fixed in body
    enumerator :: mjCAMLIGHT_TARGETBODY                             !! pos fixed in body, rot tracks target body
    enumerator :: mjCAMLIGHT_TARGETBODYCOM                          !! pos fixed in body, rot tracks target subtree com
  end enum

  enum, bind(c) ! mjtTexture_ {        !! type of texture
    enumerator :: mjTEXTURE_2D        = 0                           !! 2d texture, suitable for planes and hfields
    enumerator :: mjTEXTURE_CUBE                                    !! cube texture, suitable for all other geom types
    enumerator :: mjTEXTURE_SKYBOX                                  !! cube texture used as skybox
  end enum

  enum, bind(c) ! mjtIntegrator_ {     !! integrator mode
    enumerator :: mjINT_EULER         = 0                           !! semi-implicit Euler
    enumerator :: mjINT_RK4                                         !! 4th-order Runge Kutta
    enumerator :: mjINT_IMPLICIT                                    !! implicit in velocity
  end enum

  enum, bind(c) ! mjtCollision_ {      !! collision mode for selecting geom pairs
    enumerator :: mjCOL_ALL           = 0                           !! test precomputed and dynamic pairs
    enumerator :: mjCOL_PAIR                                        !! test predefined pairs only
    enumerator :: mjCOL_DYNAMIC                                     !! test dynamic pairs only
  end enum

  enum, bind(c) ! mjtCone_ {           !! type of friction cone
    enumerator :: mjCONE_PYRAMIDAL     = 0                          !! pyramidal
    enumerator :: mjCONE_ELLIPTIC                                   !! elliptic
  end enum

  enum, bind(c) !! mjtJacobian_ {       !! type of constraint Jacobian
    enumerator :: mjJAC_DENSE          = 0                          !! dense
    enumerator :: mjJAC_SPARSE                                      !! sparse
    enumerator :: mjJAC_AUTO                                        !! dense if nv<60, sparse otherwise
  end enum


  enum, bind(c) !! mjtSolver_ {         !! constraint solver algorithm
    enumerator :: mjSOL_PGS            = 0                          !! PGS    (dual)
    enumerator :: mjSOL_CG                                          !! CG     (primal)
    enumerator :: mjSOL_NEWTON                                      !! Newton (primal)
  end enum !! mjtSolver


  enum, bind(c) !! mjtEq_ {             !! type of equality constraint
    enumerator :: mjEQ_CONNECT        = 0                           !! connect two bodies at a point (ball joint)
    enumerator :: mjEQ_WELD                                         !! fix relative position and orientation of two bodies
    enumerator :: mjEQ_JOINT                                        !! couple the values of two scalar joints with cubic
    enumerator :: mjEQ_TENDON                                       !! couple the lengths of two tendons with cubic
    enumerator :: mjEQ_DISTANCE                                     !! fix the contact distance betweent two geoms
  end enum !! enumerator :: mjtEq


  enum, bind(c) !! enumerator :: mjtWrap_ {           !! type of tendon wrap object
    enumerator :: mjWRAP_NONE         = 0                           !! null object
    enumerator :: mjWRAP_JOINT                                      !! constant moment arm
    enumerator :: mjWRAP_PULLEY                                     !! pulley used to split tendon
    enumerator :: mjWRAP_SITE                                       !! pass through site
    enumerator :: mjWRAP_SPHERE                                     !! wrap around sphere
    enumerator :: mjWRAP_CYLINDER                                   !! wrap around (infinite) cylinder
  end enum !! enumerator :: mjtWrap


  enum, bind(c) !! enumerator :: mjtTrn_ {            !! type of actuator transmission
    enumerator :: mjTRN_JOINT         = 0                           !! force on joint
    enumerator :: mjTRN_JOINTINPARENT                               !! force on joint, expressed in parent frame
    enumerator :: mjTRN_SLIDERCRANK                                 !! force via slider-crank linkage
    enumerator :: mjTRN_TENDON                                      !! force on tendon
    enumerator :: mjTRN_SITE                                        !! force on site

    enumerator :: mjTRN_UNDEFINED     = 1000                        !! undefined transmission type
  end enum !! enumerator :: mjtTrn


  enum, bind(c) !! enumerator :: mjtDyn_ {            !! type of actuator dynamics
    enumerator :: mjDYN_NONE          = 0                           !! no internal dynamics ctrl specifies force
    enumerator :: mjDYN_INTEGRATOR                                  !! integrator: da/dt = u
    enumerator :: mjDYN_FILTER                                      !! linear filter: da/dt = (u-a) / tau
    enumerator :: mjDYN_MUSCLE                                      !! piece-wise linear filter with two time constants
    enumerator :: mjDYN_USER                                        !! user-defined dynamics type
  end enum !! enumerator :: mjtDyn


  enum, bind(c) !! enumerator :: mjtGain_ {           !! type of actuator gain
    enumerator :: mjGAIN_FIXED        = 0                           !! fixed gain
    enumerator :: mjGAIN_MUSCLE                                     !! muscle FLV curve computed by enumerator :: mju_muscleGain()
    enumerator :: mjGAIN_USER                                       !! user-defined gain type
  end enum !! enumerator :: mjtGain


  enum, bind(c) !! enumerator :: mjtBias_ {           !! type of actuator bias
    enumerator :: mjBIAS_NONE         = 0                           !! no bias
    enumerator :: mjBIAS_AFFINE                                     !! const + kp*length + kv*velocity
    enumerator :: mjBIAS_MUSCLE                                     !! muscle passive force computed by enumerator :: mju_muscleBias()
    enumerator :: mjBIAS_USER                                       !! user-defined bias type
  end enum !! enumerator :: mjtBias


  enum, bind(c) !! enumerator :: mjtObj_ {            !! type of MujoCo object
    enumerator :: mjOBJ_UNKNOWN       = 0                           !! unknown object type
    enumerator :: mjOBJ_BODY                                        !! body
    enumerator :: mjOBJ_XBODY                                       !! body  used to access regular frame instead of i-frame
    enumerator :: mjOBJ_JOINT                                       !! joint
    enumerator :: mjOBJ_DOF                                         !! dof
    enumerator :: mjOBJ_GEOM                                        !! geom
    enumerator :: mjOBJ_SITE                                        !! site
    enumerator :: mjOBJ_CAMERA                                      !! camera
    enumerator :: mjOBJ_LIGHT                                       !! light
    enumerator :: mjOBJ_MESH                                        !! mesh
    enumerator :: mjOBJ_SKIN                                        !! skin
    enumerator :: mjOBJ_HFIELD                                      !! heightfield
    enumerator :: mjOBJ_TEXTURE                                     !! texture
    enumerator :: mjOBJ_MATERIAL                                    !! material for rendering
    enumerator :: mjOBJ_PAIR                                        !! geom pair to include
    enumerator :: mjOBJ_EXCLUDE                                     !! body pair to exclude
    enumerator :: mjOBJ_EQUALITY                                    !! equality constraint
    enumerator :: mjOBJ_TENDON                                      !! tendon
    enumerator :: mjOBJ_ACTUATOR                                    !! actuator
    enumerator :: mjOBJ_SENSOR                                      !! sensor
    enumerator :: mjOBJ_NUMERIC                                     !! numeric
    enumerator :: mjOBJ_TEXT                                        !! text
    enumerator :: mjOBJ_TUPLE                                       !! tuple
    enumerator :: mjOBJ_KEY                                         !! keyframe
  end enum !! enumerator :: mjtObj


  enum, bind(c) !! enumerator :: mjtConstraint_ {     !! type of constraint
    enumerator :: mjCNSTR_EQUALITY    = 0                           !! equality constraint
    enumerator :: mjCNSTR_FRICTION_DOF                              !! dof friction
    enumerator :: mjCNSTR_FRICTION_TENDON                           !! tendon friction
    enumerator :: mjCNSTR_LIMIT_JOINT                               !! joint limit
    enumerator :: mjCNSTR_LIMIT_TENDON                              !! tendon limit
    enumerator :: mjCNSTR_CONTACT_FRICTIONLESS                      !! frictionless contact
    enumerator :: mjCNSTR_CONTACT_PYRAMIDAL                         !! frictional contact, pyramidal friction cone
    enumerator :: mjCNSTR_CONTACT_ELLIPTIC                          !! frictional contact, elliptic friction cone
  end enum !! enumerator :: mjtConstraint


  enum, bind(c) !! enumerator :: mjtConstraintState_ { !! constraint state
    enumerator :: mjCNSTRSTATE_SATISFIED = 0                        !! constraint satisfied, zero cost (limit, contact)
    enumerator :: mjCNSTRSTATE_QUADRATIC                            !! quadratic cost (equality, friction, limit, contact)
    enumerator :: mjCNSTRSTATE_LINEARNEG                            !! linear cost, negative side (friction)
    enumerator :: mjCNSTRSTATE_LINEARPOS                            !! linear cost, positive side (friction)
    enumerator :: mjCNSTRSTATE_CONE                                 !! squared distance to cone cost (elliptic contact)
  end enum !! enumerator :: mjtConstraintState


  enum, bind(c) !! enumerator :: mjtSensor_ {         !! type of sensor
    !! common robotic sensors, attached to a site
    enumerator :: mjSENS_TOUCH        = 0                           !! scalar contact normal forces summed over sensor zone
    enumerator :: mjSENS_ACCELEROMETER                              !! 3D linear acceleration, in local frame
    enumerator :: mjSENS_VELOCIMETER                                !! 3D linear velocity, in local frame
    enumerator :: mjSENS_GYRO                                       !! 3D angular velocity, in local frame
    enumerator :: mjSENS_FORCE                                      !! 3D force between site's body and its parent body
    enumerator :: mjSENS_TORQUE                                     !! 3D torque between site's body and its parent body
    enumerator :: mjSENS_MAGNETOMETER                               !! 3D magnetometer
    enumerator :: mjSENS_RANGEFINDER                                !! scalar distance to nearest geom or site along z-axis

    !! sensors related to scalar joints, tendons, actuators
    enumerator :: mjSENS_JOINTPOS                                   !! scalar joint position (hinge and slide only)
    enumerator :: mjSENS_JOINTVEL                                   !! scalar joint velocity (hinge and slide only)
    enumerator :: mjSENS_TENDONPOS                                  !! scalar tendon position
    enumerator :: mjSENS_TENDONVEL                                  !! scalar tendon velocity
    enumerator :: mjSENS_ACTUATORPOS                                !! scalar actuator position
    enumerator :: mjSENS_ACTUATORVEL                                !! scalar actuator velocity
    enumerator :: mjSENS_ACTUATORFRC                                !! scalar actuator force

    !! sensors related to ball joints
    enumerator :: mjSENS_BALLQUAT                                   !! 4D ball joint quaternion
    enumerator :: mjSENS_BALLANGVEL                                 !! 3D ball joint angular velocity

    !! joint and tendon limit sensors, in constraint space
    enumerator :: mjSENS_JOINTLIMITVEL                              !! joint limit velocity
    enumerator :: mjSENS_JOINTLIMITPOS                              !! joint limit distance-margin
    enumerator :: mjSENS_JOINTLIMITFRC                              !! joint limit force
    enumerator :: mjSENS_TENDONLIMITPOS                             !! tendon limit distance-margin
    enumerator :: mjSENS_TENDONLIMITVEL                             !! tendon limit velocity
    enumerator :: mjSENS_TENDONLIMITFRC                             !! tendon limit force

    !! sensors attached to an object with spatial frame: (x)body, geom, site, camera
    enumerator :: mjSENS_FRAMEPOS                                   !! 3D position
    enumerator :: mjSENS_FRAMEQUAT                                  !! 4D unit quaternion orientation
    enumerator :: mjSENS_FRAMEXAXIS                                 !! 3D unit vector: x-axis of object's frame
    enumerator :: mjSENS_FRAMEYAXIS                                 !! 3D unit vector: y-axis of object's frame
    enumerator :: mjSENS_FRAMEZAXIS                                 !! 3D unit vector: z-axis of object's frame
    enumerator :: mjSENS_FRAMELINVEL                                !! 3D linear velocity
    enumerator :: mjSENS_FRAMEANGVEL                                !! 3D angular velocity
    enumerator :: mjSENS_FRAMELINACC                                !! 3D linear acceleration
    enumerator :: mjSENS_FRAMEANGACC                                !! 3D angular acceleration

    !! sensors related to kinematic subtrees attached to a body (which is the subtree root)
    enumerator :: mjSENS_SUBTREECOM                                 !! 3D center of mass of subtree
    enumerator :: mjSENS_SUBTREELINVEL                              !! 3D linear velocity of subtree
    enumerator :: mjSENS_SUBTREEANGMOM                              !! 3D angular momentum of subtree

    !! user-defined sensor
    enumerator :: mjSENS_USER                                       !! sensor data provided by enumerator :: mjcb_sensor callback
  end enum !! enumerator :: mjtSensor


  enum, bind(c) !! enumerator :: mjtStage_ {          !! computation stage
    enumerator :: mjSTAGE_NONE        = 0                           !! no computations
    enumerator :: mjSTAGE_POS                                       !! position-dependent computations
    enumerator :: mjSTAGE_VEL                                       !! velocity-dependent computations
    enumerator :: mjSTAGE_ACC                                       !! acceleration/force-dependent computations
  end enum !! enumerator :: mjtStage


  enum, bind(c) !! enumerator :: mjtDataType_ {       !! data type for sensors
    enumerator :: mjDATATYPE_REAL     = 0                           !! real values, no constraints
    enumerator :: mjDATATYPE_POSITIVE                               !! positive values 0 or negative: inactive
    enumerator :: mjDATATYPE_AXIS                                   !! 3D unit vector
    enumerator :: mjDATATYPE_QUATERNION                             !! unit quaternion
  end enum !! enumerator :: mjtDataType


  enum, bind(c) !! enumerator :: mjtLRMode_ {         !! mode for actuator length range computation
    enumerator :: mjLRMODE_NONE   = 0                               !! do not process any actuators
    enumerator :: mjLRMODE_MUSCLE                                   !! process muscle actuators
    enumerator :: mjLRMODE_MUSCLEUSER                               !! process muscle and user actuators
    enumerator :: mjLRMODE_ALL                                      !! process all actuators
  end enum


  !---------------------------------- mjLROpt -------------------------------------------------------

  type, bind(c) :: mjLROpt                   !! options for mj_setLengthRange()
    !! flags
    integer(c_int)    :: mode                    !! which actuators to process (mjtLRMode)
    integer(c_int)    :: useexisting             !! use existing length range if available
    integer(c_int)    :: uselimit                !! use joint and tendon limits if available

    !! algorithm parameters
    real(mjtNum)      :: accel                   !! target acceleration used to compute force
    real(mjtNum)      :: maxforce                !! maximum force 0: no limit
    real(mjtNum)      :: timeconst               !! time constant for velocity reduction min 0.01
    real(mjtNum)      :: timestep                !! simulation timestep 0: use mjOption.timestep
    real(mjtNum)      :: inttotal                !! total simulation time interval
    real(mjtNum)      :: inteval                 !! evaluation time interval (at the end)
    real(mjtNum)      :: tolrange                !! convergence tolerance (relative to range)
  end type mjLROpt


!---------------------------------- mjVFS ---------------------------------------------------------

  type, bind(c) :: mjVFS                          !! virtual file system for loading from memory
    integer(c_int)      :: nfile                  !! number of files present
    character(len=1, kind=c_char), dimension(mjMAXVFS, mjMAXVFSNAME) :: filename !! file name without path
    integer(c_int)      :: filesize(mjMAXVFS)     !! file size in bytes
    type(c_ptr)         :: filedata               !(mjMAXVFS)     !! buffer with file data
  end type mjVFS


!---------------------------------- mjOption ------------------------------------------------------

  type, bind(c) :: mjOption !! {                !! physics options
    !! timing parameters
    real(mjtNum)      :: timestep                 !! timestep
    real(mjtNum)      :: apirate                  !! update rate for remote API (Hz)

    !! solver parameters
    real(mjtNum)      :: impratio                 !! ratio of friction-to-normal contact impedance
    real(mjtNum)      :: tolerance                !! main solver tolerance
    real(mjtNum)      :: noslip_tolerance         !! noslip solver tolerance
    real(mjtNum)      :: mpr_tolerance            !! MPR solver tolerance

    !! physical constants
    real(mjtNum)      :: gravity(3)               !! gravitational acceleration
    real(mjtNum)      :: wind(3)                  !! wind (for lift, drag and viscosity)
    real(mjtNum)      :: magnetic(3)              !! global magnetic flux
    real(mjtNum)      :: density                  !! density of medium
    real(mjtNum)      :: viscosity                !! viscosity of medium

    !! override contact solver parameters (if enabled)
    real(mjtNum)      :: o_margin                 !! margin
    real(mjtNum)      :: o_solref(mjNREF)         !! solref
    real(mjtNum)      :: o_solimp(mjNIMP)         !! solimp

    !! discrete settings
    integer(c_int)    :: integrator               !! integration mode (mjtIntegrator)
    integer(c_int)    :: collision                !! collision mode (mjtCollision)
    integer(c_int)    :: cone                     !! type of friction cone (mjtCone)
    integer(c_int)    :: jacobian                 !! type of Jacobian (mjtJacobian)
    integer(c_int)    :: solver                   !! solver algorithm (mjtSolver)
    integer(c_int)    :: iterations               !! maximum number of main solver iterations
    integer(c_int)    :: noslip_iterations        !! maximum number of noslip solver iterations
    integer(c_int)    :: mpr_iterations           !! maximum number of MPR solver iterations
    integer(c_int)    :: disableflags             !! bit flags for disabling standard features
    integer(c_int)    :: enableflags              !! bit flags for enabling optional features
  end type mjOption

  
  !---------------------------------- mjVisual ------------------------------------------------------

  type, bind(c) :: global !!{                        !! global parameters
    real(c_float)   :: fovy                   !! y-field of view (deg) for free camera
    real(c_float)   :: ipd                    !! inter-pupilary distance for free camera
    real(c_float)   :: linewidth              !! line width for wireframe and ray rendering
    real(c_float)   :: glow                   !! glow coefficient for selected body
    integer(c_int)  :: offwidth                 !! width of offscreen buffer
    integer(c_int)  :: offheight                !! height of offscreen buffer
  end type global

  type, bind(c) :: quality                        !! rendering quality
    integer(c_int)    :: shadowsize             !! size of shadowmap texture
    integer(c_int)    :: offsamples             !! number of multisamples for offscreen rendering
    integer(c_int)    :: numslices              !! number of slices for builtin geom drawing
    integer(c_int)    :: numstacks              !! number of stacks for builtin geom drawing
    integer(c_int)    :: numquads               !! number of quads for box rendering
  end type quality

  type, bind(c) :: headlight                        !! head light
    real(c_float)     :: ambient(3)             !! ambient rgb (alpha=1)
    real(c_float)     :: diffuse(3)             !! diffuse rgb (alpha=1)
    real(c_float)     :: specular(3)            !! specular rgb (alpha=1)
    integer(c_int)    :: active                 !! is headlight active
  end type headlight

  type, bind(c) :: map                        !! mapping
    real(c_float)     :: stiffness              !! mouse perturbation stiffness (space->force)
    real(c_float)     :: stiffnessrot           !! mouse perturbation stiffness (space->torque)
    real(c_float)     :: force                  !! from force units to space units
    real(c_float)     :: torque                 !! from torque units to space units
    real(c_float)     :: alpha                  !! scale geom alphas when transparency is enabled
    real(c_float)     :: fogstart               !! OpenGL fog starts at fogstart * mjModel.stat.extent
    real(c_float)     :: fogend                 !! OpenGL fog ends at fogend * mjModel.stat.extent
    real(c_float)     :: znear                  !! near clipping plane = znear * mjModel.stat.extent
    real(c_float)     :: zfar                   !! far clipping plane = zfar * mjModel.stat.extent
    real(c_float)     :: haze                   !! haze ratio
    real(c_float)     :: shadowclip             !! directional light: shadowclip * mjModel.stat.extent
    real(c_float)     :: shadowscale            !! spot light: shadowscale * light.cutoff
    real(c_float)     :: actuatortendon         !! scale tendon width
  end type map

  type, bind(c) :: scale                        !! scale of decor elements relative to mean body size
    real(c_float)     :: forcewidth             !! width of force arrow
    real(c_float)     :: contactwidth           !! contact width
    real(c_float)     :: contactheight          !! contact height
    real(c_float)     :: connect                !! autoconnect capsule width
    real(c_float)     :: com                    !! com radius
    real(c_float)     :: camera                 !! camera object
    real(c_float)     :: light                  !! light object
    real(c_float)     :: selectpoint            !! selection point
    real(c_float)     :: jointlength            !! joint length
    real(c_float)     :: jointwidth             !! joint width
    real(c_float)     :: actuatorlength         !! actuator length
    real(c_float)     :: actuatorwidth          !! actuator width
    real(c_float)     :: framelength            !! bodyframe axis length
    real(c_float)     :: framewidth             !! bodyframe axis width
    real(c_float)     :: constraint             !! constraint width
    real(c_float)     :: slidercrank            !! slidercrank width
  end type scale
  
  type, bind(c) :: rgba                       !! color of decor elements
    real(c_float)     :: fog(4)                 !! fog
    real(c_float)     :: haze(4)                !! haze
    real(c_float)     :: force(4)               !! external force
    real(c_float)     :: inertia(4)             !! inertia box
    real(c_float)     :: joint(4)               !! joint
    real(c_float)     :: actuator(4)            !! actuator, neutral
    real(c_float)     :: actuatornegative(4)    !! actuator, negative limit
    real(c_float)     :: actuatorpositive(4)    !! actuator, positive limit
    real(c_float)     :: com(4)                 !! center of mass
    real(c_float)     :: camera(4)              !! camera object
    real(c_float)     :: light(4)               !! light object
    real(c_float)     :: selectpoint(4)         !! selection point
    real(c_float)     :: connect(4)             !! auto connect
    real(c_float)     :: contactpoint(4)        !! contact point
    real(c_float)     :: contactforce(4)        !! contact force
    real(c_float)     :: contactfriction(4)     !! contact friction force
    real(c_float)     :: contacttorque(4)       !! contact torque
    real(c_float)     :: contactgap(4)          !! contact point in gap
    real(c_float)     :: rangefinder(4)         !! rangefinder ray
    real(c_float)     :: constraint(4)          !! constraint
    real(c_float)     :: slidercrank(4)         !! slidercrank
    real(c_float)     :: crankbroken(4)         !! used when crank must be stretched/broken
  end type rgba

  type, bind(c) :: mjVisual !! {                !! visualization options
    type(global)    :: global
    type(quality)   :: quality
    type(headlight) :: headlight
    type(map)       :: map
    type(scale)     :: scale
    type(rgba)      :: rgba
  end type mjVisual


  !---------------------------------- mjStatistic ---------------------------------------------------

  type, bind(c) :: mjStatistic             !! model statistics (in qpos0)
    real(mjtNum)    :: meaninertia             !! mean diagonal inertia
    real(mjtNum)    :: meanmass                !! mean body mass
    real(mjtNum)    :: meansize                !! mean body size
    real(mjtNum)    :: extent                  !! spatial extent
    real(mjtNum)    :: center(3)               !! center of model
  end type mjStatistic


  !---------------------------------- mjModel -------------------------------------------------------

  type, bind(c) :: mjModel
    !! ------------------------------- sizes

    !! sizes needed at mjModel construction
    integer(c_int)  :: nq                         !! number of generalized coordinates = dim(qpos)
    integer(c_int)  :: nv                         !! number of degrees of freedom = dim(qvel)
    integer(c_int)  :: nu                         !! number of actuators/controls = dim(ctrl)
    integer(c_int)  :: na                         !! number of activation states = dim(act)
    integer(c_int)  :: nbody                      !! number of bodies
    integer(c_int)  :: njnt                       !! number of joints
    integer(c_int)  :: ngeom                      !! number of geoms
    integer(c_int)  :: nsite                      !! number of sites
    integer(c_int)  :: ncam                       !! number of cameras
    integer(c_int)  :: nlight                     !! number of lights
    integer(c_int)  :: nmesh                      !! number of meshes
    integer(c_int)  :: nmeshvert                  !! number of vertices in all meshes
    integer(c_int)  :: nmeshtexvert               !! number of vertices with texcoords in all meshes
    integer(c_int)  :: nmeshface                  !! number of triangular faces in all meshes
    integer(c_int)  :: nmeshgraph                 !! number of ints in mesh auxiliary data
    integer(c_int)  :: nskin                      !! number of skins
    integer(c_int)  :: nskinvert                  !! number of vertices in all skins
    integer(c_int)  :: nskintexvert               !! number of vertiex with texcoords in all skins
    integer(c_int)  :: nskinface                  !! number of triangular faces in all skins
    integer(c_int)  :: nskinbone                  !! number of bones in all skins
    integer(c_int)  :: nskinbonevert              !! number of vertices in all skin bones
    integer(c_int)  :: nhfield                    !! number of heightfields
    integer(c_int)  :: nhfielddata                !! number of data points in all heightfields
    integer(c_int)  :: ntex                       !! number of textures
    integer(c_int)  :: ntexdata                   !! number of bytes in texture rgb data
    integer(c_int)  :: nmat                       !! number of materials
    integer(c_int)  :: npair                      !! number of predefined geom pairs
    integer(c_int)  :: nexclude                   !! number of excluded geom pairs
    integer(c_int)  :: neq                        !! number of equality constraints
    integer(c_int)  :: ntendon                    !! number of tendons
    integer(c_int)  :: nwrap                      !! number of wrap objects in all tendon paths
    integer(c_int)  :: nsensor                    !! number of sensors
    integer(c_int)  :: nnumeric                   !! number of numeric custom fields
    integer(c_int)  :: nnumericdata               !! number of mjtNums in all numeric fields
    integer(c_int)  :: ntext                      !! number of text custom fields
    integer(c_int)  :: ntextdata                  !! number of mjtBytes in all text fields
    integer(c_int)  :: ntuple                     !! number of tuple custom fields
    integer(c_int)  :: ntupledata                 !! number of objects in all tuple fields
    integer(c_int)  :: nkey                       !! number of keyframes
    integer(c_int)  :: nmocap                     !! number of mocap bodies
    integer(c_int)  :: nuser_body                 !! number of mjtNums in body_user
    integer(c_int)  :: nuser_jnt                  !! number of mjtNums in jnt_user
    integer(c_int)  :: nuser_geom                 !! number of mjtNums in geom_user
    integer(c_int)  :: nuser_site                 !! number of mjtNums in site_user
    integer(c_int)  :: nuser_cam                  !! number of mjtNums in cam_user
    integer(c_int)  :: nuser_tendon               !! number of mjtNums in tendon_user
    integer(c_int)  :: nuser_actuator             !! number of mjtNums in actuator_user
    integer(c_int)  :: nuser_sensor               !! number of mjtNums in sensor_user
    integer(c_int)  :: nnames                     !! number of chars in all names

    !! sizes set after mjModel construction (only affect mjData)
    integer(c_int)  :: nM                         !! number of non-zeros in sparse inertia matrix
    integer(c_int)  :: nD                         !! number of non-zeros in sparse derivative matrix
    integer(c_int)  :: nemax                      !! number of potential equality-constraint rows
    integer(c_int)  :: njmax                      !! number of available rows in constraint Jacobian
    integer(c_int)  :: nconmax                    !! number of potential contacts in contact list
    integer(c_int)  :: nstack                     !! number of fields in mjData stack
    integer(c_int)  :: nuserdata                  !! number of extra fields in mjData
    integer(c_int)  :: nsensordata                !! number of fields in sensor data vector

    integer(c_int)  :: nbuffer                    !! number of bytes in buffer

    !! ------------------------------- options and statistics

    type(mjOption)  :: opt                   !! physics options
    type(mjVisual)  :: vis                   !! visualization options
    type(mjStatistic) :: stat                !! model statistics

    !! ------------------------------- buffers

    !! main buffer
    type(c_ptr)       :: buffer               !! main buffer all pointers point in it    (nbuffer)

    !! default generalized coordinates
    type(c_ptr)       :: qpos0                !! qpos values at default pose              (nq x 1)
    type(c_ptr)       :: qpos_spring          !! reference pose for springs               (nq x 1)

    !! bodies
    type(c_ptr)       :: body_parentid        !! id of body's parent                      (nbody x 1)
    type(c_ptr)       :: body_rootid          !! id of root above body                    (nbody x 1)
    type(c_ptr)       :: body_weldid          !! id of body that this body is welded to   (nbody x 1)
    type(c_ptr)       :: body_mocapid         !! id of mocap data -1: none               (nbody x 1)
    type(c_ptr)       :: body_jntnum          !! number of joints for this body           (nbody x 1)
    type(c_ptr)       :: body_jntadr          !! start addr of joints -1: no joints      (nbody x 1)
    type(c_ptr)       :: body_dofnum          !! number of motion degrees of freedom      (nbody x 1)
    type(c_ptr)       :: body_dofadr          !! start addr of dofs -1: no dofs          (nbody x 1)
    type(c_ptr)       :: body_geomnum         !! number of geoms                          (nbody x 1)
    type(c_ptr)       :: body_geomadr         !! start addr of geoms -1: no geoms        (nbody x 1)
    type(c_ptr)       :: body_simple          !! body is simple (has diagonal M)          (nbody x 1)
    type(c_ptr)       :: body_sameframe       !! inertial frame is same as body frame     (nbody x 1)
    type(c_ptr)       :: body_pos             !! position offset rel. to parent body      (nbody x 3)
    type(c_ptr)       :: body_quat            !! orientation offset rel. to parent body   (nbody x 4)
    type(c_ptr)       :: body_ipos            !! local position of center of mass         (nbody x 3)
    type(c_ptr)       :: body_iquat           !! local orientation of inertia ellipsoid   (nbody x 4)
    type(c_ptr)       :: body_mass            !! mass                                     (nbody x 1)
    type(c_ptr)       :: body_subtreemass     !! mass of subtree starting at this body    (nbody x 1)
    type(c_ptr)       :: body_inertia         !! diagonal inertia in ipos/iquat frame     (nbody x 3)
    type(c_ptr)       :: body_invweight0      !! mean inv inert in qpos0 (trn, rot)       (nbody x 2)
    type(c_ptr)       :: body_user            !! user data                                (nbody x nuser_body)

    !! joints
    type(c_ptr)       :: jnt_type             !! type of joint (mjtJoint)                 (njnt x 1)
    type(c_ptr)       :: jnt_qposadr          !! start addr in 'qpos' for joint's data    (njnt x 1)
    type(c_ptr)       :: jnt_dofadr           !! start addr in 'qvel' for joint's data    (njnt x 1)
    type(c_ptr)       :: jnt_bodyid           !! id of joint's body                       (njnt x 1)
    type(c_ptr)       :: jnt_group            !! group for visibility                     (njnt x 1)
    type(c_ptr)       :: jnt_limited          !! does joint have limits                   (njnt x 1)
    type(c_ptr)       :: jnt_solref           !! constraint solver reference: limit       (njnt x mjNREF)
    type(c_ptr)       :: jnt_solimp           !! constraint solver impedance: limit       (njnt x mjNIMP)
    type(c_ptr)       :: jnt_pos              !! local anchor position                    (njnt x 3)
    type(c_ptr)       :: jnt_axis             !! local joint axis                         (njnt x 3)
    type(c_ptr)       :: jnt_stiffness        !! stiffness coefficient                    (njnt x 1)
    type(c_ptr)       :: jnt_range            !! joint limits                             (njnt x 2)
    type(c_ptr)       :: jnt_margin           !! min distance for limit detection         (njnt x 1)
    type(c_ptr)       :: jnt_user             !! user data                                (njnt x nuser_jnt)

    !! dofs
    type(c_ptr)       :: dof_bodyid           !! id of dof's body                         (nv x 1)
    type(c_ptr)       :: dof_jntid            !! id of dof's joint                        (nv x 1)
    type(c_ptr)       :: dof_parentid         !! id of dof's parent -1: none             (nv x 1)
    type(c_ptr)       :: dof_Madr             !! dof address in M-diagonal                (nv x 1)
    type(c_ptr)       :: dof_simplenum        !! number of consecutive simple dofs        (nv x 1)
    type(c_ptr)       :: dof_solref           !! constraint solver reference:frictionloss (nv x mjNREF)
    type(c_ptr)       :: dof_solimp           !! constraint solver impedance:frictionloss (nv x mjNIMP)
    type(c_ptr)       :: dof_frictionloss     !! dof friction loss                        (nv x 1)
    type(c_ptr)       :: dof_armature         !! dof armature inertia/mass                (nv x 1)
    type(c_ptr)       :: dof_damping          !! damping coefficient                      (nv x 1)
    type(c_ptr)       :: dof_invweight0       !! diag. inverse inertia in qpos0           (nv x 1)
    type(c_ptr)       :: dof_M0               !! diag. inertia in qpos0                   (nv x 1)

    !! geoms
    type(c_ptr)       :: geom_type            !! geometric type (mjtGeom)                 (ngeom x 1)
    type(c_ptr)       :: geom_contype         !! geom contact type                        (ngeom x 1)
    type(c_ptr)       :: geom_conaffinity     !! geom contact affinity                    (ngeom x 1)
    type(c_ptr)       :: geom_condim          !! contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
    type(c_ptr)       :: geom_bodyid          !! id of geom's body                        (ngeom x 1)
    type(c_ptr)       :: geom_dataid          !! id of geom's mesh/hfield (-1: none)      (ngeom x 1)
    type(c_ptr)       :: geom_matid           !! material id for rendering                (ngeom x 1)
    type(c_ptr)       :: geom_group           !! group for visibility                     (ngeom x 1)
    type(c_ptr)       :: geom_priority        !! geom contact priority                    (ngeom x 1)
    type(c_ptr)       :: geom_sameframe       !! same as body frame (1) or iframe (2)     (ngeom x 1)
    type(c_ptr)       :: geom_solmix          !! mixing coef for solref/imp in geom pair  (ngeom x 1)
    type(c_ptr)       :: geom_solref          !! constraint solver reference: contact     (ngeom x mjNREF)
    type(c_ptr)       :: geom_solimp          !! constraint solver impedance: contact     (ngeom x mjNIMP)
    type(c_ptr)       :: geom_size            !! geom-specific size parameters            (ngeom x 3)
    type(c_ptr)       :: geom_rbound          !! radius of bounding sphere                (ngeom x 1)
    type(c_ptr)       :: geom_pos             !! local position offset rel. to body       (ngeom x 3)
    type(c_ptr)       :: geom_quat            !! local orientation offset rel. to body    (ngeom x 4)
    type(c_ptr)       :: geom_friction        !! friction for (slide, spin, roll)         (ngeom x 3)
    type(c_ptr)       :: geom_margin          !! detect contact if dist<margin            (ngeom x 1)
    type(c_ptr)       :: geom_gap             !! include in solver if dist<margin-gap     (ngeom x 1)
    type(c_ptr)       :: geom_fluid           !! fluid interaction parameters             (ngeom x mjNFLUID)
    type(c_ptr)       :: geom_user            !! user data                                (ngeom x nuser_geom)
    type(c_ptr)       :: geom_rgba            !! rgba when material is omitted            (ngeom x 4)

    !! sites
    type(c_ptr)       :: site_type            !! geom type for rendering (mjtGeom)        (nsite x 1)
    type(c_ptr)       :: site_bodyid          !! id of site's body                        (nsite x 1)
    type(c_ptr)       :: site_matid           !! material id for rendering                (nsite x 1)
    type(c_ptr)       :: site_group           !! group for visibility                     (nsite x 1)
    type(c_ptr)       :: site_sameframe       !! same as body frame (1) or iframe (2)     (nsite x 1)
    type(c_ptr)       :: site_size            !! geom size for rendering                  (nsite x 3)
    type(c_ptr)       :: site_pos             !! local position offset rel. to body       (nsite x 3)
    type(c_ptr)       :: site_quat            !! local orientation offset rel. to body    (nsite x 4)
    type(c_ptr)       :: site_user            !! user data                                (nsite x nuser_site)
    type(c_ptr)       :: site_rgba            !! rgba when material is omitted            (nsite x 4)

    !! cameras
    type(c_ptr)       ::   cam_mode             !! camera tracking mode (mjtCamLight)       (ncam x 1)
    type(c_ptr)       ::   cam_bodyid           !! id of camera's body                      (ncam x 1)
    type(c_ptr)       ::   cam_targetbodyid     !! id of targeted body -1: none            (ncam x 1)
    type(c_ptr)       ::   cam_pos              !! position rel. to body frame              (ncam x 3)
    type(c_ptr)       ::   cam_quat             !! orientation rel. to body frame           (ncam x 4)
    type(c_ptr)       ::   cam_poscom0          !! global position rel. to sub-com in qpos0 (ncam x 3)
    type(c_ptr)       ::   cam_pos0             !! global position rel. to body in qpos0    (ncam x 3)
    type(c_ptr)       ::   cam_mat0             !! global orientation in qpos0              (ncam x 9)
    type(c_ptr)       ::   cam_fovy             !! y-field of view (deg)                    (ncam x 1)
    type(c_ptr)       ::   cam_ipd              !! inter-pupilary distance                  (ncam x 1)
    type(c_ptr)       ::   cam_user             !! user data                                (ncam x nuser_cam)

    !! lights
    type(c_ptr)       ::  light_mode           !! light tracking mode (mjtCamLight)        (nlight x 1)
    type(c_ptr)       ::  light_bodyid         !! id of light's body                       (nlight x 1)
    type(c_ptr)       ::  light_targetbodyid   !! id of targeted body -1: none            (nlight x 1)
    type(c_ptr)       ::  light_directional    !! directional light                        (nlight x 1)
    type(c_ptr)       ::  light_castshadow     !! does light cast shadows                  (nlight x 1)
    type(c_ptr)       ::  light_active         !! is light on                              (nlight x 1)
    type(c_ptr)       ::  light_pos            !! position rel. to body frame              (nlight x 3)
    type(c_ptr)       ::  light_dir            !! direction rel. to body frame             (nlight x 3)
    type(c_ptr)       ::  light_poscom0        !! global position rel. to sub-com in qpos0 (nlight x 3)
    type(c_ptr)       ::  light_pos0           !! global position rel. to body in qpos0    (nlight x 3)
    type(c_ptr)       ::  light_dir0           !! global direction in qpos0                (nlight x 3)
    type(c_ptr)       ::  light_attenuation    !! OpenGL attenuation (quadratic model)     (nlight x 3)
    type(c_ptr)       ::  light_cutoff         !! OpenGL cutoff                            (nlight x 1)
    type(c_ptr)       ::  light_exponent       !! OpenGL exponent                          (nlight x 1)
    type(c_ptr)       ::  light_ambient        !! ambient rgb (alpha=1)                    (nlight x 3)
    type(c_ptr)       ::  light_diffuse        !! diffuse rgb (alpha=1)                    (nlight x 3)
    type(c_ptr)       ::  light_specular       !! specular rgb (alpha=1)                   (nlight x 3)

    !! meshes
    type(c_ptr)       ::    mesh_vertadr         !! first vertex address                     (nmesh x 1)
    type(c_ptr)       ::    mesh_vertnum         !! number of vertices                       (nmesh x 1)
    type(c_ptr)       ::    mesh_texcoordadr     !! texcoord data address -1: no texcoord   (nmesh x 1)
    type(c_ptr)       ::    mesh_faceadr         !! first face address                       (nmesh x 1)
    type(c_ptr)       ::    mesh_facenum         !! number of faces                          (nmesh x 1)
    type(c_ptr)       ::    mesh_graphadr        !! graph data address -1: no graph         (nmesh x 1)
    type(c_ptr)       ::    mesh_vert            !! vertex positions for all meshe           (nmeshvert x 3)
    type(c_ptr)       ::    mesh_normal          !! vertex normals for all meshes            (nmeshvert x 3)
    type(c_ptr)       ::    mesh_texcoord        !! vertex texcoords for all meshes          (nmeshtexvert x 2)
    type(c_ptr)       ::    mesh_face            !! triangle face data                       (nmeshface x 3)
    type(c_ptr)       ::    mesh_graph           !! convex graph data                        (nmeshgraph x 1)

    !! skins
    type(c_ptr)       ::    skin_matid           !! skin material id -1: none               (nskin x 1)
    type(c_ptr)       ::    skin_rgba            !! skin rgba                                (nskin x 4)
    type(c_ptr)       ::    skin_inflate         !! inflate skin in normal direction         (nskin x 1)
    type(c_ptr)       ::    skin_vertadr         !! first vertex address                     (nskin x 1)
    type(c_ptr)       ::    skin_vertnum         !! number of vertices                       (nskin x 1)
    type(c_ptr)       ::    skin_texcoordadr     !! texcoord data address -1: no texcoord   (nskin x 1)
    type(c_ptr)       ::    skin_faceadr         !! first face address                       (nskin x 1)
    type(c_ptr)       ::    skin_facenum         !! number of faces                          (nskin x 1)
    type(c_ptr)       ::    skin_boneadr         !! first bone in skin                       (nskin x 1)
    type(c_ptr)       ::    skin_bonenum         !! number of bones in skin                  (nskin x 1)
    type(c_ptr)       ::    skin_vert            !! vertex positions for all skin meshes     (nskinvert x 3)
    type(c_ptr)       ::    skin_texcoord        !! vertex texcoords for all skin meshes     (nskintexvert x 2)
    type(c_ptr)       ::    skin_face            !! triangle faces for all skin meshes       (nskinface x 3)
    type(c_ptr)       ::    skin_bonevertadr     !! first vertex in each bone                (nskinbone x 1)
    type(c_ptr)       ::    skin_bonevertnum     !! number of vertices in each bone          (nskinbone x 1)
    type(c_ptr)       ::    skin_bonebindpos     !! bind pos of each bone                    (nskinbone x 3)
    type(c_ptr)       ::    skin_bonebindquat    !! bind quat of each bone                   (nskinbone x 4)
    type(c_ptr)       ::    skin_bonebodyid      !! body id of each bone                     (nskinbone x 1)
    type(c_ptr)       ::    skin_bonevertid      !! mesh ids of vertices in each bone        (nskinbonevert x 1)
    type(c_ptr)       ::    skin_bonevertweight  !! weights of vertices in each bone         (nskinbonevert x 1)

    !! height fields
    type(c_ptr)       ::   hfield_size          !! (x, y, z_top, z_bottom)                  (nhfield x 4)
    type(c_ptr)       ::   hfield_nrow          !! number of rows in grid                   (nhfield x 1)
    type(c_ptr)       ::   hfield_ncol          !! number of columns in grid                (nhfield x 1)
    type(c_ptr)       ::   hfield_adr           !! address in hfield_data                   (nhfield x 1)
    type(c_ptr)       ::   hfield_data          !! elevation data                           (nhfielddata x 1)

    !! textures
    type(c_ptr)       ::  tex_type             !! texture type (mjtTexture)                (ntex x 1)
    type(c_ptr)       ::  tex_height           !! number of rows in texture image          (ntex x 1)
    type(c_ptr)       ::  tex_width            !! number of columns in texture image       (ntex x 1)
    type(c_ptr)       ::  tex_adr              !! address in rgb                           (ntex x 1)
    type(c_ptr)       ::  tex_rgb              !! rgb (alpha = 1)                          (ntexdata x 1)

    !! materials
    type(c_ptr)       ::  mat_texid            !! texture id -1: none                     (nmat x 1)
    type(c_ptr)       ::  mat_texuniform       !! make texture cube uniform                (nmat x 1)
    type(c_ptr)       ::  mat_texrepeat        !! texture repetition for 2d mapping        (nmat x 2)
    type(c_ptr)       ::  mat_emission         !! emission (x rgb)                         (nmat x 1)
    type(c_ptr)       ::  mat_specular         !! specular (x white)                       (nmat x 1)
    type(c_ptr)       ::  mat_shininess        !! shininess coef                           (nmat x 1)
    type(c_ptr)       ::  mat_reflectance      !! reflectance (0: disable)                 (nmat x 1)
    type(c_ptr)       ::  mat_rgba             !! rgba                                     (nmat x 4)

    !! predefined geom pairs for collision detection has precedence over exclude
    type(c_ptr)       ::   pair_dim             !! contact dimensionality                   (npair x 1)
    type(c_ptr)       ::   pair_geom1           !! id of geom1                              (npair x 1)
    type(c_ptr)       ::   pair_geom2           !! id of geom2                              (npair x 1)
    type(c_ptr)       ::   pair_signature       !! (body1+1)<<16 + body2+1                  (npair x 1)
    type(c_ptr)       ::   pair_solref          !! constraint solver reference: contact     (npair x mjNREF)
    type(c_ptr)       ::   pair_solimp          !! constraint solver impedance: contact     (npair x mjNIMP)
    type(c_ptr)       ::   pair_margin          !! detect contact if dist<margin            (npair x 1)
    type(c_ptr)       ::   pair_gap             !! include in solver if dist<margin-gap     (npair x 1)
    type(c_ptr)       ::   pair_friction        !! tangent1, 2, spin, roll1, 2              (npair x 5)

    !! excluded body pairs for collision detection
    type(c_ptr)       ::      exclude_signature    !! (body1+1)<<16 + body2+1                  (nexclude x 1)

    !! equality constraints
    type(c_ptr)       ::  eq_type              !! constraint type (mjtEq)                  (neq x 1)
    type(c_ptr)       ::  eq_obj1id            !! id of object 1                           (neq x 1)
    type(c_ptr)       ::  eq_obj2id            !! id of object 2                           (neq x 1)
    type(c_ptr)       ::  eq_active            !! enable/disable constraint                (neq x 1)
    type(c_ptr)       ::  eq_solref            !! constraint solver reference              (neq x mjNREF)
    type(c_ptr)       ::  eq_solimp            !! constraint solver impedance              (neq x mjNIMP)
    type(c_ptr)       ::  eq_data              !! numeric data for constraint              (neq x mjNEQDATA)

    !! tendons
    type(c_ptr)       ::   tendon_adr           !! address of first object in tendon's path (ntendon x 1)
    type(c_ptr)       ::   tendon_num           !! number of objects in tendon's path       (ntendon x 1)
    type(c_ptr)       ::   tendon_matid         !! material id for rendering                (ntendon x 1)
    type(c_ptr)       ::   tendon_group         !! group for visibility                     (ntendon x 1)
    type(c_ptr)       ::   tendon_limited       !! does tendon have length limits           (ntendon x 1)
    type(c_ptr)       ::   tendon_width         !! width for rendering                      (ntendon x 1)
    type(c_ptr)       ::   tendon_solref_lim    !! constraint solver reference: limit       (ntendon x mjNREF)
    type(c_ptr)       ::   tendon_solimp_lim    !! constraint solver impedance: limit       (ntendon x mjNIMP)
    type(c_ptr)       ::   tendon_solref_fri    !! constraint solver reference: friction    (ntendon x mjNREF)
    type(c_ptr)       ::   tendon_solimp_fri    !! constraint solver impedance: friction    (ntendon x mjNIMP)
    type(c_ptr)       ::   tendon_range         !! tendon length limits                     (ntendon x 2)
    type(c_ptr)       ::   tendon_margin        !! min distance for limit detection         (ntendon x 1)
    type(c_ptr)       ::   tendon_stiffness     !! stiffness coefficient                    (ntendon x 1)
    type(c_ptr)       ::   tendon_damping       !! damping coefficient                      (ntendon x 1)
    type(c_ptr)       ::   tendon_frictionloss  !! loss due to friction                     (ntendon x 1)
    type(c_ptr)       ::   tendon_lengthspring  !! tendon length in qpos_spring             (ntendon x 1)
    type(c_ptr)       ::   tendon_length0       !! tendon length in qpos0                   (ntendon x 1)
    type(c_ptr)       ::   tendon_invweight0    !! inv. weight in qpos0                     (ntendon x 1)
    type(c_ptr)       ::   tendon_user          !! user data                                (ntendon x nuser_tendon)
    type(c_ptr)       ::   tendon_rgba          !! rgba when material is omitted            (ntendon x 4)

    !! list of all wrap objects in tendon paths
    type(c_ptr)       ::   wrap_type            !! wrap object type (mjtWrap)               (nwrap x 1)
    type(c_ptr)       ::   wrap_objid           !! object id: geom, site, joint             (nwrap x 1)
    type(c_ptr)       ::   wrap_prm             !! divisor, joint coef, or site id          (nwrap x 1)

    !! actuators
    type(c_ptr)       ::  actuator_trntype     !! transmission type (mjtTrn)               (nu x 1)
    type(c_ptr)       ::  actuator_dyntype     !! dynamics type (mjtDyn)                   (nu x 1)
    type(c_ptr)       ::  actuator_gaintype    !! gain type (mjtGain)                      (nu x 1)
    type(c_ptr)       ::  actuator_biastype    !! bias type (mjtBias)                      (nu x 1)
    type(c_ptr)       ::  actuator_trnid       !! transmission id: joint, tendon, site     (nu x 2)
    type(c_ptr)       ::  actuator_group       !! group for visibility                     (nu x 1)
    type(c_ptr)       ::  actuator_ctrllimited !! is control limited                       (nu x 1)
    type(c_ptr)       ::  actuator_forcelimited!! is force limited                         (nu x 1)
    type(c_ptr)       ::  actuator_actlimited  !! is activation limited                    (nu x 1)
    type(c_ptr)       ::  actuator_dynprm      !! dynamics parameters                      (nu x mjNDYN)
    type(c_ptr)       ::  actuator_gainprm     !! gain parameters                          (nu x mjNGAIN)
    type(c_ptr)       ::  actuator_biasprm     !! bias parameters                          (nu x mjNBIAS)
    type(c_ptr)       ::  actuator_ctrlrange   !! range of controls                        (nu x 2)
    type(c_ptr)       ::  actuator_forcerange  !! range of forces                          (nu x 2)
    type(c_ptr)       ::  actuator_actrange    !! range of activations                     (nu x 2)
    type(c_ptr)       ::  actuator_gear        !! scale length and transmitted force       (nu x 6)
    type(c_ptr)       ::  actuator_cranklength !! crank length for slider-crank            (nu x 1)
    type(c_ptr)       ::  actuator_acc0        !! acceleration from unit force in qpos0    (nu x 1)
    type(c_ptr)       ::  actuator_length0     !! actuator length in qpos0                 (nu x 1)
    type(c_ptr)       ::  actuator_lengthrange !! feasible actuator length range           (nu x 2)
    type(c_ptr)       ::  actuator_user        !! user data                                (nu x nuser_actuator)

    !! sensors
    type(c_ptr)       ::   sensor_type          !! sensor type (mjtSensor)                  (nsensor x 1)
    type(c_ptr)       ::   sensor_datatype      !! numeric data type (mjtDataType)          (nsensor x 1)
    type(c_ptr)       ::   sensor_needstage     !! required compute stage (mjtStage)        (nsensor x 1)
    type(c_ptr)       ::   sensor_objtype       !! type of sensorized object (mjtObj)       (nsensor x 1)
    type(c_ptr)       ::   sensor_objid         !! id of sensorized object                  (nsensor x 1)
    type(c_ptr)       ::   sensor_reftype       !! type of reference frame (mjtObj)         (nsensor x 1)
    type(c_ptr)       ::   sensor_refid         !! id of reference frame -1: global frame  (nsensor x 1)
    type(c_ptr)       ::   sensor_dim           !! number of scalar outputs                 (nsensor x 1)
    type(c_ptr)       ::   sensor_adr           !! address in sensor array                  (nsensor x 1)
    type(c_ptr)       ::   sensor_cutoff        !! cutoff for real and positive 0: ignore  (nsensor x 1)
    type(c_ptr)       ::   sensor_noise         !! noise standard deviation                 (nsensor x 1)
    type(c_ptr)       ::   sensor_user          !! user data                                (nsensor x nuser_sensor)

    !! custom numeric fields
    type(c_ptr)       ::   numeric_adr          !! address of field in numeric_data         (nnumeric x 1)
    type(c_ptr)       ::   numeric_size         !! size of numeric field                    (nnumeric x 1)
    type(c_ptr)       ::   numeric_data         !! array of all numeric fields              (nnumericdata x 1)

    !! custom text fields
    type(c_ptr)       ::     text_adr             !! address of text in text_data             (ntext x 1)
    type(c_ptr)       ::     text_size            !! size of text field (strlen+1)            (ntext x 1)
    type(c_ptr)       ::     text_data            !! array of all text fields (0-terminated)  (ntextdata x 1)

    !! custom tuple fields
    type(c_ptr)       ::   tuple_adr            !! address of text in text_data             (ntuple x 1)
    type(c_ptr)       ::   tuple_size           !! number of objects in tuple               (ntuple x 1)
    type(c_ptr)       ::   tuple_objtype        !! array of object types in all tuples      (ntupledata x 1)
    type(c_ptr)       ::   tuple_objid          !! array of object ids in all tuples        (ntupledata x 1)
    type(c_ptr)       ::   tuple_objprm         !! array of object params in all tuples     (ntupledata x 1)

    !! keyframes
    type(c_ptr)       ::   key_time             !! key time                                 (nkey x 1)
    type(c_ptr)       ::   key_qpos             !! key position                             (nkey x nq)
    type(c_ptr)       ::   key_qvel             !! key velocity                             (nkey x nv)
    type(c_ptr)       ::   key_act              !! key activation                           (nkey x na)
    type(c_ptr)       ::   key_mpos             !! key mocap position                       (nkey x 3*nmocap)
    type(c_ptr)       ::   key_mquat            !! key mocap quaternion                     (nkey x 4*nmocap)

    !! names
    type(c_ptr)       ::     name_bodyadr         !! body name pointers                       (nbody x 1)
    type(c_ptr)       ::     name_jntadr          !! joint name pointers                      (njnt x 1)
    type(c_ptr)       ::     name_geomadr         !! geom name pointers                       (ngeom x 1)
    type(c_ptr)       ::     name_siteadr         !! site name pointers                       (nsite x 1)
    type(c_ptr)       ::     name_camadr          !! camera name pointers                     (ncam x 1)
    type(c_ptr)       ::     name_lightadr        !! light name pointers                      (nlight x 1)
    type(c_ptr)       ::     name_meshadr         !! mesh name pointers                       (nmesh x 1)
    type(c_ptr)       ::     name_skinadr         !! skin name pointers                       (nskin x 1)
    type(c_ptr)       ::     name_hfieldadr       !! hfield name pointers                     (nhfield x 1)
    type(c_ptr)       ::     name_texadr          !! texture name pointers                    (ntex x 1)
    type(c_ptr)       ::     name_matadr          !! material name pointers                   (nmat x 1)
    type(c_ptr)       ::     name_pairadr         !! geom pair name pointers                  (npair x 1)
    type(c_ptr)       ::     name_excludeadr      !! exclude name pointers                    (nexclude x 1)
    type(c_ptr)       ::     name_eqadr           !! equality constraint name pointers        (neq x 1)
    type(c_ptr)       ::     name_tendonadr       !! tendon name pointers                     (ntendon x 1)
    type(c_ptr)       ::     name_actuatoradr     !! actuator name pointers                   (nu x 1)
    type(c_ptr)       ::     name_sensoradr       !! sensor name pointers                     (nsensor x 1)
    type(c_ptr)       ::     name_numericadr      !! numeric name pointers                    (nnumeric x 1)
    type(c_ptr)       ::     name_textadr         !! text name pointers                       (ntext x 1)
    type(c_ptr)       ::     name_tupleadr        !! tuple name pointers                      (ntuple x 1)
    type(c_ptr)       ::     name_keyadr          !! keyframe name pointers                   (nkey x 1)
    type(c_ptr)       ::     names                !! names of all objects, 0-terminated       (nnames x 1)
  end type mjModel
  
end module mod_mjmodel