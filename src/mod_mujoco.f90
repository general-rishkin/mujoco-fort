module mod_mujoco
  use iso_c_binding, c_char_ptr => c_ptr, c_const_char_ptr => c_ptr
  use mod_mjdata
  use mod_mjmodel
  use mod_mjrender
  use mod_mjtnum
  use mod_mjui
  use mod_mjvisualize

  implicit none
  
  integer(c_int), parameter :: mjVERSION_HEADER = 213

  ! user error and memory handlers
  abstract interface
    !! MJAPI extern void  (*mju_user_error)(const char*)
    subroutine mju_user_error(msg) bind(c)
      import :: c_char
      character(c_char), intent(in) :: msg(*)
    end subroutine mju_user_error

    !! MJAPI extern void  (*mju_user_warning)(const char*)
    subroutine mju_user_warning(msg) bind(c)
      import :: c_char
      character(c_char), intent(in) :: msg(*)
    end subroutine mju_user_warning

    !! MJAPI extern void* (*mju_user_malloc)(size_t)
    subroutine mju_user_malloc(m) bind(c)
      import :: c_size_t
      integer(c_size_t), value, intent(in) :: m
    end subroutine mju_user_malloc

    !! MJAPI extern void  (*mju_user_free)(void*)
    subroutine mju_user_free(m) bind(c)
      import :: c_ptr
      type(c_ptr), intent(inout) :: m
    end subroutine mju_user_free
  end interface

  ! callbacks extending computation pipeline
  type(c_funptr), bind(C, name="mjcb_passive") :: mjcb_passive
  type(c_funptr), bind(C, name="mjcb_control") :: mjcb_control
  type(c_funptr), bind(C, name="mjcb_contactfilter") :: mjcb_contactfilter
  type(c_funptr), bind(C, name="mjcb_sensor") :: mjcb_sensor
  type(c_funptr), bind(C, name="mjcb_time") :: mjcb_time
  type(c_funptr), bind(C, name="mjcb_act_dyn") :: mjcb_act_dyn
  type(c_funptr), bind(C, name="mjcb_act_gain") :: mjcb_act_gain
  type(c_funptr), bind(C, name="mjcb_act_bias") :: mjcb_act_bias


  ! collision function table
  ! type :: collision_func_ptrs
  !   type(c_funptr) :: func_ptr  
  ! end type collision_func_ptrs
  ! type(collision_func_ptrs) :: mjCOLLISIONFUNC(mjNGEOMTYPES, mjNGEOMTYPES)
  type(C_PTR), public, bind(C, name="mjCOLLISIONFUNC") :: mjCOLLISIONFUNC

  
  ! string names
  ! character(kind=c_char) :: mjDISABLESTRING(mjNDISABLE)
  ! character(kind=c_char) :: mjENABLESTRING(mjNENABLE)
  ! character(kind=c_char) :: mjTIMERSTRING(mjNTIMER)
  ! character(kind=c_char) :: mjLABELSTRING(mjNLABEL)
  ! character(kind=c_char) :: mjFRAMESTRING(mjNFRAME)
  ! character(kind=c_char) :: mjVISSTRING(3, mjNVISFLAG)
  ! character(kind=c_char) :: mjRNDSTRING(3, mjNRNDFLAG)
  type(C_PTR), public, bind(C, name="mjDISABLESTRING") :: mjDISABLESTRING
  type(C_PTR), public, bind(C, name="mjENABLESTRING")  :: mjENABLESTRING
  type(C_PTR), public, bind(C, name="mjTIMERSTRING") :: mjTIMERSTRING
  type(C_PTR), public, bind(C, name="mjLABELSTRING") :: mjLABELSTRING
  type(C_PTR), public, bind(C, name="mjFRAMESTRING") :: mjFRAMESTRING
  type(C_PTR), public, bind(C, name="mjVISSTRING") :: mjVISSTRING
  type(C_PTR), public, bind(C, name="mjRNDSTRING") :: mjRNDSTRING

  
  interface
  !---------------------------------- Activation ----------------------------------------------------

    !! Return 1 (for backward compatibility).
    !MJAPI int mj_activate(const char* filename)
    integer(c_int) function mj_activate(filename) bind(c, name="mj_activate")
      import :: c_int, c_char
      character(c_char), intent(in)  :: filename(*)
    end function mj_activate

    !! Do nothing (for backward compatibility).
    ! MJAPI void mj_deactivate(void)
    subroutine mj_deactivate() bind(c, name="mj_deactivate")
    end subroutine mj_deactivate


    !---------------------------------- Virtual file system -------------------------------------------

    !! Initialize VFS to empty (no deallocation).
    ! MJAPI void mj_defaultVFS(mjVFS* vfs)
    subroutine mj_defaultVFS(vfs) bind(c, name="mj_defaultVFS")
      import :: c_ptr
      type(c_ptr), value :: vfs
    end subroutine mj_defaultVFS

    !! Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: failed to load.
    ! MJAPI int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename)
    integer(c_int) function mj_addFileVFS(vfs, directory, filename) bind(c)
      import :: mjVFS, c_char, c_int
      type(mjVFS), intent(inout)    :: vfs
      character(c_char), intent(in) :: filename(*), directory(*)
    end function mj_addFileVFS

    !! Make empty file in VFS, return 0: success, 1: full, 2: repeated name.
    ! MJAPI int mj_makeEmptyFileVFS(mjVFS* vfs, const char* filename, int filesize)
    integer(c_int) function mj_makeEmptyFileVFS(vfs, filename, filesize) bind(c)
      import :: mjVFS, c_char, c_int
      type(mjVFS), intent(inout)        :: vfs
      character(c_char), intent(in)     :: filename(*)
      integer(c_int), value, intent(in) :: filesize
    end function mj_makeEmptyFileVFS

    !! Return file index in VFS, or -1 if not found in VFS.
    ! MJAPI int mj_findFileVFS(const mjVFS* vfs, const char* filename)
    integer(c_int) function mj_findFileVFS(vfs, filename) bind(c)
      import :: mjVFS, c_char, c_int
      type(mjVFS), intent(in)           :: vfs
      character(c_char), intent(in)     :: filename(*)
    end function mj_findFileVFS

    !! Delete file from VFS, return 0: success, -1: not found in VFS.
    ! MJAPI int mj_deleteFileVFS(mjVFS* vfs, const char* filename)
    integer(c_int) function mj_deleteFileVFS(vfs, filename) bind(c)
      import :: mjVFS, c_char, c_int
      type(mjVFS), intent(inout)        :: vfs
      character(c_char), intent(in)     :: filename(*)
    end function mj_deleteFileVFS

    !! Delete all files from VFS.
    ! MJAPI void mj_deleteVFS(mjVFS* vfs)
    subroutine mj_deleteVFS(vfs) bind(c)
      import :: mjVFS
      type(mjVFS), intent(inout) :: vfs
    end subroutine mj_deleteVFS


    !---------------------------------- Parse and compile ---------------------------------------------

    !! Parse XML file in MJCF or URDF format, compile it, return low-level model.
    !! If vfs is not NULL, look up files in vfs before reading from disk.
    !! If error is not NULL, it must have size error_sz.
    ! MJAPI mjModel* mj_loadXML(const char* filename, const mjVFS* vfs, char* error, int error_sz)
    function mj_loadXML(filename, vfs, error, error_sz) result(model) bind(C, name="mj_loadXML")
      import :: c_char, c_ptr, c_int
      character(c_char), intent(IN)     :: filename(*)
      type(c_ptr), value                :: vfs
      character(c_char), intent(INOUT)  :: error(*)
      integer(C_INT), value, intent(IN) :: error_sz
      type(c_ptr)                       :: model
    end function mj_loadXML

    !! Update XML data structures with info from low-level model, save as MJCF.
    !! If error is not NULL, it must have size error_sz.
    ! MJAPI int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz)
    integer(c_int) function mj_saveLastXML(filename, m, error, error_sz) bind(c, name="mj_saveLastXML")
      import :: mjModel, c_char, c_int
      character(c_char), intent(in)     :: filename(*)
      type(mjModel), intent(in)         :: m 
      character(c_char), intent(inout)  :: error(*)
      integer(c_int), value, intent(in) :: error_sz
    end function mj_saveLastXML

    !! Free last XML model if loaded. Called internally at each load.
    ! MJAPI void mj_freeLastXML(void)
    subroutine mj_freeLastXML() bind(c, name="mj_freeLastXML")
    end subroutine mj_freeLastXML

    !! Print internal XML schema as plain text or HTML, with style-padding or &nbsp.
    ! MJAPI int mj_printSchema(const char* filename, char* buffer, int buffer_sz, int flg_html, int flg_pad)
    integer(c_int) function mj_printSchema(filename, buffer, buffer_sz, flg_html, flg_pad) bind(c, name="mj_printSchema")
      import :: c_char, c_int
      character(c_char), intent(in)     :: filename(*)
      character(c_char), intent(inout)  :: buffer(*)
      integer(c_int), value, intent(in) :: buffer_sz, flg_html, flg_pad
    end function mj_printSchema

    !---------------------------------- Main simulation -----------------------------------------------

    !! Advance simulation, use control callback to obtain external force and control.
    ! MJAPI void mj_step(const mjModel* m, mjData* d)
    subroutine mj_step(m, d) bind(c, name="mj_step")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_step

    !! Advance simulation in two steps: before external force and control is set by user.
    ! MJAPI void mj_step1(const mjModel* m, mjData* d)
    subroutine mj_step1(m, d) bind(c, name="mj_step1")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_step1

    !! Advance simulation in two steps: after external force and control is set by user.
    ! MJAPI void mj_step2(const mjModel* m, mjData* d)
    subroutine mj_step2(m, d) bind(c, name="mj_step2")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_step2

    !! Forward dynamics: same as mj_step but do not integrate in time.
    ! MJAPI void mj_forward(const mjModel* m, mjData* d)
    subroutine mj_forward(m, d) bind(c, name="mj_forward")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_forward

    !! Inverse dynamics: qacc must be set before calling.
    ! MJAPI void mj_inverse(const mjModel* m, mjData* d)
    subroutine mj_inverse(m, d) bind(c, name="mj_inverse")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_inverse

    !! Forward dynamics with skip; skipstage is mjtStage.
    ! MJAPI void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor)
    subroutine mj_forwardSkip(m, d, skipstage, skipsensor) bind(c, name="mj_forwardSkip")
      import :: mjModel, mjData, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: skipstage, skipsensor
    end subroutine mj_forwardSkip

    !! Inverse dynamics with skip; skipstage is mjtStage.
    ! MJAPI void mj_inverseSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor)
    subroutine mj_inverseSkip(m, d, skipstage, skipsensor) bind(c, name="mj_inverseSkip")
      import :: mjModel, mjData, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: skipstage, skipsensor
    end subroutine mj_inverseSkip


    !---------------------------------- Initialization ------------------------------------------------

    !! Set default options for length range computation.
    ! MJAPI void mj_defaultLROpt(mjLROpt* opt)
    subroutine mj_defaultLROpt(opt) bind(c)
      import :: mjLROpt
      type(mjLROpt), intent(inout)  :: opt
    end subroutine mj_defaultLROpt

    !! Set solver parameters to default values.
    ! MJAPI void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp)
    subroutine mj_defaultSolRefImp(solref, solimp) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)  :: solref, solimp
    end subroutine mj_defaultSolRefImp

    !! Set physics options to default values.
    ! MJAPI void mj_defaultOption(mjOption* opt)
    subroutine mj_defaultOption(opt) bind(c)
      import :: mjOption
      type(mjOption), intent(inout)  :: opt
    end subroutine mj_defaultOption

    !! Set visual options to default values.
    ! MJAPI void mj_defaultVisual(mjVisual* vis)
    subroutine mj_defaultVisual(vis) bind(c)
      import :: mjVisual
      type(mjVisual), intent(inout)  :: vis
    end subroutine mj_defaultVisual

    !! Copy mjModel, allocate new if dest is NULL.
    ! MJAPI mjModel* mj_copyModel(mjModel* dest, const mjModel* src)
    type(c_ptr) function mj_copyModel(dest, src) bind(c)
      import :: mjModel, c_ptr
      type(mjModel), intent(out)    :: dest
      type(mjModel), intent(in)     :: src
    end function mj_copyModel

    !! Save model to binary MJB file or memory buffer buffer has precedence when given.
    ! MJAPI void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz)
    subroutine mj_saveModel(m, filename, buffer, buffer_sz) bind(c)
      import :: mjModel, c_char, c_int, c_ptr
      type(mjModel), value, intent(in)  :: m
      character(c_char), intent(in)     :: filename(*)
      type(c_ptr), intent(inout)        :: buffer
      integer(c_int), value, intent(in) :: buffer_sz
    end subroutine mj_saveModel

    !! Load model from binary MJB file.
    !! If vfs is not NULL, look up file in vfs before reading from disk.
    ! MJAPI mjModel* mj_loadModel(const char* filename, const mjVFS* vfs)
    type(c_ptr) function mj_loadModel(filename, vfs) bind(c)
      import :: mjModel, c_char, c_ptr, mjVFS
      character(c_char), intent(in)     :: filename(*)
      type(mjVFS), value, intent(in)    :: vfs
    end function mj_loadModel

    !! Free memory allocation in model.
    ! MJAPI void mj_deleteModel(mjModel* m)
    subroutine mj_deleteModel(m) bind(c, name="mj_deleteModel")
      import :: c_ptr
      type(c_ptr), value                :: m
    end subroutine mj_deleteModel

    !! Return size of buffer needed to hold model.
    ! MJAPI int mj_sizeModel(const mjModel* m)
    integer(c_int) function mj_sizeModel(m) bind(c)
      import :: c_int, mjModel
      type(mjModel), value, intent(in)  :: m
    end function mj_sizeModel

    !! Allocate mjData correponding to given model.
    !! If the model buffer is unallocated the initial configuration will not be set.
    ! MJAPI mjData* mj_makeData(const mjModel* m)
    function mj_makeData(m) result(data) bind(C, name="mj_makeData")
      import :: C_PTR
      type(c_ptr), value  :: m
      type(c_ptr)         :: data
    end function mj_makeData

    !! Copy mjData.
    !! m is only required to contain the size fields from MJMODEL_INTS.
    ! MJAPI mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src)
    type(c_ptr) function mj_copyData(dest, m, src) bind(c, name="mj_copyData")
      import :: c_ptr, mjModel, mjData
      type(mjData), intent(inout)      :: dest
      type(mjModel), value, intent(in) :: m
      type(mjData), value, intent(in)  :: src
    end function mj_copyData

    !! Reset data to defaults.
    ! MJAPI void mj_resetData(const mjModel* m, mjData* d)
    subroutine mj_resetData(m, d) bind(c, name="mj_resetData")
      import :: c_ptr
      type(c_ptr), intent(in), value   :: m
      type(c_ptr), value               :: d
    end subroutine mj_resetData

    !! Reset data to defaults, fill everything else with debug_value.
    ! MJAPI void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value)
    subroutine mj_resetDataDebug(m, d, debug_value) bind(c, name="mj_resetDataDebug")
      import :: mjModel, mjData, c_signed_char
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      character(c_signed_char), value, intent(in)  :: debug_value
    end subroutine mj_resetDataDebug

    !! Reset data, set fields from specified keyframe.
    ! MJAPI void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key)
    subroutine mj_resetDataKeyframe(m, d, key) bind(c, name="mj_resetDataKeyframe")
      import :: mjModel, mjData, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: key
    end subroutine mj_resetDataKeyframe

    !! Allocate array of specified size on mjData stack. Call mju_error on stack overflow.
    ! MJAPI mjtNum* mj_stackAlloc(mjData* d, int size)
    real(mjtNum) function mj_stackAlloc(d, size) bind(c)
      import :: mjtNum, mjData, c_int
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: size
    end function mj_stackAlloc

    !! Free memory allocation in mjData.
    ! MJAPI void mj_deleteData(mjData* d)
    subroutine mj_deleteData(d) bind(c, name="mj_deleteData")
      import :: c_ptr
      type(c_ptr), value                :: d
    end subroutine mj_deleteData

    !! Reset all callbacks to NULL pointers (NULL is the default).
    ! MJAPI void mj_resetCallbacks(void)
    subroutine mj_resetCallbacks() bind(c)
    end subroutine mj_resetCallbacks

    !! Set constant fields of mjModel, corresponding to qpos0 configuration.
    ! MJAPI void mj_setConst(mjModel* m, mjData* d)
    subroutine mj_setConst(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), intent(inout)      :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_setConst
    
    !! Set actuator_lengthrange for specified actuator return 1 if ok, 0 if error.
    ! MJAPI int mj_setLengthRange(mjModel* m, mjData* d, int index, const mjLROpt* opt, char* error, int error_sz)
    integer(c_int) function mj_setLengthRange(m, d, index, opt, error, error_sz) bind(c)
      import :: c_int, mjModel, mjData, mjLROpt, c_char
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: index, error_sz
      type(mjLROpt), value, intent(in)  :: opt
      character(c_char), intent(in)     :: error(*)
    end function mj_setLengthRange


    !---------------------------------- Printing ------------------------------------------------------

    !! Print mjModel to text file, specifying format.
    !! float_format must be a valid printf-style format string for a single float value.
    ! MJAPI void mj_printFormattedModel(const mjModel* m, const char* filename, const char* float_format)
    subroutine mj_printFormattedModel(m, filename, float_format) bind(c)
      import :: mjModel, c_char
      type(mjModel), value, intent(in)  :: m
      character(c_char), intent(in)     :: filename(*), float_format(*)
    end subroutine mj_printFormattedModel

    !! Print model to text file.
    ! MJAPI void mj_printModel(const mjModel* m, const char* filename)
    subroutine mj_printModel(m, filename) bind(c)
      import :: mjModel, c_char
      type(mjModel), value, intent(in)  :: m
      character(c_char), intent(in)     :: filename(*)
    end subroutine mj_printModel

    !! Print mjData to text file, specifying format.
    !! float_format must be a valid printf-style format string for a single float value
    ! MJAPI void mj_printFormattedData(const mjModel* m, mjData* d, const char* filename, const char* float_format)
    subroutine mj_printFormattedData(m, d, filename, float_format) bind(c)
      import :: mjModel, mjData, c_char
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      character(c_char), intent(in)     :: filename(*), float_format(*)
    end subroutine mj_printFormattedData

    !! Print data to text file.
    ! MJAPI void mj_printData(const mjModel* m, mjData* d, const char* filename)
    subroutine mj_printData(m, d, filename) bind(c)
      import :: mjModel, mjData, c_char
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      character(c_char), intent(in)     :: filename(*)
    end subroutine mj_printData

    !! Print matrix to screen.
    ! MJAPI void mju_printMat(const mjtNum* mat, int nr, int nc)
    subroutine mju_printMat(mat, nr, nc) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), value, intent(in)   :: mat
      integer(c_int), value, intent(in) :: nr, nc
    end subroutine mju_printMat

    !! Print sparse matrix to screen.
    ! MJAPI void mju_printMatSparse(const mjtNum* mat, int nr, const int* rownnz, const int* rowadr, const int* colind)
    subroutine mju_printMatSparse(mat, nr, rownnz, rowadr, colind) bind(c)
      import :: mjtNum, c_int, c_ptr
      real(mjtNum), value, intent(in)   :: mat
      integer(c_int), value, intent(in) :: nr
      type(c_ptr), value, intent(in)    :: rownnz, rowadr, colind
    end subroutine mju_printMatSparse


    !---------------------------------- Components ----------------------------------------------------

    !! Run position-dependent computations.
    ! MJAPI void mj_fwdPosition(const mjModel* m, mjData* d)
    subroutine mj_fwdPosition(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_fwdPosition

    !! Run velocity-dependent computations.
    ! MJAPI void mj_fwdVelocity(const mjModel* m, mjData* d)
    subroutine mj_fwdVelocity(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_fwdVelocity

    !! Compute actuator force qfrc_actuator.
    ! MJAPI void mj_fwdActuation(const mjModel* m, mjData* d)
    subroutine mj_fwdActuation(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_fwdActuation

    !! Add up all non-constraint forces, compute qacc_unc.
    ! MJAPI void mj_fwdAcceleration(const mjModel* m, mjData* d)
    subroutine mj_fwdAcceleration(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_fwdAcceleration

    !! Run selected constraint solver.
    ! MJAPI void mj_fwdConstraint(const mjModel* m, mjData* d)
    subroutine mj_fwdConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_fwdConstraint

    !! Euler integrator, semi-implicit in velocity.
    ! MJAPI void mj_Euler(const mjModel* m, mjData* d)
    subroutine mj_Euler(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_Euler

    !! Runge-Kutta explicit order-N integrator.
    ! MJAPI void mj_RungeKutta(const mjModel* m, mjData* d, int N)
    subroutine mj_RungeKutta(m, d, N) bind(c)
      import :: mjModel, mjData, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: N
    end subroutine mj_RungeKutta

    !! Run position-dependent computations in inverse dynamics.
    ! MJAPI void mj_invPosition(const mjModel* m, mjData* d)
    subroutine mj_invPosition(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_invPosition

    !! Run velocity-dependent computations in inverse dynamics.
    ! MJAPI void mj_invVelocity(const mjModel* m, mjData* d)
    subroutine mj_invVelocity(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_invVelocity

    !! Apply the analytical formula for inverse constraint dynamics.
    ! MJAPI void mj_invConstraint(const mjModel* m, mjData* d)
    subroutine mj_invConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_invConstraint

    !! Compare forward and inverse dynamics, save results in fwdinv.
    ! MJAPI void mj_compareFwdInv(const mjModel* m, mjData* d)
    subroutine mj_compareFwdInv(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_compareFwdInv


    !---------------------------------- Sub components ------------------------------------------------

    !! Evaluate position-dependent sensors.
    ! MJAPI void mj_sensorPos(const mjModel* m, mjData* d)
    subroutine mj_sensorPos(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_sensorPos

    !! Evaluate velocity-dependent sensors.
    ! MJAPI void mj_sensorVel(const mjModel* m, mjData* d)
    subroutine mj_sensorVel(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_sensorVel

    !! Evaluate acceleration and force-dependent sensors.
    ! MJAPI void mj_sensorAcc(const mjModel* m, mjData* d)
    subroutine mj_sensorAcc(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_sensorAcc

    !! Evaluate position-dependent energy (potential).
    ! MJAPI void mj_energyPos(const mjModel* m, mjData* d)
    subroutine mj_energyPos(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_energyPos

    !! Evaluate velocity-dependent energy (kinetic).
    ! MJAPI void mj_energyVel(const mjModel* m, mjData* d)
    subroutine mj_energyVel(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_energyVel

    !! Check qpos, reset if any element is too big or nan.
    ! MJAPI void mj_checkPos(const mjModel* m, mjData* d)
    subroutine mj_checkPos(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_checkPos

    !! Check qvel, reset if any element is too big or nan.
    ! MJAPI void mj_checkVel(const mjModel* m, mjData* d)
    subroutine mj_checkVel(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_checkVel

    !! Check qacc, reset if any element is too big or nan.
    ! MJAPI void mj_checkAcc(const mjModel* m, mjData* d)
    subroutine mj_checkAcc(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_checkAcc

    !! Run forward kinematics.
    ! MJAPI void mj_kinematics(const mjModel* m, mjData* d)
    subroutine mj_kinematics(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_kinematics

    !! Map inertias and motion dofs to global frame centered at CoM.
    ! MJAPI void mj_comPos(const mjModel* m, mjData* d)
    subroutine mj_comPos(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_comPos

    !! Compute camera and light positions and orientations.
    ! MJAPI void mj_camlight(const mjModel* m, mjData* d)
    subroutine mj_camlight(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_camlight

    !! Compute tendon lengths, velocities and moment arms.
    ! MJAPI void mj_tendon(const mjModel* m, mjData* d)
    subroutine mj_tendon(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_tendon

    !! Compute actuator transmission lengths and moments.
    ! MJAPI void mj_transmission(const mjModel* m, mjData* d)
    subroutine mj_transmission(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_transmission

    !! Run composite rigid body inertia algorithm (CRB).
    ! MJAPI void mj_crb(const mjModel* m, mjData* d)
    subroutine mj_crb(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_crb

    !! Compute sparse L'*D*L factorizaton of inertia matrix.
    ! MJAPI void mj_factorM(const mjModel* m, mjData* d)
    subroutine mj_factorM(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_factorM

    !! Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y
    ! MJAPI void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n)
    subroutine mj_solveM(m, d, x, y, n) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: x
      real(mjtNum), value, intent(in)   :: y
      integer(c_int), value, intent(in) :: n
    end subroutine mj_solveM

    !! Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
    ! MJAPI void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n)
    subroutine mj_solveM2(m, d, x, y, n) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: x
      real(mjtNum), value, intent(in)   :: y
      integer(c_int), value, intent(in) :: n
    end subroutine mj_solveM2

    !! Compute cvel, cdof_dot.
    ! MJAPI void mj_comVel(const mjModel* m, mjData* d)
    subroutine mj_comVel(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_comVel

    !! Compute qfrc_passive from spring-dampers, viscosity and density.
    ! MJAPI void mj_passive(const mjModel* m, mjData* d)
    subroutine mj_passive(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_passive

    !! subtree linear velocity and angular momentum
    ! MJAPI void mj_subtreeVel(const mjModel* m, mjData* d)
    subroutine mj_subtreeVel(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_subtreeVel

    !! RNE: compute M(qpos)*qacc + C(qpos,qvel) flg_acc=0 removes inertial term.
    ! MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result)
    subroutine mj_rne(m, d, flg_acc, result) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: result
      integer(c_int), value, intent(in) :: flg_acc
    end subroutine mj_rne

    !! RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
    ! MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d)
    subroutine mj_rnePostConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_rnePostConstraint

    !! Run collision detection.
    ! MJAPI void mj_collision(const mjModel* m, mjData* d)
    subroutine mj_collision(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_collision

    !! Construct constraints.
    ! MJAPI void mj_makeConstraint(const mjModel* m, mjData* d)
    subroutine mj_makeConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_makeConstraint

    !! Compute inverse constaint inertia efc_AR.
    ! MJAPI void mj_projectConstraint(const mjModel* m, mjData* d)
    subroutine mj_projectConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_projectConstraint

    !! Compute efc_vel, efc_aref.
    ! MJAPI void mj_referenceConstraint(const mjModel* m, mjData* d)
    subroutine mj_referenceConstraint(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
    end subroutine mj_referenceConstraint

    !! Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
    !! If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
    ! MJAPI void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar, mjtNum cost[1], int flg_coneHessian)
    subroutine mj_constraintUpdate(m, d, jar, cost, flg_coneHessian) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), value, intent(in)   :: jar
      real(mjtNum), intent(inout)       :: cost(1)
      integer(c_int), value, intent(in) :: flg_coneHessian
    end subroutine mj_constraintUpdate


    !---------------------------------- Support -------------------------------------------------------

    !! Add contact to d->contact list return 0 if success 1 if buffer full.
    ! MJAPI int mj_addContact(const mjModel* m, mjData* d, const mjContact* con)
    integer(c_int) function mj_addContact(m, d, con) bind(c)
      import :: c_int, mjModel, mjData, mjContact
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjContact), value, intent(in)  :: con
    end function mj_addContact

    !! Determine type of friction cone.
    ! MJAPI int mj_isPyramidal(const mjModel* m)
    integer(c_int) function mj_isPyramidal(m) bind(c)
      import :: c_int, mjModel
      type(mjModel), value, intent(in)    :: m
    end function mj_isPyramidal

    !! Determine type of constraint Jacobian.
    ! MJAPI int mj_isSparse(const mjModel* m)
    integer(c_int) function mj_isSparse(m) bind(c)
      import :: c_int, mjModel
      type(mjModel), value, intent(in)    :: m
    end function mj_isSparse

    !! Determine type of solver (PGS is dual, CG and Newton are primal).
    ! MJAPI int mj_isDual(const mjModel* m)
    integer(c_int) function mj_isDual(m) bind(c)
      import :: c_int, mjModel
      type(mjModel), value, intent(in)    :: m
    end function mj_isDual

    !! Multiply dense or sparse constraint Jacobian by vector.
    ! MJAPI void mj_mulJacVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec)
    subroutine mj_mulJacVec(m, d, res, vec) bind(c)
      import :: mjModel, mjData, mjtNum
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: res
      real(mjtNum), value, intent(in)   :: vec
    end subroutine mj_mulJacVec

    !! Multiply dense or sparse constraint Jacobian transpose by vector.
    ! MJAPI void mj_mulJacTVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec)
    subroutine mj_mulJacTVec(m, d, res, vec) bind(c)
      import :: mjModel, mjData, mjtNum
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: res
      real(mjtNum), value, intent(in)   :: vec
    end subroutine mj_mulJacTVec

    !! Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.
    ! MJAPI void mj_jac(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body)
    subroutine mj_jac(m, d, jacp, jacr, point, body) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: jacp, jacr
      real(mjtNum), intent(in)          :: point(3)
      integer(c_int), value, intent(in) :: body
    end subroutine mj_jac

    !! Compute body frame end-effector Jacobian.
    ! MJAPI void mj_jacBody(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body)
    subroutine mj_jacBody(m, d, jacp, jacr, body) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: jacp, jacr
      integer(c_int), value, intent(in) :: body
    end subroutine mj_jacBody

    !! Compute body center-of-mass end-effector Jacobian.
    ! MJAPI void mj_jacBodyCom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body)
    subroutine mj_jacBodyCom(m, d, jacp, jacr, body) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: jacp, jacr
      integer(c_int), value, intent(in) :: body
    end subroutine mj_jacBodyCom

    !! Compute geom end-effector Jacobian.
    ! MJAPI void mj_jacGeom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int geom)
    subroutine mj_jacGeom(m, d, jacp, jacr, geom) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: jacp, jacr
      integer(c_int), value, intent(in) :: geom
    end subroutine mj_jacGeom

    !! Compute site end-effector Jacobian.
    ! MJAPI void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site)
    subroutine mj_jacSite(m, d, jacp, jacr, site) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: jacp, jacr
      integer(c_int), value, intent(in) :: site
    end subroutine mj_jacSite

    !! Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
    ! MJAPI void mj_jacPointAxis(const mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis, const mjtNum point[3], const mjtNum axis[3], int body)
    subroutine mj_jacPointAxis(m, d, jacPoint, jacAxis, point, axis, body) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: jacPoint, jacAxis
      real(mjtNum), intent(in)          :: point(3), axis(3)
      integer(c_int), value, intent(in) :: body
    end subroutine mj_jacPointAxis

    !! Get id of object with specified name, return -1 if not found type is mjtObj.
    ! MJAPI int mj_name2id(const mjModel* m, int type, const char* name)
    integer(c_int) function mj_name2id(m, type, name) bind(c)
      import :: c_int, mjModel, c_char
      type(mjModel), intent(in)         :: m !type(mjModel), value, intent(in)  :: m
      integer(c_int), value, intent(in) :: type
      character(c_char), intent(in)     :: name(*)
    end function mj_name2id

    !! Get name of object with specified id, return 0 if invalid type or id type is mjtObj.
    ! MJAPI const char* mj_id2name(const mjModel* m, int type, int id)
    type(c_const_char_ptr) function mj_id2name(m, type, id) bind(c)
      import :: c_const_char_ptr, c_int, mjModel
      type(mjModel), value, intent(in)  :: m
      integer(c_int), value, intent(in) :: type, id
    end function mj_id2name

    !! Convert sparse inertia matrix M into full (i.e. dense) matrix.
    ! MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M)
    subroutine mj_fullM(m, dst, MM) bind(c)
      import :: mjModel, mjtNum
      type(mjModel), value, intent(in)  :: m
      real(mjtNum), intent(inout)       :: dst
      real(mjtNum), value, intent(in)   :: MM
    end subroutine mj_fullM

    !! Multiply vector by inertia matrix.
    ! MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec)
    subroutine mj_mulM(m, d, res, vec) bind(c)
      import :: mjModel, mjData, mjtNum
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: res
      real(mjtNum), value, intent(in)   :: vec
    end subroutine mj_mulM

    !! Multiply vector by (inertia matrix)^(1/2).
    ! MJAPI void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec)
    subroutine mj_mulM2(m, d, res, vec) bind(c)
      import :: mjModel, mjData, mjtNum
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: res
      real(mjtNum), value, intent(in)   :: vec
    end subroutine mj_mulM2

    !! Add inertia matrix to destination matrix.
    !! Destination can be sparse uncompressed, or dense when all int* are NULL
    ! MJAPI void mj_addM(const mjModel* m, mjData* d, mjtNum* dst, int* rownnz, int* rowadr, int* colind)
    subroutine mj_addM(m, d, dst, rownnz, rowadr, colind) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: dst
      integer(c_int), intent(inout)     :: rownnz, rowadr, colind
    end subroutine mj_addM

    !! Apply cartesian force and torque (outside xfrc_applied mechanism).
    ! MJAPI void mj_applyFT(const mjModel* m, mjData* d, const mjtNum force[3], const mjtNum torque[3], const mjtNum point[3], int body, mjtNum* qfrc_target)
    subroutine mj_applyFT(m, d, force, torque, point, body, qfrc_target) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: qfrc_target
      real(mjtNum), intent(in)          :: force(3), torque(3), point(3)
      integer(c_int), value, intent(in) :: body
    end subroutine mj_applyFT

    !! Compute object 6D velocity in object-centered frame, world/local orientation.
    ! MJAPI void mj_objectVelocity(const mjModel* m, const mjData* d, int objtype, int objid, mjtNum res[6], int flg_local)
    subroutine mj_objectVelocity(m, d, objtype, objid, res, flg_local) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: res(6)
      integer(c_int), value, intent(in) :: objtype, objid, flg_local
    end subroutine mj_objectVelocity

    !! Compute object 6D acceleration in object-centered frame, world/local orientation.
    ! MJAPI void mj_objectAcceleration(const mjModel* m, const mjData* d, int objtype, int objid, mjtNum res[6], int flg_local)
    subroutine mj_objectAcceleration(m, d, objtype, objid, res, flg_local) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: res(6)
      integer(c_int), value, intent(in) :: objtype, objid, flg_local
    end subroutine mj_objectAcceleration

    !! Extract 6D force:torque given contact id, in the contact frame.
    ! MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6])
    subroutine mj_contactForce(m, d, id, result) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)  :: m
      type(mjData), value, intent(in)   :: d
      real(mjtNum), intent(inout)       :: result(6)
      integer(c_int), value, intent(in) :: id
    end subroutine mj_contactForce

    !! Compute velocity by finite-differencing two positions.
    ! MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt, const mjtNum* qpos1, const mjtNum* qpos2)
    subroutine mj_differentiatePos(m, qvel, dt, qpos1, qpos2) bind(c)
      import :: mjModel, mjtNum
      type(mjModel), value, intent(in)  :: m
      real(mjtNum), intent(inout)       :: qvel
      real(mjtNum), value, intent(in)   :: dt, qpos1, qpos2
    end subroutine mj_differentiatePos

    !! Integrate position with given velocity.
    ! MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt)
    subroutine mj_integratePos(m, qpos, qvel, dt) bind(c)
      import :: mjModel, mjtNum
      type(mjModel), value, intent(in)  :: m
      real(mjtNum), intent(inout)       :: qpos
      real(mjtNum), value, intent(in)   :: qvel, dt
    end subroutine mj_integratePos

    !! Normalize all quaternions in qpos-type vector.
    ! MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos)
    subroutine mj_normalizeQuat(m, qpos) bind(c)
      import :: mjModel, mjtNum
      type(mjModel), value, intent(in)  :: m
      real(mjtNum), intent(inout)       :: qpos
    end subroutine mj_normalizeQuat

    !! Map from body local to global Cartesian coordinates.
    ! MJAPI void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9], const mjtNum pos[3], const mjtNum quat[4], int body, mjtByte sameframe)
    subroutine mj_local2Global(d, xpos, xmat, pos, quat, body, sameframe) bind(c)
      import :: mjData, mjtNum, c_int, mjtByte
      type(mjData), intent(inout)       :: d
      real(mjtNum), intent(inout)       :: xpos(3), xmat(9)
      real(mjtNum), intent(in)          :: pos(3), quat(4)
      integer(c_int), value, intent(in) :: body
      integer(mjtByte), value, intent(in) :: sameframe
    end subroutine mj_local2Global

    !! Sum all body masses.
    ! MJAPI mjtNum mj_getTotalmass(const mjModel* m)
    real(mjtNum) function mj_getTotalmass(m) bind(c)
      import :: mjtNum, mjModel
      type(mjModel), value, intent(in)  :: m
    end function mj_getTotalmass

    !! Scale body masses and inertias to achieve specified total mass.
    ! MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass)
    subroutine mj_setTotalmass(m, newmass) bind(c)
      import :: mjtNum, mjModel
      type(mjModel), intent(inout)      :: m
      real(mjtNum), value, intent(in)   :: newmass
    end subroutine mj_setTotalmass

    !! Return version number: 1.0.2 is encoded as 102.
    ! MJAPI int mj_version(void)
    integer(c_int) function mj_version() bind(c)
      import :: c_int
    end function mj_version

    !! Return the current version of MuJoCo as a null-terminated string.
    ! MJAPI const char* mj_versionString()
    type(c_const_char_ptr) function mj_versionString() bind(c)
      import :: c_const_char_ptr
    end function mj_versionString

    !---------------------------------- Ray collisions ------------------------------------------------

    !! Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
    !! Return distance (x) to nearest surface, or -1 if no intersection and output geomid.
    !! geomgroup, flg_static are as in mjvOption geomgroup==NULL skips group exclusion.
    ! MJAPI mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum pnt[3], const mjtNum vec[3], const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude, int geomid[1])
    real(mjtNum) function mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid) bind(c)
      import :: mjModel, mjData, mjtNum, mjtByte, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      real(mjtNum), intent(in)            :: pnt(3), vec(3)
      integer(mjtByte), value, intent(in) :: geomgroup
      integer(mjtByte), intent(inout)     :: flg_static
      integer(c_int), value, intent(in)   :: bodyexclude
      integer(c_int), intent(in)          :: geomid(1)
    end function mj_ray

    !! Interect ray with hfield, return nearest distance or -1 if no intersection.
    ! MJAPI mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid, const mjtNum pnt[3], const mjtNum vec[3])
    real(mjtNum) function mj_rayHfield(m, d, geomid, pnt, vec) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      integer(c_int), value, intent(in)   :: geomid
      real(mjtNum), intent(in)            :: pnt(3), vec(3)
    end function mj_rayHfield
    !! Interect ray with mesh, return nearest distance or -1 if no intersection.
    ! MJAPI mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid, const mjtNum pnt[3], const mjtNum vec[3])
    real(mjtNum) function mj_rayMesh(m, d, geomid, pnt, vec) bind(c)
      import :: mjModel, mjData, mjtNum, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      integer(c_int), value, intent(in)   :: geomid
      real(mjtNum), intent(in)            :: pnt(3), vec(3)
    end function mj_rayMesh

    !! Interect ray with pure geom, return nearest distance or -1 if no intersection.
    ! MJAPI mjtNum mju_rayGeom(const mjtNum pos[3], const mjtNum mat[9], const mjtNum size[3], const mjtNum pnt[3], const mjtNum vec[3], int geomtype)
    real(mjtNum) function mju_rayGeom(pos, mat, size, pnt, vec, geomtype) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(in)            :: pos(3), mat(3), size(3), pnt(3), vec(3)
      integer(c_int), value, intent(in)   :: geomtype
    end function mju_rayGeom
    !! Interect ray with skin, return nearest distance or -1 if no intersection,
    !! and also output nearest vertex id.
    ! MJAPI mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert, const mjtNum pnt[3], const mjtNum vec[3], int vertid[1])
    real(mjtNum) function mju_raySkin(nface, nvert, face, vert, pnt, vec, vertid) bind(c)
      import :: mjtNum, c_int, c_float
      real(mjtNum), intent(in)            :: pnt(3), vec(3)
      integer(c_int), value, intent(in)   :: nface, nvert, face
      integer(c_int), intent(in)          :: vertid(1)
      real(c_float), value, intent(in)    :: vert
    end function mju_raySkin


    !---------------------------------- Interaction ---------------------------------------------------

    !! Set default camera.
    ! MJAPI void mjv_defaultCamera(mjvCamera* cam)
    subroutine mjv_defaultCamera(cam) bind(C, name="mjv_defaultCamera")
      import :: c_ptr
      type(c_ptr), intent(in), value :: cam
    end subroutine mjv_defaultCamera

    !! Set default perturbation.
    ! MJAPI void mjv_defaultPerturb(mjvPerturb* pert)
    subroutine mjv_defaultPerturb(pert) bind(c)
      import :: mjvPerturb
      type(mjvPerturb), intent(inout)  :: pert
    end subroutine mjv_defaultPerturb

    !! Transform pose from room to model space.
    ! MJAPI void mjv_room2model(mjtNum modelpos[3], mjtNum modelquat[4], const mjtNum roompos[3], const mjtNum roomquat[4], const mjvScene* scn)
    subroutine mjv_room2model(modelpos, modelquat, roompos, roomquat, scn) bind(c, name="mjv_room2model")
      import :: mjtNum, mjvScene
      real(mjtNum), intent(inout)       :: modelpos(3), modelquat(4)
      real(mjtNum), intent(in)          :: roompos(3), roomquat(4)
      type(mjvScene), intent(in)        :: scn
    end subroutine mjv_room2model

    ! !! Transform pose from model to room space.
    ! MJAPI void mjv_model2room(mjtNum roompos[3], mjtNum roomquat[4], const mjtNum modelpos[3],
    !                           const mjtNum modelquat[4], const mjvScene* scn)
    subroutine mjv_model2room(roompos, roomquat, modelpos, modelquat, scn) bind(c, name="mjv_model2room")
      import :: mjtNum, mjvScene
      real(mjtNum), intent(inout)       :: roompos(3), roomquat(4)
      real(mjtNum), intent(in)          :: modelpos(3), modelquat(4)
      type(mjvScene), intent(in)        :: scn
    end subroutine mjv_model2room

    ! !! Get camera info in model space average left and right OpenGL cameras.
    ! MJAPI void mjv_cameraInModel(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
    !                             const mjvScene* scn)
    subroutine mjv_cameraInModel(headpos, forward, up, scn) bind(c)
      import :: mjtNum, mjvScene
      real(mjtNum), intent(inout)       :: headpos(3), forward(3), up(3)
      type(mjvScene), intent(in)        :: scn
    end subroutine mjv_cameraInModel

    ! !! Get camera info in room space average left and right OpenGL cameras.
    ! MJAPI void mjv_cameraInRoom(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
    !                             const mjvScene* scn)
    subroutine mjv_cameraInRoom(headpos, forward, up, scn) bind(c)
      import :: mjtNum, mjvScene
      type(mjvScene), value, intent(in) :: scn
      real(mjtNum), intent(inout)       :: headpos(3), forward(3), up(3)
    end subroutine mjv_cameraInRoom

    ! !! Get frustum height at unit distance from camera average left and right OpenGL cameras.
    ! MJAPI mjtNum mjv_frustumHeight(const mjvScene* scn)
    real(mjtNum) function mjv_frustumHeight(scn) bind(c)
      import :: mjtNum, mjvScene
      type(mjvScene), intent(in)        :: scn
    end function mjv_frustumHeight

    ! !! Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y).
    ! MJAPI void mjv_alignToCamera(mjtNum res[3], const mjtNum vec[3], const mjtNum forward[3])
    subroutine mjv_alignToCamera(res, vec, forward) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)       :: res(3)
      real(mjtNum), intent(in)          :: vec(3), forward(3)
    end subroutine mjv_alignToCamera

    ! !! Move camera with mouse action is mjtMouse.
    ! MJAPI void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
    !                           const mjvScene* scn, mjvCamera* cam)
    subroutine mjv_moveCamera(m, action, reldx, reldy, scn, cam) bind(c, name="mjv_moveCamera")
      import :: c_ptr, mjtNum, mjvScene, mjvCamera, c_int
      type(c_ptr), intent(in), value    :: m
      integer(c_int), value, intent(in) :: action
      real(mjtNum), value, intent(in)   :: reldx, reldy
      type(c_ptr), intent(in), value    :: scn
      type(c_ptr), value                :: cam
    end subroutine mjv_moveCamera

    ! !! Move perturb object with mouse action is mjtMouse.
    ! MJAPI void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
    !                           mjtNum reldy, const mjvScene* scn, mjvPerturb* pert)
    subroutine mjv_movePerturb(m, d, action, reldx, reldy, scn, pert) bind(c)
      import :: mjtNum, mjvScene, mjvPerturb, mjModel, mjData, c_int
      type(mjvScene), value, intent(in)   :: scn
      type(mjvPerturb), intent(inout)     :: pert
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      real(mjtNum), value, intent(in)     :: reldx, reldy
      integer(c_int), value, intent(in)   :: action
    end subroutine mjv_movePerturb

    ! !! Move model with mouse action is mjtMouse.
    ! MJAPI void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
    !                         const mjtNum roomup[3], mjvScene* scn)
    subroutine mjv_moveModel(m, action, reldx, reldy, roomup, scn) bind(c)
      import :: mjtNum, mjvScene, mjModel, c_int
      type(mjModel), value, intent(in)    :: m
      integer(c_int), value, intent(in)   :: action
      real(mjtNum), value, intent(in)     :: reldx, reldy
      real(mjtNum), intent(in)            :: roomup(3)
      type(mjvScene), value, intent(in)   :: scn
    end subroutine mjv_moveModel

    ! !! Copy perturb pos,quat from selected body set scale for perturbation.
    ! MJAPI void mjv_initPerturb(const mjModel* m, const mjData* d,
    !                           const mjvScene* scn, mjvPerturb* pert)
    subroutine mjv_initPerturb(m, d, scn, pert) bind(c)
      import :: mjvScene, mjvPerturb, mjModel, mjData
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      type(mjvScene), value, intent(in)   :: scn
      type(mjvPerturb), intent(inout)     :: pert
    end subroutine mjv_initPerturb

    ! !! Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
    ! !! Write d->qpos only if flg_paused and subtree root for selected body has free joint.
    ! MJAPI void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
    !                                 int flg_paused)
    subroutine mjv_applyPerturbPose(m, d, pert, flg_paused) bind(c)
      import :: mjvPerturb, mjModel, mjData, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      integer(c_int), value, intent(in)   :: flg_paused
      type(mjvPerturb), value, intent(in) :: pert
    end subroutine mjv_applyPerturbPose

    ! !! Set perturb force,torque in d->xfrc_applied, if selected body is dynamic.
    ! MJAPI void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert)
    subroutine mjv_applyPerturbForce(m, d, pert) bind(c)
      import :: mjvPerturb, mjModel, mjData
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjvPerturb), value, intent(in) :: pert
    end subroutine mjv_applyPerturbForce

    ! !! Return the average of two OpenGL cameras.
    ! MJAPI mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2)
    type(mjvGLCamera) function mjv_averageCamera(cam1, cam2) bind(c, name="mjv_averageCamera")
      import :: mjvGLCamera
      type(mjvGLCamera),  intent(in) :: cam1, cam2
    end function mjv_averageCamera

    ! !! Select geom or skin with mouse, return bodyid -1: none selected.
    ! MJAPI int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
    !                     mjtNum aspectratio, mjtNum relx, mjtNum rely,
    !                     const mjvScene* scn, mjtNum selpnt[3], int geomid[1], int skinid[1])
    integer(c_int) function mjv_select(m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid) bind(c)
      import :: mjModel, mjData, mjvOption, mjtNum, mjvScene, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), value, intent(in)     :: d
      type(mjvOption), value, intent(in)  :: vopt
      real(mjtNum), value, intent(in)     :: aspectratio, relx, rely
      type(mjvScene), value, intent(in)   :: scn
      real(mjtNum), intent(in)            :: selpnt 
      integer(c_int), intent(in)          :: geomid(1), skinid(1)
    end function mjv_select


    !---------------------------------- Visualization -------------------------------------------------

    !! Set default visualization options.
    ! MJAPI void mjv_defaultOption(mjvOption* opt)
    subroutine mjv_defaultOption(opt) bind(C, name="mjv_defaultOption")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: opt
    end subroutine mjv_defaultOption

    !! Set default figure.
    ! MJAPI void mjv_defaultFigure(mjvFigure* fig)
    subroutine mjv_defaultFigure(fig) bind(c)
      import :: mjvFigure
      type(mjvFigure), intent(inout)       :: fig
    end subroutine mjv_defaultFigure

    !! Initialize given geom fields when not NULL, set the rest to their default values.
    ! MJAPI void mjv_initGeom(mjvGeom* geom, int type, const mjtNum size[3],
                            ! const mjtNum pos[3], const mjtNum mat[9], const float rgba[4])
    subroutine mjv_initGeom(geom, type, size_, pos, mat, rgba_) bind(c, name="mjv_initGeom")
      import :: mjvGeom, c_int, mjtNum, c_float
      type(mjvGeom), intent(inout)        :: geom
      integer(c_int), value, intent(in)   :: type
      real(mjtNum), intent(in)            :: size_(3), pos(3), mat(9) 
      real(c_float), intent(in)           :: rgba_(4)
    end subroutine mjv_initGeom
    
    !! Set (type, size, pos, mat) for connector-type geom between given points.
    !! Assume that mjv_initGeom was already called to set all other properties.
    ! MJAPI void mjv_makeConnector(mjvGeom* geom, int type, mjtNum width,
                                ! mjtNum a0, mjtNum a1, mjtNum a2,
                                ! mjtNum b0, mjtNum b1, mjtNum b2)
    subroutine mjv_makeConnector(geom, type, width, a0, a1, a2, b0, b1, b2) bind(c)
      import :: mjvGeom, c_int, mjtNum
      type(mjvGeom), intent(inout)        :: geom
      integer(c_int), value, intent(in)   :: type
      real(mjtNum), value, intent(in)     :: width, a0, a1, a2, b0, b1, b2
    end subroutine mjv_makeConnector

    !! Set default abstract scene.
    ! MJAPI void mjv_defaultScene(mjvScene* scn)
    subroutine mjv_defaultScene(scn) bind(c, name="mjv_defaultScene")
      import :: c_ptr
      type(c_ptr), intent(in), value :: scn
    end subroutine mjv_defaultScene

    !! Allocate resources in abstract scene.
    ! MJAPI void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom)
    subroutine mjv_makeScene(m, scn, maxgeom) bind(c, name="mjv_makeScene")
      import :: c_ptr, C_INT
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: scn
      integer(C_INT), value, intent(IN) :: maxgeom
    end subroutine mjv_makeScene

    !! Free abstract scene.
    ! MJAPI void mjv_freeScene(mjvScene* scn)
    subroutine mjv_freeScene(scn) bind(c, name="mjv_freeScene")
      import :: c_ptr
      type(c_ptr), value                 :: scn
    end subroutine mjv_freeScene

    !! Update entire scene given model state.
    ! MJAPI void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                              ! const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn)
    subroutine mjv_updateScene(m, d, opt, pert, cam, catmask, scn) bind(c, name="mjv_updateScene")
      import :: c_ptr, c_int
      type(c_ptr), intent(IN), value    :: m
      type(c_ptr), value                :: d
      type(c_ptr), intent(IN), value    :: opt
      type(c_ptr), intent(IN), value    :: pert
      type(c_ptr), value                :: cam
      integer(C_INT), value, intent(IN) :: catmask
      type(c_ptr), value                :: scn
    end subroutine mjv_updateScene

    !! Add geoms from selected categories.
    ! MJAPI void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
    !                         const mjvPerturb* pert, int catmask, mjvScene* scn)
    subroutine mjv_addGeoms(m, d, opt, pert, cammask, scn) bind(c)
      import :: mjvPerturb, mjModel, mjData, mjvOption, mjvCamera, mjvScene, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjvOption), value, intent(in)  :: opt
      type(mjvPerturb), value, intent(in) :: pert
      integer(c_int), value, intent(in)   :: cammask
      type(mjvScene), intent(inout)       :: scn
    end subroutine mjv_addGeoms
    !! Make list of lights.
    ! MJAPI void mjv_makeLights(const mjModel* m, mjData* d, mjvScene* scn)
    subroutine mjv_makeLights(m, d, scn) bind(c)
      import :: mjModel, mjData, mjvScene
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjvScene), intent(inout)       :: scn
    end subroutine mjv_makeLights

    !! Update camera.
    ! MJAPI void mjv_updateCamera(const mjModel* m, mjData* d, mjvCamera* cam, mjvScene* scn)
    subroutine mjv_updateCamera(m, d, cam, scn) bind(c)
      import :: mjModel, mjData, mjvScene, mjvCamera
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjvCamera), intent(inout)      :: cam
      type(mjvScene), intent(inout)       :: scn
    end subroutine mjv_updateCamera

    !! Update skins.
    ! MJAPI void mjv_updateSkin(const mjModel* m, mjData* d, mjvScene* scn)
    subroutine mjv_updateSkin(m, d, scn) bind(c)
      import :: mjModel, mjData, mjvScene
      type(mjModel), value, intent(in)    :: m
      type(mjData), intent(inout)         :: d
      type(mjvScene), intent(inout)       :: scn
    end subroutine mjv_updateSkin


    !---------------------------------- OpenGL rendering ----------------------------------------------

    !! Set default mjrContext.
    ! MJAPI void mjr_defaultContext(mjrContext* con)
    subroutine mjr_defaultContext(con) bind(c, name="mjr_defaultContext")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: con
    end subroutine mjr_defaultContext

    !! Allocate resources in custom OpenGL context fontscale is mjtFontScale.
    ! MJAPI void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale)
    subroutine mjr_makeContext(m, con, fontscale) bind(c, name="mjr_makeContext")
      import :: C_INT, c_ptr
      implicit none
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: con
      integer(C_INT), value, intent(IN) :: fontscale
    end subroutine mjr_makeContext

    !! Change font of existing context.
    ! MJAPI void mjr_changeFont(int fontscale, mjrContext* con)
    subroutine mjr_changeFont(fontscale, con) bind(c)
      import :: mjrContext, c_int
      integer(c_int), value, intent(in)   :: fontscale
      type(mjrContext), intent(inout)     :: con
    end subroutine mjr_changeFont

    !! Add Aux buffer with given index to context free previous Aux buffer.
    ! MJAPI void mjr_addAux(int index, int width, int height, int samples, mjrContext* con)
    subroutine mjr_addAux(index, width, height, samples, con) bind(c)
      import :: mjrContext, c_int
      integer(c_int), value, intent(in)   :: index, width, height, samples
      type(mjrContext), intent(inout)     :: con
    end subroutine mjr_addAux

    !! Free resources in custom OpenGL context, set to default.
    ! MJAPI void mjr_freeContext(mjrContext* con)
    subroutine mjr_freeContext(con) bind(c, name="mjr_freeContext")
      import :: c_ptr
      type(c_ptr), value                  :: con
    end subroutine mjr_freeContext

    !! Upload texture to GPU, overwriting previous upload if any.
    ! MJAPI void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid)
    subroutine mjr_uploadTexture(m, con, texid) bind(c)
      import :: mjModel,  mjrContext, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjrContext), value, intent(in) :: con
      integer(c_int), value, intent(in)   :: texid
    end subroutine mjr_uploadTexture

    !! Upload mesh to GPU, overwriting previous upload if any.
    ! MJAPI void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid)
    subroutine mjr_uploadMesh(m, con, meshid) bind(c)
      import :: mjModel,  mjrContext, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjrContext), value, intent(in) :: con
      integer(c_int), value, intent(in)   :: meshid
    end subroutine mjr_uploadMesh

    !! Upload height field to GPU, overwriting previous upload if any.
    ! MJAPI void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid)
    subroutine mjr_uploadHField(m, con, hfieldid) bind(c)
      import :: mjModel,  mjrContext, c_int
      type(mjModel), value, intent(in)    :: m
      type(mjrContext), value, intent(in) :: con
      integer(c_int), value, intent(in)   :: hfieldid
    end subroutine mjr_uploadHField

    !! Make con->currentBuffer current again.
    ! MJAPI void mjr_restoreBuffer(const mjrContext* con)
    subroutine mjr_restoreBuffer(con) bind(c)
      import :: mjrContext
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_restoreBuffer

    !! Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
    !! If only one buffer is available, set that buffer and ignore framebuffer argument.
    ! MJAPI void mjr_setBuffer(int framebuffer, mjrContext* con)
    subroutine mjr_setBuffer(framebuffer, con) bind(c)
      import :: mjrContext, c_int
      integer(c_int), value, intent(in)   :: framebuffer
      type(mjrContext), intent(inout)     :: con
    end subroutine mjr_setBuffer

    !! Read pixels from current OpenGL framebuffer to client buffer.
    !! Viewport is in OpenGL framebuffer client buffer starts at (0,0).
    ! MJAPI void mjr_readPixels(unsigned char* rgb, float* depth,
    !                           mjrRect viewport, const mjrContext* con)
    subroutine mjr_readPixels(rgb, depth, viewport, con) bind(c)
      import :: c_signed_char, c_float, mjrRect, mjrContext
      character(c_signed_char), intent(inout)  :: rgb(*)
      real(c_float), intent(inout)        :: depth
      type(mjrRect), intent(in)            :: viewport
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_readPixels

    !! Draw pixels from client buffer to current OpenGL framebuffer.
    !! Viewport is in OpenGL framebuffer client buffer starts at (0,0).
    ! MJAPI void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                              ! mjrRect viewport, const mjrContext* con)
    subroutine mjr_drawPixels(rgb, depth, viewport, con) bind(c)
      import :: c_signed_char, c_float, mjrRect, mjrContext
      character(c_signed_char), intent(in)  :: rgb(*)
      real(c_float), value, intent(in)    :: depth
      type(mjrRect), intent(in)            :: viewport
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_drawPixels

    !! Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
    !! If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR.
    ! MJAPI void mjr_blitBuffer(mjrRect src, mjrRect dst,
                              ! int  flg_color, int flg_depth, const mjrContext* con)
    subroutine mjr_blitBuffer(src, dst, flg_color, flg_depth, con) bind(c)
      import :: mjrRect, mjrContext, c_int
      type(mjrRect), intent(inout)         :: src, dst
      integer(c_int), value, intent(in)   :: flg_color, flg_depth
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_blitBuffer

    !! Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
    ! MJAPI void mjr_setAux(int index, const mjrContext* con)
    subroutine mjr_setAux(index, con) bind(c)
      import :: mjrContext, c_int
      integer(c_int), value, intent(in)   :: index
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_setAux

    !! Blit from Aux buffer to con->currentBuffer.
    ! MJAPI void mjr_blitAux(int index, mjrRect src, int left, int bottom,
                          ! const mjrContext* con)
    subroutine mjr_blitAux(index, src, left, bottom, con) bind(c)
      import :: mjrRect, mjrContext, c_int
      integer(c_int), value, intent(in)   :: index, left, bottom
      type(mjrRect), intent(inout)         :: src 
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_blitAux

    !! Draw text at (x,y) in relative coordinates font is mjtFont.
    ! MJAPI void mjr_text(int font, const char* txt, const mjrContext* con,
                        ! float x, float y, float r, float g, float b)
    subroutine mjr_text(font, txt, con, x, y, r, g, b) bind(c)
      import :: c_char, c_float, c_int, mjrContext
      integer(c_int), value, intent(in)   :: font
      character(c_char), intent(in)       :: txt(*)
      type(mjrContext), value, intent(in) :: con
      real(c_float), value, intent(in)    :: x, y, r, g, b
    end subroutine mjr_text

    !! Draw text overlay font is mjtFont gridpos is mjtGridPos.
    ! MJAPI void mjr_overlay(int font, int gridpos, mjrRect viewport,
                          ! const char* overlay, const char* overlay2, const mjrContext* con)
    subroutine mjr_overlay(font, gridpos, viewport, overlay, overlay2, con) bind(c)
      import :: c_char, c_float, c_int, mjrRect, mjrContext
      integer(c_int), value, intent(in)   :: font, gridpos
      type(mjrRect), intent(inout)        :: viewport
      character(c_char), intent(in)       :: overlay(*), overlay2(*)
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_overlay

    !! Get maximum viewport for active buffer.
    ! MJAPI mjrRect mjr_maxViewport(const mjrContext* con)
    type(mjrRect) function mjr_maxViewport(con) bind(c)
      import :: mjrRect, mjrContext
      type(mjrContext), value, intent(in) :: con
    end function mjr_maxViewport

    !! Draw rectangle.
    ! MJAPI void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a)
    subroutine mjr_rectangle(viewport, r, g, b, a) bind(c)
      import :: mjrRect, c_float
      type(mjrRect), intent(inout)        :: viewport
      real(c_float), value, intent(in)    :: r, g, b, a
    end subroutine mjr_rectangle

    !! Draw rectangle with centered text.
    ! MJAPI void mjr_label(mjrRect viewport, int font, const char* txt,
                        ! float r, float g, float b, float a, float rt, float gt, float bt,
                        ! const mjrContext* con)
    subroutine mjr_label(viewport, font, txt, r, g, b, a, rt, gt, bt, con) bind(c)
      import :: mjrRect, c_char, c_float, c_int, mjrContext
      type(mjrRect), intent(inout)        :: viewport
      integer(c_int), value, intent(in)   :: font
      character(c_char), intent(in)       :: txt(*)
      real(c_float), value, intent(in)    :: r, g, b, a, rt, gt, bt
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_label

    !! Draw 2D figure.
    ! MJAPI void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con)
    subroutine mjr_figure(viewport, fig, con) bind(c)
      import :: mjrRect, mjvFigure, mjrContext
      type(mjrRect), intent(inout)        :: viewport
      type(mjvFigure), intent(inout)      :: fig
      type(mjrContext), value, intent(in) :: con
    end subroutine mjr_figure

    !! Render 3D scene.
    ! MJAPI void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con)
    subroutine mjr_render(viewport, scn, con) bind(c, name="mjr_render")
      import :: c_ptr, mjrRect
        type(mjrRect), value, intent(IN) :: viewport
        type(c_ptr), value               :: scn
        type(c_ptr), intent(IN), value   :: con
    end subroutine mjr_render

    !! Render 3D scene. --- MY ADDITION
    ! MJAPI void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con, mjvGLCamera* cam)
    subroutine mjr_render_fortran(viewport, scn, con, cam) bind(c, name="mjr_render_fortran")
      import :: mjrRect, mjvScene, mjrContext, mjvGLCamera
      type(mjrRect), intent(inout)        :: viewport
      type(mjvScene), intent(inout)       :: scn
      type(mjrContext), intent(in)        :: con
      type(mjvGLCamera), intent(inout)    :: cam
    end subroutine mjr_render_fortran

    !! Call glFinish.
    ! MJAPI void mjr_finish(void)
    subroutine mjr_finish() bind(c)
    end subroutine mjr_finish

    !! Call glGetError and return result.
    ! MJAPI int mjr_getError(void)
    integer(c_int) function mjr_getError() bind(c)
      import :: c_int
    end function mjr_getError

    !! Find first rectangle containing mouse, -1: not found.
    ! MJAPI int mjr_findRect(int x, int y, int nrect, const mjrRect* rect)
    integer(c_int) function mjr_findRect(x, y, nrect, rect) bind(c, name="mjr_findRect")
      import :: mjrRect, c_int
      integer(c_int), value, intent(in)   :: x, y, nrect
      type(mjrRect), intent(in)           :: rect
    end function mjr_findRect


    !---------------------------------- UI framework --------------------------------------------------

    !! Get builtin UI theme spacing (ind: 0-1).
    ! MJAPI mjuiThemeSpacing mjui_themeSpacing(int ind)
    type(mjuiThemeSpacing) function mjui_themeSpacing(ind) bind(c)
      import :: mjuiThemeSpacing, c_int
      integer(c_int), value, intent(in)   :: ind
    end function mjui_themeSpacing

    !! Get builtin UI theme color (ind: 0-3).
    ! MJAPI mjuiThemeColor mjui_themeColor(int ind)
    type(mjuiThemeColor) function mjui_themeColor(ind) bind(c)
      import :: mjuiThemeColor, c_int
      integer(c_int), value, intent(in)   :: ind
    end function mjui_themeColor

    !! Add definitions to UI.
    ! MJAPI void mjui_add(mjUI* ui, const mjuiDef* def)
    subroutine mjui_add(ui, def) bind(c)
      import :: mjUI, mjuiDef
      type(mjUI), intent(inout)           :: ui
      type(mjuiDef), value, intent(in)    :: def
    end subroutine mjui_add

    !! Add definitions to UI section.
    ! MJAPI void mjui_addToSection(mjUI* ui, int sect, const mjuiDef* def)
    subroutine mjui_addToSection(ui, sect, def) bind(c)
      import :: mjUI, mjuiDef, c_int
      type(mjUI), intent(inout)           :: ui
      integer(c_int), value, intent(in)   :: sect
      type(mjuiDef), value, intent(in)    :: def
    end subroutine mjui_addToSection

    !! Compute UI sizes.
    ! MJAPI void mjui_resize(mjUI* ui, const mjrContext* con)
    subroutine mjui_resize(ui, con) bind(c)
      import :: mjUI, mjrContext
      type(mjUI), intent(inout)           :: ui
      type(mjrContext), value, intent(in) :: con
    end subroutine mjui_resize

    !! Update specific section/item -1: update all.
    ! MJAPI void mjui_update(int section, int item, const mjUI* ui,
                          ! const mjuiState* state, const mjrContext* con)
    subroutine mjui_update(section, item, ui, state, con) bind(c)
      import :: mjUI, mjuiState, mjrContext, c_int
      integer(c_int), value, intent(in)   :: section, item
      type(mjUI), value, intent(in)       :: ui
      type(mjuiState), value, intent(in)  :: state
      type(mjrContext), value, intent(in) :: con
    end subroutine mjui_update

    !! Handle UI event, return pointer to changed item, NULL if no change.
    ! MJAPI mjuiItem* mjui_event(mjUI* ui, mjuiState* state, const mjrContext* con)
    type(mjuiItem) function mjui_event(ui, state, con) bind(c)
      import :: mjuiItem, mjUI, mjuiState, mjrContext
      type(mjUI), intent(inout)           :: ui
      type(mjuiState), intent(inout)      :: state
      type(mjrContext), value, intent(in) :: con
    end function mjui_event

    !! Copy UI image to current buffer.
    ! MJAPI void mjui_render(mjUI* ui, const mjuiState* state, const mjrContext* con)
    subroutine mjui_render(ui, state, con) bind(c)
      import :: mjUI, mjuiState, mjrContext
      type(mjUI), intent(inout)           :: ui
      type(mjuiState), value, intent(in)  :: state
      type(mjrContext), value, intent(in) :: con
    end subroutine mjui_render


    !---------------------------------- Error and memory ----------------------------------------------

    !! Main error function does not return to caller.
    ! MJAPI void mju_error(const char* msg)
    subroutine mju_error(msg) bind(c)
      import :: c_char
      character(c_char), intent(in)       :: msg(*)
    end subroutine mju_error

    !! Error function with int argument msg is a printf format string.
    ! MJAPI void mju_error_i(const char* msg, int i)
    subroutine mju_error_i(msg, i) bind(c)
      import :: c_char, c_int
      character(c_char), intent(in)       :: msg(*)
      integer(c_int), value, intent(in)   :: i
    end subroutine mju_error_i

    !! Error function with string argument.
    ! MJAPI void mju_error_s(const char* msg, const char* text)
    subroutine mju_error_s(msg, text) bind(c, name="mju_error_s")
      import :: c_char
      character(c_char), intent(in)       :: msg(*)
      character(c_char), intent(in)       :: text(*)
    end subroutine mju_error_s

    !! Main warning function returns to caller.
    ! MJAPI void mju_warning(const char* msg)
    subroutine mju_warning(msg) bind(c)
      import :: c_char
      character(c_char), intent(in)       :: msg(*)
    end subroutine mju_warning

    !! Warning function with int argument.
    ! MJAPI void mju_warning_i(const char* msg, int i)
    subroutine mju_warning_i(msg, i) bind(c)
      import :: c_char, c_int
      character(c_char), intent(in)       :: msg(*)
      integer(c_int), value, intent(in)   :: i
    end subroutine mju_warning_i

    !! Warning function with string argument.
    ! MJAPI void mju_warning_s(const char* msg, const char* text)
    subroutine mju_warning_s(msg, text) bind(c)
      import :: c_char
      character(c_char), intent(in)       :: msg(*)
      character(c_char), intent(in)       :: text(*)
    end subroutine mju_warning_s

    !! Clear user error and memory handlers.
    ! MJAPI void mju_clearHandlers(void)
    subroutine mju_clearHandlers() bind(c)
    end subroutine mju_clearHandlers

    !! Allocate memory byte-align on 64; pad size to multiple of 64.
    ! MJAPI void* mju_malloc(size_t size)
    type(c_ptr) function mju_malloc(size) bind(c)
      import :: c_ptr, c_size_t
      integer(c_size_t), value, intent(in) :: size
    end function mju_malloc

    !! Free memory, using free() by default.
    ! MJAPI void mju_free(void* ptr)
    subroutine mju_free(ptr) bind(c)
      import :: c_ptr
      type(c_ptr), intent(inout)           :: ptr
    end subroutine mju_free

    !! High-level warning function: count warnings in mjData, print only the first.
    ! MJAPI void mj_warning(mjData* d, int warning, int info)
    subroutine mj_warning(d, warning, info) bind(c)
      import :: c_int, mjData
      type(mjData), intent(inout)           :: d
      integer(c_int), value, intent(in) :: warning, info
    end subroutine mj_warning

    !! Write [datetime, type: message] to MUJOCO_LOG.TXT.
    ! MJAPI void mju_writeLog(const char* type, const char* msg)
    subroutine mju_writeLog(type, msg) bind(c)
      import :: c_char
      character(c_char), intent(in)       :: type(*)
      character(c_char), intent(in)       :: msg(*)
    end subroutine mju_writeLog

    !---------------------------------- Activation ----------------------------------------------------

    ! !! Return 1 (for backward compatibility).
    ! ! MJAPI int mj_activate(const char* filename);
    ! integer(c_int) function mj_activate(filename) bind(c, name="mj_activate")
    !   import :: c_int, c_char
    !   character(c_char), intent(in)       :: filename(*)
    ! end function mj_activate

    !! Do nothing (for backward compatibility).
    !MJAPI void mj_deactivate(void);
    ! subroutine mj_deactivate() bind(c, name="mj_deactivate")
    ! end subroutine mj_deactivate

    !---------------------------------- Vector math ---------------------------------------------------

    !! Set res = 0.
    ! MJAPI void mju_zero3(mjtNum res[3])
    subroutine mju_zero3(res) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
    end subroutine mju_zero3

    !! Set res = vec.
    ! MJAPI void mju_copy3(mjtNum res[3], const mjtNum data[3])
    subroutine mju_copy3(res, data) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), value, intent(in)     :: data 
    end subroutine mju_copy3

    !! Set res = vec*scl.
    ! MJAPI void mju_scl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl)
    subroutine mju_scl3(res, vec, scl) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3)
      real(mjtNum), value, intent(in)     :: scl 
    end subroutine mju_scl3

    !! Set res = vec1 + vec2.
    ! MJAPI void mju_add3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3])
    subroutine mju_add3(res, vec1, vec2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec1(3), vec2(3) 
    end subroutine mju_add3

    !! Set res = vec1 - vec2.
    ! MJAPI void mju_sub3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3])
    subroutine mju_sub3(res, vec1, vec2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec1(3), vec2(3) 
    end subroutine mju_sub3

    !! Set res = res + vec.
    ! MJAPI void mju_addTo3(mjtNum res[3], const mjtNum vec[3])
    subroutine mju_addTo3(res, vec) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3) 
    end subroutine mju_addTo3

    !! Set res = res - vec.
    ! MJAPI void mju_subFrom3(mjtNum res[3], const mjtNum vec[3])
    subroutine mju_subFrom3(res, vec) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3) 
    end subroutine mju_subFrom3

    !! Set res = res + vec*scl.
    ! MJAPI void mju_addToScl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl)
    subroutine mju_addToScl3(res, vec, scl) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3)
      real(mjtNum), value, intent(in)     :: scl
    end subroutine mju_addToScl3

    !! Set res = vec1 + vec2*scl.
    ! MJAPI void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl)
    subroutine mju_addScl3(res, vec1, vec2, scl) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec1(3), vec2(3)
      real(mjtNum), value, intent(in)     :: scl
    end subroutine mju_addScl3

    !! Normalize vector, return length before normalization.
    ! MJAPI mjtNum mju_normalize3(mjtNum res[3])
    real(mjtNum) function mju_normalize3(res) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
    end function mju_normalize3

    !! Return vector length (without normalizing the vector).
    ! MJAPI mjtNum mju_norm3(const mjtNum vec[3])
    real(mjtNum) function mju_norm3(vec) bind(c)
      import :: mjtNum
      real(mjtNum), intent(in)            :: vec(3)
    end function mju_norm3

    !! Return dot-product of vec1 and vec2.
    ! MJAPI mjtNum mju_dot3(const mjtNum vec1[3], const mjtNum vec2[3])
    real(mjtNum) function mju_dot3(vec1, vec2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(in)            :: vec1(3), vec2(3)
    end function mju_dot3

    !! Return Cartesian distance between 3D vectors pos1 and pos2.
    ! MJAPI mjtNum mju_dist3(const mjtNum pos1[3], const mjtNum pos2[3])
    real(mjtNum) function mju_dist3(pos1, pos2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(in)            :: pos1(3), pos2(3)
    end function mju_dist3

    !! Multiply vector by 3D rotation matrix: res = mat * vec.
    ! MJAPI void mju_rotVecMat(mjtNum res[3], const mjtNum vec[3], const mjtNum mat[9])
    subroutine mju_rotVecMat(res, vec, mat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3), mat(9)
    end subroutine mju_rotVecMat

    !! Multiply vector by transposed 3D rotation matrix: res = mat' * vec.
    ! MJAPI void mju_rotVecMatT(mjtNum res[3], const mjtNum vec[3], const mjtNum mat[9])
    subroutine mju_rotVecMatT(res, vec, mat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3), mat(9)
    end subroutine mju_rotVecMatT

    !! Compute cross-product: res = cross(a, b).
    ! MJAPI void mju_cross(mjtNum res[3], const mjtNum a[3], const mjtNum b[3])
    subroutine mju_cross(res, a, b) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: a(3), b(3)
    end subroutine mju_cross

    !! Set res = 0.
    ! MJAPI void mju_zero4(mjtNum res[4])
    subroutine mju_zero4(res) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
    end subroutine mju_zero4

    !! Set res = (1,0,0,0).
    ! MJAPI void mju_unit4(mjtNum res[4])
    subroutine mju_unit4(res) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
    end subroutine mju_unit4

    !! Set res = vec.
    ! MJAPI void mju_copy4(mjtNum res[4], const mjtNum data[4])
    subroutine mju_copy4(res, data) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: data(4)
    end subroutine mju_copy4

    !! Normalize vector, return length before normalization.
    ! MJAPI mjtNum mju_normalize4(mjtNum res[4])
    real(mjtNum) function mju_normalize4(res) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
    end function mju_normalize4

    !! Set res = 0.
    ! MJAPI void mju_zero(mjtNum* res, int n)
    subroutine mju_zero(res, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_zero

    !! Set res = vec.
    ! MJAPI void mju_copy(mjtNum* res, const mjtNum* data, int n)
    subroutine mju_copy(res, data, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: data(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_copy

    !! Return sum(vec).
    ! MJAPI mjtNum mju_sum(const mjtNum* vec, int n)
    real(mjtNum) function mju_sum(vec, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end function mju_sum

    !! Return L1 norm: sum(abs(vec)).
    ! MJAPI mjtNum mju_L1(const mjtNum* vec, int n)
    real(mjtNum) function mju_L1(vec, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end function mju_L1

    !! Set res = vec*scl.
    ! MJAPI void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n)
    subroutine mju_scl(res, vec, scl, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      real(mjtNum), value, intent(in)     :: scl
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_scl

    !! Set res = vec1 + vec2.
    ! MJAPI void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n)
    subroutine mju_add(res, vec1, vec2, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec1(*), vec2(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_add

    !! Set res = vec1 - vec2.
    ! MJAPI void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n)
    subroutine mju_sub(res, vec1, vec2, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec1(*), vec2(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_sub

    !! Set res = res + vec.
    ! MJAPI void mju_addTo(mjtNum* res, const mjtNum* vec, int n)
    subroutine mju_addTo(res, vec, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_addTo

    !! Set res = res - vec.
    ! MJAPI void mju_subFrom(mjtNum* res, const mjtNum* vec, int n)
    subroutine mju_subFrom(res, vec, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_subFrom

    !! Set res = res + vec*scl.
    ! MJAPI void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n)
    subroutine mju_addToScl(res, vec, scl, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      real(mjtNum), value, intent(in)     :: scl
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_addToScl

    !! Set res = vec1 + vec2*scl.
    ! MJAPI void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n)
    subroutine mju_addScl(res, vec1, vec2, scl, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: vec1(*), vec2(*)
      real(mjtNum), value, intent(in)     :: scl
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_addScl

    !! Normalize vector, return length before normalization.
    ! MJAPI mjtNum mju_normalize(mjtNum* res, int n)
    real(mjtNum) function mju_normalize(res, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      integer(c_int), value, intent(in)   :: n
    end function mju_normalize

    !! Return vector length (without normalizing vector).
    ! MJAPI mjtNum mju_norm(const mjtNum* res, int n)
    real(mjtNum) function mju_norm(res, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(in)            :: res(*)
      integer(c_int), value, intent(in)   :: n
    end function mju_norm

    !! Return dot-product of vec1 and vec2.
    ! MJAPI mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, const int n)
    real(mjtNum) function mju_dot(vec1, vec2, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(in)            :: vec1(*), vec2(*)
      integer(c_int), value, intent(in)   :: n
    end function mju_dot

    !! Multiply matrix and vector: res = mat * vec.
    ! MJAPI void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                            ! int nr, int nc)
    subroutine mju_mulMatVec(res, mat, vec, nr, nc) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat(*), vec(*)
      integer(c_int), value, intent(in)   :: nr, nc
    end subroutine mju_mulMatVec

    !! Multiply transposed matrix and vector: res = mat' * vec.
    ! MJAPI void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                              ! int nr, int nc)
    subroutine mju_mulMatTVec(res, mat, vec, nr, nc) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat(*), vec(*)
      integer(c_int), value, intent(in)   :: nr, nc
    end subroutine mju_mulMatTVec

    !! Transpose matrix: res = mat'.
    ! MJAPI void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc)
    subroutine mju_transpose(res, mat, nr, nc) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat(*)
      integer(c_int), value, intent(in)   :: nr, nc
    end subroutine mju_transpose

    !! Multiply matrices: res = mat1 * mat2.
    ! MJAPI void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                            ! int r1, int c1, int c2)
    subroutine mju_mulMatMat(res, mat1, mat2, r1, c1, c2) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat1(*), mat2(*)
      integer(c_int), value, intent(in)   :: r1, c1, c2
    end subroutine mju_mulMatMat

    !! Multiply matrices, second argument transposed: res = mat1 * mat2'.
    ! MJAPI void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                              ! int r1, int c1, int r2)
    subroutine mju_mulMatMatT(res, mat1, mat2, r1, c1, r2) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat1(*), mat2(*)
      integer(c_int), value, intent(in)   :: r1, c1, r2
    end subroutine mju_mulMatMatT

    !! Multiply matrices, first argument transposed: res = mat1' * mat2.
    ! MJAPI void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                              ! int r1, int c1, int c2)
    subroutine mju_mulMatTMat(res, mat1, mat2, r1, c1, c2) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat1(*), mat2(*)
      integer(c_int), value, intent(in)   :: r1, c1, c2
    end subroutine mju_mulMatTMat

    !! Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.
    ! MJAPI void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc)
    subroutine mju_sqrMatTD(res, mat, diag, nr, nc) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat(*), diag(*)
      integer(c_int), value, intent(in)   :: nr, nc
    end subroutine mju_sqrMatTD

    !! Coordinate transform of 6D motion or force vector in rotation:translation format.
    !! rotnew2old is 3-by-3, NULL means no rotation flg_force specifies force or motion type.
    ! MJAPI void mju_transformSpatial(mjtNum res[6], const mjtNum vec[6], int flg_force,
                                    ! const mjtNum newpos[3], const mjtNum oldpos[3],
                                    ! const mjtNum rotnew2old[9])
    subroutine mju_transformSpatial(res, vec, flg_force, newpos, oldpos, rotnew2old) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: res(6)
      real(mjtNum), intent(in)            :: vec(6), newpos(3), oldpos(3), rotnew2old(9)
      integer(c_int), value, intent(in)   :: flg_force
    end subroutine mju_transformSpatial


    !---------------------------------- Quaternions ---------------------------------------------------

    !! Rotate vector by quaternion.
    ! MJAPI void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4])
    subroutine mju_rotVecQuat(res, vec, quat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: vec(3), quat(4)
    end subroutine mju_rotVecQuat

    !! Conjugate quaternion, corresponding to opposite rotation.
    ! MJAPI void mju_negQuat(mjtNum res[4], const mjtNum quat[4])
    subroutine mju_negQuat(res, quat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
      real(mjtNum), intent(in)            :: quat(4)
    end subroutine mju_negQuat

    !! Multiply quaternions.
    ! MJAPI void mju_mulQuat(mjtNum res[4], const mjtNum quat1[4], const mjtNum quat2[4])
    subroutine mju_mulQuat(res, quat1, quat2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
      real(mjtNum), intent(in)            :: quat1(4), quat2(4)
    end subroutine mju_mulQuat

    !! Multiply quaternion and axis.
    ! MJAPI void mju_mulQuatAxis(mjtNum res[4], const mjtNum quat[4], const mjtNum axis[3])
    subroutine mju_mulQuatAxis(res, quat, axis) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
      real(mjtNum), intent(in)            :: quat(4), axis(3)
    end subroutine mju_mulQuatAxis

    !! Convert axisAngle to quaternion.
    ! MJAPI void mju_axisAngle2Quat(mjtNum res[4], const mjtNum axis[3], mjtNum angle)
    subroutine mju_axisAngle2Quat(res, axis, angle) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
      real(mjtNum), intent(in)            :: axis(3)
      real(mjtNum), value, intent(in)     :: angle
    end subroutine mju_axisAngle2Quat

    !! Convert quaternion (corresponding to orientation difference) to 3D velocity.
    ! MJAPI void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt)
    subroutine mju_quat2Vel(res, quat, dt) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: quat(4)
      real(mjtNum), value, intent(in)     :: dt
    end subroutine mju_quat2Vel

    !! Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
    ! MJAPI void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4])
    subroutine mju_subQuat(res, qa, qb) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: qa(4), qb(4)
    end subroutine mju_subQuat

    !! Convert quaternion to 3D rotation matrix.
    ! MJAPI void mju_quat2Mat(mjtNum res[9], const mjtNum quat[4])
    subroutine mju_quat2Mat(res, quat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(9)
      real(mjtNum), intent(in)            :: quat(4)
    end subroutine mju_quat2Mat

    !! Convert 3D rotation matrix to quaternion.
    ! MJAPI void mju_mat2Quat(mjtNum quat[4], const mjtNum mat[9])
    subroutine mju_mat2Quat(quat, mat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: quat(4)
      real(mjtNum), intent(in)            :: mat(9)
    end subroutine mju_mat2Quat

    !! Compute time-derivative of quaternion, given 3D rotational velocity.
    ! MJAPI void mju_derivQuat(mjtNum res[4], const mjtNum quat[4], const mjtNum vel[3])
    subroutine mju_derivQuat(res, quat, vel) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(4)
      real(mjtNum), intent(in)            :: quat(4), vel(3)
    end subroutine mju_derivQuat

    !! Integrate quaternion given 3D angular velocity.
    ! MJAPI void mju_quatIntegrate(mjtNum quat[4], const mjtNum vel[3], mjtNum scale)
    subroutine mju_quatIntegrate(quat, vel, scale) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: quat(4)
      real(mjtNum), intent(in)            :: vel(3)
      real(mjtNum), value, intent(in)     :: scale
    end subroutine mju_quatIntegrate

    !! Construct quaternion performing rotation from z-axis to given vector.
    ! MJAPI void mju_quatZ2Vec(mjtNum quat[4], const mjtNum vec[3])
    subroutine mju_quatZ2Vec(quat, vec) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: quat(4)
      real(mjtNum), intent(in)            :: vec(3)
    end subroutine mju_quatZ2Vec


    !---------------------------------- Poses ---------------------------------------------------------

    !! Multiply two poses.
    ! MJAPI void mju_mulPose(mjtNum posres[3], mjtNum quatres[4],
                          ! const mjtNum pos1[3], const mjtNum quat1[4],
                          ! const mjtNum pos2[3], const mjtNum quat2[4])
    subroutine mju_mulPose(posres, quatres, pos1, quat1, pos2, quat2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: posres(3), quatres(4)
      real(mjtNum), intent(in)            :: pos1(3), quat1(4), pos2(3), quat2(4)
    end subroutine mju_mulPose

    !! Conjugate pose, corresponding to the opposite spatial transformation.
    ! MJAPI void mju_negPose(mjtNum posres[3], mjtNum quatres[4],
                          ! const mjtNum pos[3], const mjtNum quat[4])
    subroutine mju_negPose(posres, quatres, pos, quat) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: posres(3), quatres(4)
      real(mjtNum), intent(in)            :: pos(3), quat(4)
    end subroutine mju_negPose

    !! Transform vector by pose.
    ! MJAPI void mju_trnVecPose(mjtNum res[3], const mjtNum pos[3], const mjtNum quat[4],
                              ! const mjtNum vec[3])
    subroutine mju_trnVecPose(res, pos, quat, vec) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(3)
      real(mjtNum), intent(in)            :: pos(3), quat(4), vec(3)
    end subroutine mju_trnVecPose


    !--------------------------------- Decompositions -------------------------------------------------

    !! Cholesky decomposition: mat = L*L' return rank, decomposition performed in-place into mat.
    ! MJAPI int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag)
    integer(c_int) function mju_cholFactor(mat, n, mindiag) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: mat(*)
      integer(c_int), value, intent(in)   :: n
      real(mjtNum), value, intent(in)     :: mindiag
    end function mju_cholFactor

    !! Solve mat * res = vec, where mat is Cholesky-factorized
    ! MJAPI void mju_cholSolve(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n)
    subroutine mju_cholSolve(res, mat, vec, n) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: res(*)
      real(mjtNum), intent(in)            :: mat(*), vec(*)
      real(mjtNum), value, intent(in)     :: n
    end subroutine mju_cholSolve

    !! Cholesky rank-one update: L*L' +/- x*x' return rank.
    ! MJAPI int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus)
    integer(c_int) function mju_cholUpdate(mat, x, n, flg_plus) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: mat(*), x(*)
      integer(c_int), value, intent(in)   :: n, flg_plus
    end function mju_cholUpdate

    !! Eigenvalue decomposition of symmetric 3x3 matrix.
    ! MJAPI int mju_eig3(mjtNum eigval[3], mjtNum eigvec[9], mjtNum quat[4], const mjtNum mat[9])
    integer(c_int) function mju_eig3(eigval, eigvec, quat, mat) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: eigval(3), eigvec(9), quat(4)
      real(mjtNum), intent(in)            :: mat(9)
    end function mju_eig3


    !---------------------- Miscellaneous --------------------------------------------------

    !! Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
    ! MJAPI mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                                ! mjtNum acc0, const mjtNum prm[9])
    real(mjtNum) function mju_muscleGain(len, vel, lengthrange, acc0, prm) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: len, vel, acc0
      real(mjtNum), intent(in)            :: lengthrange(2), prm(9)
    end function mju_muscleGain

    !! Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
    ! MJAPI mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                                ! mjtNum acc0, const mjtNum prm[9])
    real(mjtNum) function mju_muscleBias(len, lengthrange, acc0, prm) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: len, acc0
      real(mjtNum), intent(in)            :: lengthrange(2), prm(9)
    end function mju_muscleBias

    !! Muscle activation dynamics, prm = (tau_act, tau_deact).
    ! MJAPI mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[2])
    real(mjtNum) function mju_muscleDynamics(ctrl, act, prm) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: ctrl, act
      real(mjtNum), intent(in)            :: prm(2)
    end function mju_muscleDynamics

    !! Convert contact force to pyramid representation.
    ! MJAPI void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force,
                                ! const mjtNum* mu, int dim)
    subroutine mju_encodePyramid(pyramid, force, mu, dim) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: pyramid(*)
      real(mjtNum), intent(in)            :: force(*), mu(*)
      integer(c_int), value, intent(in)   :: dim
    end subroutine mju_encodePyramid

    !! Convert pyramid representation to contact force.
    ! MJAPI void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid,
                                ! const mjtNum* mu, int dim)
    subroutine mju_decodePyramid(force, pyramid, mu, dim) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: force(*)
      real(mjtNum), intent(in)            :: pyramid(*), mu(*)
      integer(c_int), value, intent(in)   :: dim
    end subroutine mju_decodePyramid

    !! Integrate spring-damper analytically, return pos(dt).
    ! MJAPI mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt)
    real(mjtNum) function mju_springDamper(pos0, vel0, Kp, Kv, dt) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: pos0, vel0, Kp, Kv, dt
    end function mju_springDamper

    !! Return min(a,b) with single evaluation of a and b.
    ! MJAPI mjtNum mju_min(mjtNum a, mjtNum b)
    real(mjtNum) function mju_min(a, b) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: a, b
    end function mju_min

    !! Return max(a,b) with single evaluation of a and b.
    ! MJAPI mjtNum mju_max(mjtNum a, mjtNum b)
    real(mjtNum) function mju_max(a, b) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: a, b
    end function mju_max

    !! Return sign of x: +1, -1 or 0.
    ! MJAPI mjtNum mju_sign(mjtNum x)
    real(mjtNum) function mju_sign(x) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: x
    end function mju_sign

    !! Round x to nearest integer.
    ! MJAPI int mju_round(mjtNum x)
    integer(c_int) function mju_round(x) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), value, intent(in)     :: x
    end function mju_round

    !! Convert type id (mjtObj) to type name.
    ! MJAPI const char* mju_type2Str(int type)
    character(c_char) function mju_type2Str(type) bind(c)
      import :: c_char, c_int
      integer(c_int), value, intent(in)   :: type
    end function mju_type2Str

    !! Convert type name to type id (mjtObj).
    ! MJAPI int mju_str2Type(const char* str)
    integer(c_int) function mju_str2Type(str) bind(c)
      import :: c_int, c_char
      character(len=1, kind=c_char), dimension(*), intent(in) :: str
    end function mju_str2Type

    !! Construct a warning message given the warning type and info.
    ! MJAPI const char* mju_warningText(int warning, int info)
    character(c_char) function mju_warningText(warning, info) bind(c)
      import :: c_char, c_int
      integer(c_int), value, intent(in)   :: warning, info
    end function mju_warningText

    !! Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.
    ! MJAPI int mju_isBad(mjtNum x)
    integer(c_int) function mju_isBad(x) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), value, intent(in)     :: x
    end function mju_isBad

    !! Return 1 if all elements are 0.
    ! MJAPI int mju_isZero(mjtNum* vec, int n)
    integer(c_int) function mju_isZero(vec, int) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: vec(*)
      integer(c_int), value, intent(in)   :: int
    end function mju_isZero

    !! Standard normal random number generator (optional second number).
    ! MJAPI mjtNum mju_standardNormal(mjtNum* num2)
    real(mjtNum) function mju_standardNormal(num2) bind(c)
      import :: mjtNum
      real(mjtNum), intent(inout)         :: num2(*)
    end function mju_standardNormal

    !! Convert from float to mjtNum.
    ! MJAPI void mju_f2n(mjtNum* res, const float* vec, int n)
    subroutine mju_f2n(res, vec, n) bind(c)
      import :: mjtNum, c_float, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(c_float), intent(in)           :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_f2n

    !! Convert from mjtNum to float.
    ! MJAPI void mju_n2f(float* res, const mjtNum* vec, int n)
    subroutine mju_n2f(res, vec, n) bind(c)
      import :: mjtNum, c_float, c_int
      real(c_float), intent(inout)        :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_n2f

    !! Convert from double to mjtNum.
    ! MJAPI void mju_d2n(mjtNum* res, const double* vec, int n)
    subroutine mju_d2n(res, vec, n) bind(c)
      import :: mjtNum, c_double, c_int
      real(mjtNum), intent(inout)         :: res(*)
      real(c_double), intent(in)          :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_d2n

    !! Convert from mjtNum to double.
    ! MJAPI void mju_n2d(double* res, const mjtNum* vec, int n)
    subroutine mju_n2d(res, vec, n) bind(c)
      import :: mjtNum, c_double, c_int
      real(c_double), intent(inout)       :: res(*)
      real(mjtNum), intent(in)            :: vec(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_n2d

    !! Insertion sort, resulting list is in increasing order.
    ! MJAPI void mju_insertionSort(mjtNum* list, int n)
    subroutine mju_insertionSort(list, n) bind(c)
      import :: mjtNum, c_int
      real(mjtNum), intent(inout)         :: list(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_insertionSort

    !! Integer insertion sort, resulting list is in increasing order.
    ! MJAPI void mju_insertionSortInt(int* list, int n)
    subroutine mju_insertionSortInt(list, n) bind(c)
      import :: c_int
      integer(c_int), intent(inout)       :: list(*)
      integer(c_int), value, intent(in)   :: n
    end subroutine mju_insertionSortInt

    !! Generate Halton sequence.
    ! MJAPI mjtNum mju_Halton(int index, int base)
    real(mjtNum) function mju_Halton(index, base) bind(c)
      import :: mjtNum, c_int
      integer(c_int), value, intent(in)   :: index, base
    end function mju_Halton

    !! Call strncpy, then set dst[n-1] = 0.
    ! MJAPI char* mju_strncpy(char *dst, const char *src, int n)
    character(c_char) function mju_strncpy(dst, src, n) bind(c)
      import :: c_char, c_int
      character(len=1, kind=c_char), dimension(*), intent(out) :: dst
      character(len=1, kind=c_char), dimension(*), intent(in)  :: src
      integer(c_int), value, intent(in)   :: n
    end function mju_strncpy

    !! Sigmoid function over 0<=x<=1 constructed from half-quadratics.
    ! MJAPI mjtNum mju_sigmoid(mjtNum x)
    real(mjtNum) function mju_sigmoid(x) bind(c)
      import :: mjtNum
      real(mjtNum), value, intent(in)     :: x
    end function mju_sigmoid
  end interface

contains

! subroutine render(viewport, scn, con)
!   type(mjrRect), intent(inout)        :: viewport
!   type(mjvScene), intent(inout)       :: scn
!   type(mjrContext), intent(in)        :: con

!   integer(c_int)          :: stereo, nt, ngeom, nlight
!   integer(c_int)          :: drawbuffer
!   type(mjvGLCamera)       :: cam
!   real(c_double)          :: hpos(0:2), hfwd(0:2)
!   real(c_float)           :: temp(0:3), headpos(0:2), forward(0:2), skyboxdst
!   real(c_float)           :: camProject(0:15), camView(0:15), lightProject(0:15), lightView(0:15)
!   real(c_double)          :: clipplane(0:3)
!   real(c_float)           :: biasMatrix(0:15)
!   real(c_float)           :: tempMatrix(0:15), textureMatrix(0:15)
!   type(mjvGeom)           :: thisgeom, tempgeom
!   type(mjvLight)          :: thislight

!   integer(c_int)          :: nskin_index
!   ! integer(GLuint), pointer:: skinvertVBO(:)


!   ngeom  = scn%ngeom
!   nlight = min( mjMAXLIGHT, scn%nlight )
!   biasMatrix    = 0
!   biasMatrix(0) = 0.5_c_float
!   biasMatrix(5) = 0.5_c_float
!   biasMatrix(10) = 0.5_c_float
!   biasMatrix(12) = 0.5_c_float
!   biasMatrix(13) = 0.5_c_float
!   biasMatrix(14) = 0.5_c_float
!   biasMatrix(15) = 1.0_c_float

!   ! Empty viewport: nothing to do
!   if ( (viewport%width <= 0) .or. (viewport%height <= 0) ) return

!   ! Aerage cameras
!   cam = mjv_averageCamera( scn%camera(1), scn%camera(2) )

!   print *, "cam%frustum_near: ", cam%frustum_near
  
!   ! Check znear
!   if ( cam%frustum_near < mjMINVAL ) then
!     !! geoms: error
!     if (scn%ngeom > 0) then
!       call mju_error( "mjvScene frustum_near too small in mjr_render"//c_null_char )
!     else
!       !! no geoms: return silently
!       return
!     end if
!   end if

! !   ! Upload dynamic skin data to GPU
! !   call c_f_pointer( con%skinvertVBO, skinvertVBO, [scn%nskin] );

! !   scn%skinv
! !   print *, "skinvertVBO = ", scn%nskin !skinvertVBO
! !   stop
! !   do nskin_index = 1, scn%nskin
! !     !! Upload positions to VBO
! !     call glBindBuffer( GL_ARRAY_BUFFER, skinvertVBO(nskin_index) )
! !     call glBufferData( GL_ARRAY_BUFFER,
! !                3*scn->skinvertnum[i]*sizeof(float),
! ! !                scn->skinvert + 3*scn->skinvertadr[i],
! ! !                GL_STREAM_DRAW);

! ! !   // upload normals to VBO
! ! !   glBindBuffer(GL_ARRAY_BUFFER, con->skinnormalVBO[i]);
! ! !   glBufferData(GL_ARRAY_BUFFER,
! ! !                3*scn->skinvertnum[i]*sizeof(float),
! ! !                scn->skinnormal + 3*scn->skinvertadr[i],
! ! !                GL_STREAM_DRAW);
! ! ! }
! !   end do

!   print *, "Exiting RENDER subroutine"
! end subroutine render
  
end module mod_mujoco