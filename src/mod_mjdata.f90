module mod_mjdata
  use iso_c_binding
  use mod_mjtnum
  use mod_mjmodel
  implicit none
  
  !---------------------------------- primitive types (mjt) -----------------------------------------

  enum, bind(c) !! mjtWarning_ {        !! warning types
    enumerator :: mjWARN_INERTIA      = 0       !! (near) singular inertia matrix
    enumerator :: mjWARN_CONTACTFULL            !! too many contacts in contact list
    enumerator :: mjWARN_CNSTRFULL              !! too many constraints
    enumerator :: mjWARN_VGEOMFULL              !! too many visual geoms
    enumerator :: mjWARN_BADQPOS                !! bad number in qpos
    enumerator :: mjWARN_BADQVEL                !! bad number in qvel
    enumerator :: mjWARN_BADQACC                !! bad number in qacc
    enumerator :: mjWARN_BADCTRL                !! bad number in ctrl
    enumerator :: mjNWARNING                    !! number of warnings
  end enum

  enum, bind(c) !! mjtTimer_ {
    !! main api
    enumerator :: mjTIMER_STEP        = 0        !! step
    enumerator :: mjTIMER_FORWARD                !! forward
    enumerator :: mjTIMER_INVERSE                !! inverse

    !! breakdown of step/forward
    enumerator :: mjTIMER_POSITION               !! fwdPosition
    enumerator :: mjTIMER_VELOCITY               !! fwdVelocity
    enumerator :: mjTIMER_ACTUATION              !! fwdActuation
    enumerator :: mjTIMER_ACCELERATION           !! fwdAcceleration
    enumerator :: mjTIMER_CONSTRAINT             !! fwdConstraint

    !! breakdown of fwdPosition
    enumerator :: mjTIMER_POS_KINEMATICS         !! kinematics com tendon transmission
    enumerator :: mjTIMER_POS_INERTIA            !! inertia computations
    enumerator :: mjTIMER_POS_COLLISION          !! collision detection
    enumerator :: mjTIMER_POS_MAKE               !! make constraints
    enumerator :: mjTIMER_POS_PROJECT            !! project constraints

    enumerator :: mjNTIMER                        !! number of timers
  end enum

  !---------------------------------- mjContact -----------------------------------------------------

  type, bind(c) :: mjContact               !! result of collision detection functions
    !! contact parameters set by geom-specific collision detector
    real(mjtNum)              :: dist                    !! distance between nearest points neg: penetration
    real(mjtNum)              :: pos(3)                  !! position of contact point: midpoint between geoms
    real(mjtNum)              :: frame(9)                !! normal is in [0-2]

    !! contact parameters set by mj_collideGeoms
    real(mjtNum)              :: includemargin           !! include if dist<includemargin=margin-gap
    real(mjtNum)              :: friction(5)             !! tangent1 2 spin roll1 2
    real(mjtNum)              :: solref(mjNREF)          !! constraint solver reference
    real(mjtNum)              :: solimp(mjNIMP)          !! constraint solver impedance

    !! internal storage used by solver
    real(mjtNum)              :: mu                      !! friction of regularized cone set by mj_makeConstraint
    real(mjtNum)              :: H(36)                   !! cone Hessian set by mj_updateConstraint

    !! contact descriptors set by mj_collideGeoms
    integer(c_int)            :: dim                        !! contact space dimensionality: 1 3 4 or 6
    integer(c_int)            :: geom1                      !! id of geom 1
    integer(c_int)            :: geom2                      !! id of geom 2

    !! flag set by mj_fuseContact or mj_instantianteEquality
    integer(c_int)            :: exclude                    !! 0: include 1: in gap 2: fused 3: equality 4: no dofs

    !! address computed by mj_instantiateContact
    integer(c_int)            :: efc_address                !! address in efc -1: not included -2-i: distance constraint i
  end type mjContact


  !---------------------------------- diagnostics ---------------------------------------------------

  type, bind(c) :: mjWarningStat           !! warning statistics
    integer(c_int)            :: lastinfo                   !! info from last warning
    integer(c_int)            :: number                     !! how many times was warning raised
  end type mjWarningStat


  type, bind(c) :: mjTimerStat             !! timer statistics
    real(mjtNum)              :: duration                !! cumulative duration
    integer(c_int)            :: number                     !! how many times was timer called
  end type mjTimerStat


  type, bind(c) :: mjSolverStat            !! per-iteration solver statistics
    real(mjtNum)              :: improvement             !! cost reduction, scaled by 1/trace(M(qpos0))
    real(mjtNum)              :: gradient                !! gradient norm (primal only, scaled)
    real(mjtNum)              :: lineslope               !! slope in linesearch
    integer(c_int)            :: nactive                    !! number of active constraints
    integer(c_int)            :: nchange                    !! number of constraint state changes
    integer(c_int)            :: neval                      !! number of cost evaluations in line search
    integer(c_int)            :: nupdate                    !! number of Cholesky updates in line search
  end type mjSolverStat


  !---------------------------------- mjData --------------------------------------------------------

  type, bind(c) :: mjData
    !! constant sizes
    integer(c_int)            :: nstack                     !! number of mjtNums that can fit in stack
    integer(c_int)            :: nbuffer                    !! size of main buffer in bytes

    !! stack pointer
    integer(c_int)            :: pstack                     !! first available mjtNum address in stack

    !! memory utilization stats
    integer(c_int)            :: maxuse_stack               !! maximum stack allocation
    integer(c_int)            :: maxuse_con                 !! maximum number of contacts
    integer(c_int)            :: maxuse_efc                 !! maximum number of scalar constraints

    !! diagnostics
    type(mjWarningStat)       :: warning(mjNWARNING) !! warning statistics
    type(mjTimerStat)         :: timer(mjNTIMER)    !! timer statistics
    type(mjSolverStat)        :: solver(mjNSOLVER) !! solver statistics per iteration
    integer(c_int)            :: solver_iter                !! number of solver iterations
    integer(c_int)            :: solver_nnz                 !! number of non-zeros in Hessian or efc_AR
    real(mjtNum)              :: solver_fwdinv(2)        !! forward-inverse comparison: qfrc, efc

    !! variable sizes
    integer(c_int)            :: ne                         !! number of equality constraints
    integer(c_int)            :: nf                         !! number of friction constraints
    integer(c_int)            :: nefc                       !! number of constraints
    integer(c_int)            :: ncon                       !! number of detected contacts

    !! global properties
    real(mjtNum)              :: time                    !! simulation time
    real(mjtNum)              :: energy(2)               !! potential, kinetic energy

    !!-------------------------------- end of info header

    !! buffers
    type(c_ptr)               :: buffer               !! main buffer all pointers point in it    (nbuffer bytes)
    type(c_ptr)               :: stack                !! stack buffer                             (nstack mjtNums)

    !!-------------------------------- main inputs and outputs of the computation

    !! state
    type(c_ptr)               :: qpos                 !! position                                 (nq x 1)
    type(c_ptr)               :: qvel                 !! velocity                                 (nv x 1)
    type(c_ptr)               :: act                  !! actuator activation                      (na x 1)
    type(c_ptr)               :: qacc_warmstart       !! acceleration used for warmstart          (nv x 1)

    !! control
    type(c_ptr)               :: ctrl                 !! control                                  (nu x 1)
    type(c_ptr)               :: qfrc_applied         !! applied generalized force                (nv x 1)
    type(c_ptr)               :: xfrc_applied         !! applied Cartesian force/torque           (nbody x 6)

    !! mocap data
    type(c_ptr)               :: mocap_pos            !! positions of mocap bodies                (nmocap x 3)
    type(c_ptr)               :: mocap_quat           !! orientations of mocap bodies             (nmocap x 4)

    !! dynamics
    type(c_ptr)               :: qacc                 !! acceleration                             (nv x 1)
    type(c_ptr)               :: act_dot              !! time-derivative of actuator activation   (na x 1)

    !! user data
    type(c_ptr)               :: userdata             !! user data, not touched by engine         (nuserdata x 1)

    !! sensors
    type(c_ptr)               :: sensordata           !! sensor data array                        (nsensordata x 1)

    !!-------------------------------- POSITION dependent

    !! computed by mj_fwdPosition/mj_kinematics
    type(c_ptr)               :: xpos                 !! Cartesian position of body frame         (nbody x 3)
    type(c_ptr)               :: xquat                !! Cartesian orientation of body frame      (nbody x 4)
    type(c_ptr)               :: xmat                 !! Cartesian orientation of body frame      (nbody x 9)
    type(c_ptr)               :: xipos                !! Cartesian position of body com           (nbody x 3)
    type(c_ptr)               :: ximat                !! Cartesian orientation of body inertia    (nbody x 9)
    type(c_ptr)               :: xanchor              !! Cartesian position of joint anchor       (njnt x 3)
    type(c_ptr)               :: xaxis                !! Cartesian joint axis                     (njnt x 3)
    type(c_ptr)               :: geom_xpos            !! Cartesian geom position                  (ngeom x 3)
    type(c_ptr)               :: geom_xmat            !! Cartesian geom orientation               (ngeom x 9)
    type(c_ptr)               :: site_xpos            !! Cartesian site position                  (nsite x 3)
    type(c_ptr)               :: site_xmat            !! Cartesian site orientation               (nsite x 9)
    type(c_ptr)               :: cam_xpos             !! Cartesian camera position                (ncam x 3)
    type(c_ptr)               :: cam_xmat             !! Cartesian camera orientation             (ncam x 9)
    type(c_ptr)               :: light_xpos           !! Cartesian light position                 (nlight x 3)
    type(c_ptr)               :: light_xdir           !! Cartesian light direction                (nlight x 3)

    !! computed by mj_fwdPosition/mj_comPos
    type(c_ptr)               :: subtree_com          !! center of mass of each subtree           (nbody x 3)
    type(c_ptr)               :: cdof                 !! com-based motion axis of each dof        (nv x 6)
    type(c_ptr)               :: cinert               !! com-based body inertia and mass          (nbody x 10)

    !! computed by mj_fwdPosition/mj_tendon
    type(c_ptr)               :: ten_wrapadr          !! start address of tendon's path           (ntendon x 1)
    type(c_ptr)               :: ten_wrapnum          !! number of wrap points in path            (ntendon x 1)
    type(c_ptr)               :: ten_J_rownnz         !! number of non-zeros in Jacobian row      (ntendon x 1)
    type(c_ptr)               :: ten_J_rowadr         !! row start address in colind array        (ntendon x 1)
    type(c_ptr)               :: ten_J_colind         !! column indices in sparse Jacobian        (ntendon x nv)
    type(c_ptr)               :: ten_length           !! tendon lengths                           (ntendon x 1)
    type(c_ptr)               :: ten_J                !! tendon Jacobian                          (ntendon x nv)
    type(c_ptr)               :: wrap_obj             !! geom id -1: site -2: pulley            (nwrap*2 x 1)
    type(c_ptr)               :: wrap_xpos            !! Cartesian 3D points in all path          (nwrap*2 x 3)

    !! computed by mj_fwdPosition/mj_transmission
    type(c_ptr)               :: actuator_length      !! actuator lengths                         (nu x 1)
    type(c_ptr)               :: actuator_moment      !! actuator moments                         (nu x nv)

    !! computed by mj_fwdPosition/mj_crb
    type(c_ptr)               :: crb                  !! com-based composite inertia and mass     (nbody x 10)
    type(c_ptr)               :: qM                   !! total inertia                            (nM x 1)

    !! computed by mj_fwdPosition/mj_factorM
    type(c_ptr)               :: qLD                  !! L'*D*L factorization of M                (nM x 1)
    type(c_ptr)               :: qLDiagInv            !! 1/diag(D)                                (nv x 1)
    type(c_ptr)               :: qLDiagSqrtInv        !! 1/sqrt(diag(D))                          (nv x 1)

    !! computed by mj_fwdPosition/mj_collision
    type(c_ptr)               :: contact             !! list of all detected contacts            (nconmax x 1)

    !! computed by mj_fwdPosition/mj_makeConstraint
    type(c_ptr)               :: efc_type             !! constraint type (mjtConstraint)          (njmax x 1)
    type(c_ptr)               :: efc_id               !! id of object of specified type           (njmax x 1)
    type(c_ptr)               :: efc_J_rownnz         !! number of non-zeros in Jacobian row      (njmax x 1)
    type(c_ptr)               :: efc_J_rowadr         !! row start address in colind array        (njmax x 1)
    type(c_ptr)               :: efc_J_rowsuper       !! number of subsequent rows in supernode   (njmax x 1)
    type(c_ptr)               :: efc_J_colind         !! column indices in Jacobian               (njmax x nv)
    type(c_ptr)               :: efc_JT_rownnz        !! number of non-zeros in Jacobian row    T (nv x 1)
    type(c_ptr)               :: efc_JT_rowadr        !! row start address in colind array      T (nv x 1)
    type(c_ptr)               :: efc_JT_rowsuper      !! number of subsequent rows in supernode T (nv x 1)
    type(c_ptr)               :: efc_JT_colind        !! column indices in Jacobian             T (nv x njmax)
    type(c_ptr)               :: efc_J                !! constraint Jacobian                      (njmax x nv)
    type(c_ptr)               :: efc_JT               !! constraint Jacobian transposed           (nv x njmax)
    type(c_ptr)               :: efc_pos              !! constraint position (equality, contact)  (njmax x 1)
    type(c_ptr)               :: efc_margin           !! inclusion margin (contact)               (njmax x 1)
    type(c_ptr)               :: efc_frictionloss     !! frictionloss (friction)                  (njmax x 1)
    type(c_ptr)               :: efc_diagApprox       !! approximation to diagonal of A           (njmax x 1)
    type(c_ptr)               :: efc_KBIP             !! stiffness, damping, impedance, imp'      (njmax x 4)
    type(c_ptr)               :: efc_D                !! constraint mass                          (njmax x 1)
    type(c_ptr)               :: efc_R                !! inverse constraint mass                  (njmax x 1)

    !! computed by mj_fwdPosition/mj_projectConstraint
    type(c_ptr)               :: efc_AR_rownnz        !! number of non-zeros in AR                (njmax x 1)
    type(c_ptr)               :: efc_AR_rowadr        !! row start address in colind array        (njmax x 1)
    type(c_ptr)               :: efc_AR_colind        !! column indices in sparse AR              (njmax x njmax)
    type(c_ptr)               :: efc_AR               !! J*inv(M)*J' + R                          (njmax x njmax)

    !!-------------------------------- POSITION, VELOCITY dependent

    !! computed by mj_fwdVelocity
    type(c_ptr)               :: ten_velocity         !! tendon velocities                        (ntendon x 1)
    type(c_ptr)               :: actuator_velocity    !! actuator velocities                      (nu x 1)

    !! computed by mj_fwdVelocity/mj_comVel
    type(c_ptr)               :: cvel                 !! com-based velocity [3D rot 3D tran]     (nbody x 6)
    type(c_ptr)               :: cdof_dot             !! time-derivative of cdof                  (nv x 6)

    !! computed by mj_fwdVelocity/mj_rne (without acceleration)
    type(c_ptr)               :: qfrc_bias            !! C(qpos,qvel)                             (nv x 1)

    !! computed by mj_fwdVelocity/mj_passive
    type(c_ptr)               :: qfrc_passive         !! passive force                            (nv x 1)

    !! computed by mj_fwdVelocity/mj_referenceConstraint
    type(c_ptr)               :: efc_vel              !! velocity in constraint space: J*qvel     (njmax x 1)
    type(c_ptr)               :: efc_aref             !! reference pseudo-acceleration            (njmax x 1)

    !! computed by mj_sensorVel/mj_subtreeVel if needed
    type(c_ptr)               :: subtree_linvel       !! linear velocity of subtree com           (nbody x 3)
    type(c_ptr)               :: subtree_angmom       !! angular momentum about subtree com       (nbody x 3)

    type(c_ptr)               :: D_rownnz
    type(c_ptr)               :: D_rowadr
    type(c_ptr)               :: D_colind
    type(c_ptr)               :: qDeriv
    type(c_ptr)               :: qLU
    !!-------------------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

    !! computed by mj_fwdActuation
    type(c_ptr)               :: actuator_force       !! actuator force in actuation space        (nu x 1)
    type(c_ptr)               :: qfrc_actuator        !! actuator force                           (nv x 1)

    !! computed by mj_fwdAcceleration
    type(c_ptr)               :: qfrc_smooth             !! net unconstrained force                  (nv x 1)
    type(c_ptr)               :: qacc_smooth             !! unconstrained acceleration               (nv x 1)

    !! computed by mj_fwdConstraint/mj_inverse
    type(c_ptr)               :: efc_b                !! linear cost term: J*qacc_unc - aref      (njmax x 1)
    type(c_ptr)               :: efc_force            !! constraint force in constraint space     (njmax x 1)
    type(c_ptr)               :: efc_state            !! constraint state (mjtConstraintState)    (njmax x 1)
    type(c_ptr)               :: qfrc_constraint      !! constraint force                         (nv x 1)

    !! computed by mj_inverse
    type(c_ptr)               :: qfrc_inverse         !! net external force should equal:        (nv x 1)
                                    !! qfrc_applied + J'*xfrc_applied + qfrc_actuator

    !! computed by mj_sensorAcc/mj_rnePostConstraint if needed rotation:translation format
    type(c_ptr)               :: cacc                 !! com-based acceleration                   (nbody x 6)
    type(c_ptr)               :: cfrc_int             !! com-based interaction force with parent  (nbody x 6)
    type(c_ptr)               :: cfrc_ext             !! com-based external force on body         (nbody x 6)
  end type mjData


  !---------------------------------- callback function types ---------------------------------------
  
  abstract interface
    ! generic MuJoCo function
    !! typedef void (*mjfGeneric)(const mjModel* m, mjData* d);
    subroutine mjfGeneric(m, d) bind(c)
      import :: mjModel, mjData
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
    end subroutine mjfGeneric

    ! contact filter: 1- discard, 0- collide
    !! typedef int (*mjfConFilt)(const mjModel* m, mjData* d, int geom1, int geom2);
    integer(c_int) function mjfConFilt(m, d, geom1, geom2) bind(c)
      import :: c_int, mjModel, mjData
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: geom1, geom2
    end function mjfConFilt

    ! sensor simulation
    !! typedef void (*mjfSensor)(const mjModel* m, mjData* d, int stage);
    subroutine mjfSensor(m, d, stage) bind(c)
      import :: mjModel, mjData, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(inout)       :: d
      integer(c_int), value, intent(in) :: stage
    end subroutine mjfSensor

    ! timer
    !! typedef mjtNum (*mjfTime)(void);
    real(mjtNum) function mjfTime() bind(c)
      import :: mjtNum
    end function mjfTime

    ! actuator dynamics, gain, bias
    !! typedef mjtNum (*mjfAct)(const mjModel* m, const mjData* d, int id);
    real(mjtNum) function mjfAct(m, d, id) bind(c)
      import :: mjtNum, mjModel, mjData, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(in)          :: d
      integer(c_int), intent(in), value :: id
    end function mjfAct

    ! collision detection
    !! typedef int (*mjfCollision)(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);
    integer(c_int) function mjfCollision(m, d, con, g1, g2, margin) bind(c)
      import :: mjModel, mjData, mjContact, mjtNum, c_int
      type(mjModel), intent(in)         :: m
      type(mjData), intent(in)          :: d
      type(mjContact), intent(inout)    :: con
      integer(c_int), value, intent(in) :: g1, g2
      real(mjtNum), value, intent(in)   :: margin
    end function mjfCollision
  end interface

end module mod_mjdata