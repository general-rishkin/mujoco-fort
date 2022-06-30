program main_test_speed
  use iso_c_binding
  use iso_fortran_env, only: f64 => real64 
  use mod_mujoco
  use mod_glfw
  implicit none
  
  integer, parameter  :: n_step     = 10000
  integer, parameter  :: n_thread   = 1 
  real(f64)           :: ctrl_noise = 0.01

  character(kind=c_char) :: filename(60) = "/home/ismail/Workspace/FORTRAN/mujoco-fort/app/ball.xml"//c_null_char
  character(len=1, kind=c_char) :: loaderror(1000) = "Could not load binary model"//c_null_char
  integer(c_int), parameter :: error_sz = 1000_c_int
  
  integer :: id
  type(c_ptr) :: model_ptr = c_null_ptr
  type(c_ptr) :: data_ptr = c_null_ptr
  type(mjModel), pointer :: m
  type(mjData), pointer  :: d

  integer               :: contacts, constraints

  ! load and compile model
  model_ptr = mj_loadXML(&
            filename="/home/ismail/Workspace/FORTRAN/mujoco-fort/src/assets/ball.xml"//c_null_char, &
            error=loaderror, error_sz=error_sz)
  if ( .not. c_associated(model_ptr) ) call mju_error_s("Load model error: %s"//c_null_char, loaderror)
  call c_f_pointer(model_ptr, m)

  ! Make per-thread data
  data_ptr = mj_makeData( m )
  if ( .not. c_associated(data_ptr) ) call mju_error_s("Load data error: %s"//c_null_char, loaderror)
  call c_f_pointer(data_ptr, d)

  do id = 0, n_thread-1
    call simulate( id, n_step, ctrl_noise )
  end do

  print *, "Contacts per step   : ", contacts/real(n_step)
  print *, "Constraints per step: ", constraints/real(n_step)
  print *, "Degrees of freedom  :", m%nv

contains

  subroutine simulate( id, n_step, ctrl_noise )
    integer, intent(in)   :: id, n_step
    real(f64), intent(in) :: ctrl_noise

    integer               :: i

    contacts = 0
    constraints = 0

    do i = 0, n_step-1
      call mj_step( m, d )

      contacts = contacts + d%ncon
      constraints = constraints + d%nefc
    end do

  end subroutine simulate


end program main_test_speed