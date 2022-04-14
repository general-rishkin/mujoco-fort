module mod_mjtnum
  use, intrinsic  :: iso_c_binding, only: c_int, c_float, c_double
  implicit none

  ! compile-time configuration options
  integer(c_int)        :: mjUSEDOUBLE = 1        !! single or double precision for mjtNum
  integer, parameter    :: f32        = c_float
  integer, parameter    :: mjtNum     = c_double
  
  real(mjtNum), parameter  :: mjMINVAL = 1E-15       !! minimum value in any denominator
end module mod_mjtnum