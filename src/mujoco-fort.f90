module mujoco_fort
  implicit none
  private

  public :: say_hello
contains
  subroutine say_hello
    print *, "Hello, mujoco-fort!"
  end subroutine say_hello
end module mujoco_fort
