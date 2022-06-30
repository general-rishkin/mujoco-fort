program basis
  use iso_c_binding
  use mod_glfw
  use mod_mj
  use mod_mjdata
  use mod_mjvisualize
  use mod_mjrender
  implicit none
  
  character(len=1, kind=c_char) :: loaderror(1000) = "Could not load binary model"//c_null_char
  character(len=*, kind=c_char), parameter :: filename &
                    ="/home/ismail/Workspace/FORTRAN/mujoco-fort/src/assets/ball.xml"//c_null_char
  type(c_ptr) :: m, d, window !, opt
  type(mjvCamera), target :: cam
  type(mjvOption), target :: opt
  type(mjvScene), target :: scn
  type(mjrContext), target :: con
  type(mjvPerturb)        :: pert

  type(mjData), pointer   :: data
  type(mjrRect)           :: viewport

  real(c_double)          :: sim_start
  

  m = mj_load_xml(filename, c_null_ptr, loaderror, 1000)
  if (.not. c_associated(m) ) call mju_error_s("Load model error: %s"//c_null_char, loaderror)

  d = mj_make_data(m)
  if ( .not. c_associated(d) ) call mju_error_s("Load data error: %s"//c_null_char, loaderror)

  if (glfwInit() == GLFW_FALSE) call mju_error("Could not initialize GLFW")

  window = glfwCreateWindow(1200, 900, "Demo"//c_null_char, c_null_ptr, c_null_ptr)
  if ( .not. c_associated(window) ) call mju_error_s("Could not create GLFW Window!!!"//c_null_char, loaderror)
  
  call glfwMakeContextCurrent(window)
  call glfwSwapInterval(1_c_int)

  ! Initialize visualization data structures
  call mjv_default_camera(c_loc(cam))
  call mjv_default_option(c_loc(opt))
  call mjv_default_scene(c_loc(scn))
  call mjr_default_context(c_loc(con))

  ! Create scene and context
  call mjv_make_scene(m, c_loc(scn), 2000);
  call mjr_make_context(m, c_loc(con), mjFONTSCALE_150);

  call c_f_pointer(d, data)

  do while ( glfwWindowShouldClose(window) == 0)
    sim_start = data%time
    do while ( (data%time - sim_start) < (1.0/60.0) )
      call mj_step( m, d )
    end do

    ! Get framebuffer viewport
    viewport = mjrRect( 0_c_int, 0_c_int, 0_c_int, 0_c_int )
    call glfwGetFramebufferSize( window, viewport%width, viewport%height )

  !   ! Update scene and render
    call mjv_update_scene(m, d, c_loc(opt), c_null_ptr, c_loc(cam), mjCAT_ALL, c_loc(scn))

    call mjr_render(viewport, c_loc(scn), c_loc(con) )

    call glfwSwapBuffers(window)

    call glfwPollEvents()
  end do

end program basis